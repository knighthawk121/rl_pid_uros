#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rl_pid_uros.action import TunePID
import numpy as np
from rl_pid_uros_py.q_learner import QLearningAgent
from rl_pid_uros_py.rl_agent import SARSAAgent, DQNAgent
import random
import asyncio
import sys
import torch

class MotorPIDTuner(Node):
    def __init__(self):
        super().__init__('rl_pid_tuner')
        
        # Get agent selection from user
        self.agent = self.select_agent()

        self.callback_group = ReentrantCallbackGroup()
        self.action_client = ActionClient(
            self,
            TunePID,
            'tune_pid_action',
            callback_group=self.callback_group
        )
        
        self.current_goal_handle = None
        self.goal_active = False
        self.latest_feedback = None
        self.feedback_history = []
        self.goal_timeout = 30.0  # 30 second timeout
        
        self.initial_kp = 0.5  # Start with non-zero values
        self.initial_ki = 0.0
        self.initial_kd = 0.01

        self.get_logger().info(f'Q-learning agent initialized with {self.agent.episode_count} previous episodes')
        if self.agent.best_pid_values is not None:
            self.get_logger().info(f'Best known PID values - Kp: {self.agent.best_pid_values[0]:.4f}, '
                                f'Ki: {self.agent.best_pid_values[1]:.4f}, '
                                f'Kd: {self.agent.best_pid_values[2]:.4f}')
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available!')

        # Create timer for sending new goals with longer interval
        self.timer = self.create_timer(10.0, self.send_new_goal_wrapper)
        self.get_logger().info('Timer started for sending goals every 10 seconds')

    def select_agent(self):
        while True:
            print("\nSelect RL agent type:")
            print("1. Q-Learning")
            print("2. SARSA")
            print("3. DQN (Deep Q-Network)")
            
            try:
                choice = input("Enter choice (1-3): ").strip()
                if choice == "1":
                    return QLearningAgent()
                elif choice == "2":
                    return SARSAAgent()
                elif choice == "3":
                    return DQNAgent()
                else:
                    print("Invalid choice. Please select 1-3.")
            except Exception as e:
                print(f"Error selecting agent: {str(e)}")
                sys.exit(1)
                
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.latest_feedback = feedback
        self.feedback_history.append({
            'error': feedback.current_error,
            'position': feedback.current_position,
            'target': feedback.target_position,
            'timestamp': self.get_clock().now().to_msg()
        })
        
        # Log every 5th feedback for monitoring
        if len(self.feedback_history) % 5 == 0:
            self.get_logger().info(
                f'Current Error: {feedback.current_error:.2f}, '
                f'Position: {feedback.current_position}, '
                f'Target: {feedback.target_position}'
            )

    async def send_new_goal_wrapper(self):
        if not self.goal_active:
            await self.send_new_goal()

    async def send_new_goal(self):
        if self.goal_active:
            self.get_logger().warn('Previous goal still active, canceling...')
            if self.current_goal_handle:
                await self.current_goal_handle.cancel_goal_async()
            self.goal_active = False
            await asyncio.sleep(1.0)
            
        try:
            state = self.get_current_state()
            if len(self.feedback_history) < 2:
                kp, ki, kd = self.initial_kp, self.initial_ki, self.initial_kd
            else:
                kp, ki, kd = self.agent.get_pid_values(state)
            
            target_position = random.randint(0, 180)
            
            goal_msg = TunePID.Goal()
            goal_msg.kp = float(kp)
            goal_msg.ki = float(ki)
            goal_msg.kd = float(kd)
            goal_msg.target_position = target_position
            
            self.get_logger().info(
                f'Sending new goal - kp: {kp:.4f}, ki: {ki:.4f}, kd: {kd:.4f}, '
                f'target: {target_position}'
            )
            
            self.feedback_history = []
            send_goal_future = await self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            if not send_goal_future.accepted:
                self.get_logger().error('Goal was rejected!')
                return

            self.current_goal_handle = send_goal_future
            self.goal_active = True
            
            try:
                get_result_future = await send_goal_future.get_result_async()
                result = get_result_future.result
                
                if result:
                    self.get_logger().info(f'Goal succeeded with final error: {result.final_error}')
                    reward = self.calculate_episode_reward()
                    next_state = self.get_current_state()
                    
                    # Modified section of send_new_goal method
                    if isinstance(self.agent, SARSAAgent):
                        next_pid_values = self.agent.get_pid_values(next_state)
                        self.agent.update(state, (kp, ki, kd), reward, next_state, next_pid_values)
                    elif isinstance(self.agent, DQNAgent):
                        # Convert PID values to state tensor for DQN
                        current_state = torch.FloatTensor([state])
                        next_state_tensor = torch.FloatTensor([next_state])
                        
                        # Store experience in replay buffer
                        self.agent.remember(current_state.numpy(), 
                                         (kp, ki, kd), 
                                         reward, 
                                         next_state_tensor.numpy(), 
                                         True)
                        self.agent.replay()
                        
                        # Update target network periodically
                        if self.agent.episode_count % 10 == 0:
                            self.agent.update_target_model()
                    else:  # QLearningAgent
                        self.agent.update(state, (kp, ki, kd), reward, next_state)
            except asyncio.TimeoutError:
                self.get_logger().error('Goal timed out!')
                if self.current_goal_handle:
                    await self.current_goal_handle.cancel_goal_async()
                    
        except Exception as e:
            self.get_logger().error(f'Error in send_new_goal: {str(e)}')
        finally:
            self.goal_active = False

    def calculate_episode_reward(self):
        if not self.feedback_history:
            return -100.0
            
        errors = [abs(fb['error']) for fb in self.feedback_history]
        final_error = errors[-1]
        settling_time = len(errors) * 0.1
        max_overshoot = max(errors) if errors else float('inf')
        
        # Reward factors
        error_factor = -0.5 * final_error
        time_factor = -0.3 * settling_time
        overshoot_factor = -0.2 * max_overshoot
        
        reward = error_factor + time_factor + overshoot_factor
        
        self.get_logger().info(
            f'Episode complete - Final error: {final_error:.2f}, '
            f'Settling time: {settling_time:.2f}, '
            f'Max overshoot: {max_overshoot:.2f}, '
            f'Reward: {reward:.2f}'
        )
        
        return reward

    def get_current_state(self):
        if self.latest_feedback is not None:
            return self.latest_feedback.current_error
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorPIDTuner()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()