#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.executors import Executor
from rl_pid_uros.action import TunePID
import numpy as np
from rl_pid_uros_py.q_learner import QLearningAgent
import random

class MotorPIDTuner(Node):
    def __init__(self):
        super().__init__('rl_pid_tuner')
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action client
        self.action_client = ActionClient(
            self,
            TunePID,
            'tune_pid_action',
            callback_group=self.callback_group
        )
        
        # Track current goal status
        self.current_goal_handle = None
        self.goal_active = False
        
        # Store feedback data for learning
        self.latest_feedback = None
        self.feedback_history = []
        
        # Initialize Q-learning agent
        self.agent = QLearningAgent()
        self.get_logger().info('Q-learning agent initialized')
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available!')

        # Create timer for sending new goals
        self.timer = self.create_timer(5.0, self.send_new_goal_wrapper)
        self.get_logger().info('Timer started for sending goals every 5 seconds')

    def generate_random_target(self):
        """Generate a random target position between 0 and 360 degrees"""
        return random.randint(0, 360)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        self.latest_feedback = feedback
        self.feedback_history.append({
            'error': feedback.current_error,
            'position': feedback.current_position,
            'target': feedback.target_position,
            'timestamp': self.get_clock().now().to_msg()
        })
        
        self.get_logger().debug(
            f'Feedback received - Error: {feedback.current_error:.2f}, '
            f'Position: {feedback.current_position}, '
            f'Target: {feedback.target_position}'
        )

    async def send_new_goal_wrapper(self):
        """Non-async wrapper for the timer callback"""
        await self.send_new_goal()

    async def send_new_goal(self):
        """Send a new goal to the action server"""
        if self.goal_active:
            self.get_logger().debug('Previous goal still active, skipping...')
            return
            
        try:
            # Get current state and PID values from Q-learning agent
            state = self.get_current_state()
            kp, ki, kd = self.agent.get_pid_values(state)
            
            # Generate new random target position
            target_position = self.generate_random_target()
            
            # Create goal message
            goal_msg = TunePID.Goal()
            goal_msg.kp = float(kp)
            goal_msg.ki = float(ki)
            goal_msg.kd = float(kd)
            goal_msg.target_position = target_position
            
            self.get_logger().info(
                f'Sending new goal - kp: {kp:.4f}, ki: {ki:.4f}, kd: {kd:.4f}, '
                f'target: {target_position} degrees'
            )
            
            # Clear feedback history for new episode
            self.feedback_history = []
            
            # Send goal and wait for acceptance
            send_goal_future = await self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            # Check if goal was accepted
            if not send_goal_future.accepted:
                self.get_logger().warn('Goal was rejected!')
                return

            # Store the goal handle
            self.current_goal_handle = send_goal_future
            self.goal_active = True
                
            # Wait for result
            get_result_future = await send_goal_future.get_result_async()
            result = get_result_future.result
            
            # Process result and update Q-learning agent
            final_error = result.final_error
            reward = self.calculate_episode_reward()
            next_state = int(final_error)
            
            # Update Q-learning agent with episode results
            self.agent.update(state, (kp, ki, kd), reward, next_state)
            
            self.get_logger().info(
                f'Goal completed - Final error: {final_error:.2f}, '
                f'Reward: {reward:.2f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in send_new_goal: {str(e)}')
        finally:
            self.goal_active = False
        
        rclpy.spin_once(self)   
    def calculate_episode_reward(self):
        """Calculate reward based on feedback history"""
        if not self.feedback_history:
            return -1000.0  # Large penalty for no feedback
            
        errors = np.array([abs(fb['error']) for fb in self.feedback_history])
        
        # Metrics for reward calculation
        final_error = errors[-1]
        settling_time = len(errors)
        overshoot = np.max(errors) - final_error
        
        # Reward components
        error_reward = -final_error
        settling_reward = -settling_time
        overshoot_reward = -overshoot
        
        total_reward = (
            0.5 * error_reward + 
            0.3 * settling_reward + 
            0.2 * overshoot_reward
        )
        
        return float(total_reward)

    def get_current_state(self):
        """Get current state for Q-learning"""
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