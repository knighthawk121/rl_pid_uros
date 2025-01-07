#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
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
            'tune_pid',
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

        # Set initial PID values
        self.kp = 1.0
        self.ki = 0.001
        self.kd = 0.0
        
        # Current target position
        self.current_target = 0
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available!')

        # Create timer for sending new goals
        self.timer = self.create_timer(5.0, self.send_new_goal)
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
        
        # Log feedback data
        self.get_logger().debug(
            f'Feedback received - Error: {feedback.current_error:.2f}, '
            f'Position: {feedback.current_position}, '
            f'Target: {feedback.target_position}'
        )

    def calculate_episode_reward(self):
        """Calculate reward based on feedback history"""
        if not self.feedback_history:
            return 0.0
            
        # Calculate various performance metrics
        errors = [fb['error'] for fb in self.feedback_history]
        avg_error = np.mean(np.abs(errors))
        max_error = np.max(np.abs(errors))
        settling_time = len(self.feedback_history) * 0.1  # Feedback every 100ms
        
        # Combine metrics into a reward
        reward = -(
            0.5 * avg_error +  # Penalize average error
            0.3 * max_error +  # Penalize maximum error
            0.2 * settling_time  # Penalize longer settling times
        )
        
        return reward

    async def send_new_goal(self):
        """Send a new goal to the action server"""
        # Don't send new goal if one is already active
        if self.goal_active:
            self.get_logger().debug('Previous goal still active, skipping...')
            return
            
        try:
            # Get current state and PID values from Q-learning agent
            state = self.get_current_state()
            kp, ki, kd = self.agent.get_pid_values(state)
            
            # Generate new random target position
            target_position = self.generate_random_target()
            self.current_target = target_position
            
            # Create and send goal
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
            
            # Send goal and get future
            send_goal_future = await self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            # Handle goal response
            self.current_goal_handle = await send_goal_future
            if not self.current_goal_handle.accepted:
                self.get_logger().warn('Goal was rejected!')
                return
                
            self.goal_active = True
            
            # Wait for result
            result_future = await self.current_goal_handle.get_result_async()
            result = result_future.result
            
            # Process result and update Q-learning agent
            final_error = result.final_error
            reward = self.calculate_episode_reward()
            next_state = int(final_error)  # Simplistic state representation
            
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

    def get_current_state(self):
        """Get current state for Q-learning"""
        if self.latest_feedback is not None:
            # Use the latest error as state
            return int(self.latest_feedback.current_error)
        return 0  # Default state when no feedback available

async def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorPIDTuner()
        
        # Use multiple threads for concurrent execution
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            await executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Terminating the client.")