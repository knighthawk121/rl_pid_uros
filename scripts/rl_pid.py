#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rl_pid_uros.srv import Tminusp
import numpy as np
import time  # To handle the 5-second interval
from rl_pid_uros_py.rl_agent import QLearningAgent


class MotorPIDTuner(Node):
    def __init__(self):
        super().__init__('rl_pid_tuner')
        
        # Create a service client for /tune_pid
        self.client = self.create_client(Tminusp, '/tune_pid')
        
        # Log that we're waiting for the service
        self.get_logger().info('Waiting for /tune_pid service to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Initialize Q-learning agent
        self.agent = QLearningAgent()
        self.get_logger().info('Q-learning agent initialized.')

        # Set initial PID values
        self.kp = 1.0
        self.ki = 0.001
        self.kd = 0.0
        self.get_logger().info(f'Initial PID values set: kp={self.kp}, ki={self.ki}, kd={self.kd}')

        # Start the timer to send PID values every 5 seconds
        self.timer = self.create_timer(5.0, self.send_pid_values)  # 5 seconds interval
        self.get_logger().info('Timer started to send PID values every 5 seconds.')

    def send_pid_values(self):
        # Get PID values from the Q-learning agent based on the current state
        state = self.get_current_state()  # Placeholder for actual state calculation
        self.get_logger().info(f'Getting PID values for state: {state}')
        
        # Retrieve PID values from the agent
        kp, ki, kd = self.agent.get_pid_values(state)
        self.get_logger().info(f'PID values from agent: kp={kp}, ki={ki}, kd={kd}')

        # Create the service request with the PID values
        request = Tminusp.Request()
        request.kp = kp
        request.ki = ki
        request.kd = kd
        self.get_logger().info(f'Created service request with values: kp={kp}, ki={ki}, kd={kd}')

        # Send the request asynchronously and wait for the response
        future = self.client.call_async(request)
        self.get_logger().info('Sending service request asynchronously...')
        rclpy.spin_until_future_complete(self, future)

        # Process the response
        if future.result() is not None:
            response = future.result()
            tvp = response.tvp  # Target vs position value from ESP32 response
            self.get_logger().info(f'Received target vs position value: {tvp}')
            
            # Calculate the reward based on the state (e.g., negative reward for large error)
            reward = -abs(tvp)
            self.get_logger().info(f'Calculated reward: {reward}')

            # Update Q-table with feedback
            kp_idx = int(np.clip(kp * 10, 0, 9))
            ki_idx = int(np.clip(ki * 100, 0, 9))
            kd_idx = int(np.clip(kd * 1000, 0, 9))
            self.get_logger().info(f'Updating Q-table with indices: kp_idx={kp_idx}, ki_idx={ki_idx}, kd_idx={kd_idx}')
            self.agent.update_q_table(kp_idx, ki_idx, kd_idx, reward)

            # Log the update
            self.get_logger().info(f'Q-table updated with reward: {reward}')
        else:
            self.get_logger().error('Service call failed. No response received.')

    def get_current_state(self):
        # Implement a way to get the current state based on the motor position or error
        # For example, calculate the motor error here
        self.get_logger().info('Fetching current state (placeholder value).')
        return 0  # Placeholder: Replace with actual state calculation (e.g., motor position)

def main(args=None):
    try:
        rclpy.init(args=args)
        motor_pid_tuner = MotorPIDTuner()
        rclpy.spin(motor_pid_tuner)
        motor_pid_tuner.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"Error: {e}")
        raise

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Terminating the client.")
