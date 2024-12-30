#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rl_pid_uros.srv import Tminusp
import numpy as np
import time  # To handle the 5-second interval
from rl_pid_uros_py.q_learner import QLearningAgent



class MotorPIDTuner(Node):
    def __init__(self):
        super().__init__('rl_pid_tuner')

        # Create a service client for /tune_pid
        self.client = self.create_client(Tminusp, '/tune_pid')

        # Log that we're waiting for the service
        self.get_logger().info('Waiting for /tune_pid service to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # Add heartbeat timer
        self.heartbeat_timer = self.create_timer(1.0, self.check_connection)
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
    
    
    def check_connection(self):

        if not self.client.service_is_ready():
            self.get_logger().warn('Service not available')
    
    
    def send_pid_values(self):
            try:
                
                state = self.get_current_state()
                self.get_logger().info(f'Getting PID values for state: {state}')

                # Retrieve PID values from the agent
                kp, ki, kd = self.agent.get_pid_values(state)
                self.get_logger().info(f'PID values from agent: kp={kp}, ki={ki}, kd={kd}')

                # Create the service request with the PID values
                request = Tminusp.Request()
                request.kp = float(kp)
                request.ki = float(ki)
                request.kd = float(kd)
                self.get_logger().info(f'Created service request with values: kp={kp}, ki={ki}, kd={kd}')
                future = self.client.call_async(request)

                if not rclpy.spin_until_future_complete(self, future, timeout_sec=5.0):
                        self.get_logger().warn('Service call timed out')
                        return
                # Add validity checks for response
                if not isinstance(future.result(), Tminusp.Response):
                    self.get_logger().error('Received invalid response type')
                    return
                    
                response = future.result()
                error = response.tvp
                
                # Add bounds checking for reward calculation
                if abs(error) > 1000:  # Arbitrary threshold
                    self.get_logger().warn(f'Unusually large error value: {error}')
                    return
                    
                # Use a more sophisticated reward function
                reward = -abs(error) / (1 + abs(error))  # Bounded negative reward
                
            except Exception as e:
                self.get_logger().error(f'Error in send_pid_values: {str(e)}')

    def get_current_state(self):
        # Fetch current state based on motor position or error
        # Placeholder logic for demonstration purposes
        error = np.random.uniform(-10, 10)  # Replace with actual sensor data if available
        self.get_logger().info(f'Current state (error): {error}')
        return int(error)


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
