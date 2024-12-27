#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rl_pid_uros.srv import Tminusp
import numpy as np
import socket
from rl_pid_uros_py.rl_agent import QLearningAgent  # Import the agent

# Define the UDP connection to the ESP32
UDP_IP = "192.168.31.249"  # Replace with your ESP32's IP address
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initial PID values
kp = 1.0
ki = 0.001
kd = 0.0

class MotorFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('rl_pid_sub')
        self.subscription = self.create_subscription(
            Float32,
            'esp32_motor_controller',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.agent = QLearningAgent()  # Use QLearningAgent or SARSAAgent

        # Create a service client
        self.client = self.create_client(Tminusp, '/tune_pid')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def listener_callback(self, msg):
        # Use the feedback to update the RL agent
        state = msg.data
        kp, ki, kd = self.agent.get_pid_values(state)

        # Create a request
        request = Tminusp.Request()
        request.kp = kp
        request.ki = ki
        request.kd = kd

        # Send the request and wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None:
            tvp = response.tvp
            self.get_logger().info(f'Received target vs position value: {tvp}')

            # Calculate the reward based on the state
            reward = -abs(tvp)  # Example: negative reward for large error

            # Update Q-table with feedback
            kp_idx = int(np.clip(kp * 10, 0, 9))
            ki_idx = int(np.clip(ki * 100, 0, 9))
            kd_idx = int(np.clip(kd * 1000, 0, 9))
            self.agent.update_q_table(kp_idx, ki_idx, kd_idx, reward)
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    motor_feedback_subscriber = MotorFeedbackSubscriber()
    rclpy.spin(motor_feedback_subscriber)
    motor_feedback_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Terminating the client.")
    finally:
        sock.close()
