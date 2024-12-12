import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rl_pid_uros.msg import PIDValues
from rl_pid_uros.srv import Tminusp
import numpy as np
import socket

# Define the UDP connection to the ESP32
UDP_IP = "192.168.31.249"  # Replace with your ESP32's IP address
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Q-learning parameters
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 0.1  # Exploration rate

# Initial PID values
kp = 3.1
ki = 0.049
kd = 0.03

# Q-table initialization (for simplicity, treat PID values as discrete ranges)
q_table = np.zeros((10, 10, 10))  # 3D Q-table for KP, KI, KD

class RLPIDAgent:
    def __init__(self):
        self.q_table = q_table

    def get_action(self, kp_idx, ki_idx, kd_idx):
        if np.random.uniform(0, 1) < epsilon:  # Explore
            return np.random.choice([0, 1, 2])  # Change in PID values: Decrease, Stay, Increase
        else:  # Exploit
            return np.argmax(self.q_table[kp_idx, ki_idx, kd_idx])

    def update_q_table(self, kp_idx, ki_idx, kd_idx, reward):
        best_future_q = np.max(self.q_table[kp_idx, ki_idx, kd_idx])
        current_q = self.q_table[kp_idx, ki_idx, kd_idx]
        self.q_table[kp_idx, ki_idx, kd_idx] += alpha * (reward + gamma * best_future_q - current_q)

    def get_pid_values(self, state):
        # Discretize PID values for Q-table indexing
        kp_idx = int(np.clip(kp * 10, 0, 9))
        ki_idx = int(np.clip(ki * 100, 0, 9))
        kd_idx = int(np.clip(kd * 1000, 0, 9))

        # Get Q-table action
        action = self.get_action(kp_idx, ki_idx, kd_idx)

        # Adjust PID values based on Q-table action
        if action == 0:
            kp_adjusted = kp - 0.1
            ki_adjusted = ki - 0.001
            kd_adjusted = kd - 0.005
        elif action == 1:
            kp_adjusted = kp
            ki_adjusted = ki
            kd_adjusted = kd
        else:
            kp_adjusted = kp + 0.1
            ki_adjusted = ki + 0.001
            kd_adjusted = kd + 0.005

        return kp_adjusted, ki_adjusted, kd_adjusted

class MotorFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('rl_pid')
        self.subscription = self.create_subscription(
            Float32,
            'uros_msg_pid',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.agent = RLPIDAgent()

    def listener_callback(self, msg):
        # Use the feedback to update the RL agent
        state = msg.data
        kp, ki, kd = self.agent.get_pid_values(state)

        # Send the new PID values to the ESP32
        message = f"{kp},{ki},{kd}".encode('utf-8')
        sock.sendto(message, (UDP_IP, UDP_PORT))
        self.get_logger().info(f'Sent PID values: kp={kp}, ki={ki}, kd={kd}')

        # Calculate the reward based on the state
        reward = -abs(state)  # Example: negative reward for large error

        # Update Q-table with feedback
        kp_idx = int(np.clip(kp * 10, 0, 9))
        ki_idx = int(np.clip(ki * 100, 0, 9))
        kd_idx = int(np.clip(kd * 1000, 0, 9))
        self.agent.update_q_table(kp_idx, ki_idx, kd_idx, reward)

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
