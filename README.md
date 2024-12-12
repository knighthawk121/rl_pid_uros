RL PID Controller for Encoder Motor
This repository contains a reinforcement learning-based PID controller for an encoder motor, utilizing ROS2 and Micro-ROS as middleware. The project aims to optimize the PID parameters using Q-learning to improve motor control performance.

Table of Contents
Introduction
Features
Installation
Usage
Configuration
License
Contact
Introduction
The project implements a PID controller enhanced with reinforcement learning techniques to dynamically adjust the PID parameters (Kp, Ki, Kd) for an encoder motor. The controller is designed to work with ROS2 and Micro-ROS, facilitating communication with embedded devices like the ESP32.

Features
Reinforcement Learning: Utilizes Q-learning to adapt PID parameters based on motor feedback.
ROS2 Integration: Leverages ROS2 for communication and control.
Micro-ROS Support: Compatible with Micro-ROS for embedded systems.
UDP Communication: Sends PID values to an ESP32 via UDP.
Installation
Clone the repository:

git clone https://github.com/yourusername/rl_pid_uros.git
cd rl_pid_uros
Install dependencies: Ensure you have ROS2 and Micro-ROS installed. You can follow the official installation guides for ROS2 and Micro-ROS.

Build the package:

colcon build
Usage
Run the ROS2 node:

source install/setup.bash
ros2 run rl_py_pid_uros rl_pid
Configure the ESP32: Ensure your ESP32 is set up to receive UDP packets on the specified IP and port.

Configuration
PID Parameters: Initial PID values are set in rl_pid.py. Adjust kp, ki, and kd as needed.
Q-learning Parameters: Modify alpha, gamma, and epsilon to tune the learning behavior.
UDP Settings: Update UDP_IP and UDP_PORT with your ESP32's IP address and port.
License
This project is licensed under the Apache-2.0 License. See the LICENSE file for details.

Contact
For questions or collaboration, please contact Arvindh R at arvindh1793@gmail.com.
