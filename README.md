# RL PID Controller for Encoder Motor

This repository contains a **reinforcement learning-based PID controller** for an encoder motor, utilizing **ROS2** and **Micro-ROS** as middleware. The project aims to optimize the PID parameters using Q-learning to improve motor control performance.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [License](#license)

## Introduction

The project implements a PID controller enhanced with reinforcement learning techniques to dynamically adjust the PID parameters (Kp, Ki, Kd) for an encoder motor. The controller is designed to work with ROS2 and Micro-ROS, facilitating communication with embedded devices like the ESP32.

## Features

- **Reinforcement Learning**: Utilizes Q-learning to adapt PID parameters based on motor feedback.
- **ROS2 Integration**: Leverages ROS2 for communication and control.
- **Micro-ROS Support**: Compatible with Micro-ROS for embedded systems.
- **UDP Communication**: Sends PID values to an ESP32 via UDP.

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/knighthawk121/rl_pid_uros.git
   cd rl_pid_uros

2. **Install dependencies:**
Ensure you have ROS2 and Micro-ROS installed. You can follow the official installation guides for [ROS2](https://docs.ros.org/en/jazzy/Installation.html) and [Micro-ROS](https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca).

3. **Build the package:**
   ```bash
   colcon build

## Usage

1. **Run the ROS2 node:**
   
   ```bash
   source install/setup.bash
   ros2 run rl_py_pid_uros rl_pid

  2.**Configure the ESP32:**

  Ensure your ESP32 is set up to receive UDP packets on the specified IP and port.

## Configuration

   1.**PID Parameters:** Initial PID values are set in rl_pid.py. Adjust kp, ki, and kd as needed.
    
   2.**Q-learning Parameters:** Modify alpha, gamma, and epsilon to tune the learning behavior.

   3.**UDP Settings:** Update UDP_IP and UDP_PORT with your ESP32's IP address and port.
## License

This project is licensed under the Apache-2.0 License. See the LICENSE file for details. 
