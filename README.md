# RL PID Controller for Encoder Motor

This repository contains a **Reinforcement learning-based PID controller** for an encoder motor, utilizing **ROS2** and **Micro-ROS** as middleware. The project aims to optimize the PID parameters using Reinforcement learning to improve motor control performance.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [License](#license)

## Introduction

The project implements a PID controller enhanced with reinforcement learning techniques to dynamically adjust the PID parameters (Kp, Ki, Kd) for an encoder motor. The controller is designed to work with ROS2 and Micro-ROS, facilitating communication with embedded devices like the ESP32.

## Features

- **Reinforcement Learning**: Utilizes various RL algorithms to adapt PID parameters based on motor feedback.
- **ROS2 Integration**: Leverages ROS2 Actions for communication and control.
- **Micro-ROS Support**: Compatible with Micro-ROS for embedded systems (Tested on ESP32)

## Dependencies
   the project requires pytorch which can be installed using pip or conda package manager to use locally.
   - [Conda Environment](https://docs.anaconda.com/miniconda/install/)
   - [ROS2 humble from robostack to make use of pytorch](https://robostack.github.io/GettingStarted.html)
   - [pytorch](https://pytorch.org/get-started/locally/)
   
   follow the instructions given in the robostack website carefully, all the dependencies of ros2 must be installed inside the virtual env as given in the instructions.

## Installation

1. **Clone the repository**:
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/knighthawk121/rl_pid_uros.git

2. **Install dependencies:**
Ensure you have ROS2 and Micro-ROS installed. You can follow the official installation guides for [ROS2](https://docs.ros.org/en/jazzy/Installation.html) and [Micro-ROS](https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca).

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build 

## Usage

1. **Run the ROS2 node:**
   
   ```bash
   source install/setup.bash
   ros2 run rl_pid_uros pid_tuner

  2.**Configure the ESP32:**

  Ensure your ESP32 is set up to receive UDP packets on the specified IP and port.

## Configuration

   1.**PID Parameters:** Initial PID values are set in rl_pid.py. Adjust kp, ki, and kd as needed.
    
   2.**Q-learning Parameters:** Modify alpha, gamma, and epsilon to tune the learning behavior.

   3.**UDP Settings:** Update UDP_IP and UDP_PORT with your ESP32's IP address and port.


## License

This project is licensed under the Apache-2.0 License. See the LICENSE file for details. 
