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
   alternatively you can use this prebuilt [docker image](https://hub.docker.com/r/knighthawk121/rl_pid_uros/tags) with ros2 and pytorch inside conda venv.
   - [Conda Environment](https://docs.anaconda.com/miniconda/install/)
   - [ROS2 humble from robostack to make use of pytorch](https://robostack.github.io/GettingStarted.html)
   - [pytorch](https://pytorch.org/get-started/locally/)
   
   follow the instructions given in the robostack website carefully, all the dependencies of ros2 must be installed inside the virtual env as given in the instructions.

## Installation

   ### ROS2_Client
   follow this instructions to install the package locally on conda venv or simply use [the docker image](https://hub.docker.com/r/knighthawk121/rl_pid_uros/tags) 

1. **Clone the repository**:
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/knighthawk121/rl_pid_uros.git
   ```
2. **Install dependencies:**
Ensure you have ROS2 installed inside conda - mamba virtual env to use pytorch or You can follow the official installation guides for [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) if you dont want use the deep learning models for this project. (need to modify the main script to not use pytorch).

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build 
   ```
   ### Micro-ros server
   follow these instructions to install this package inside micro_ros_arduino library.
4. **Clone the Microros library for arduino**:
Select the suitable branch based on your ROS2 distro.
   ```bash
   cd ~
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_arduino.git
   ```
5. **Clone the repository of the interfaces**:
   ```bash
   #the package should be placed inside the folder below
   cd ~/micro_ros_arduino/extras/library_generation/extra_packages
   git clone https://github.com/knighthawk121/rl_pid_uros_interfaces.git
   ```
6. **Building the library for Arduino IDE**:
   building this custom library requires [Docker](https://docs.docker.com/engine/install/ubuntu/), make sure you have installed it properly and the daemon is running.

   ```bash
   cd ~/micro_ros_arduino

   #pulling the docker image
   docker pull microros/micro_ros_static_library_builder:jazzy #use your ROS2 distro
   
   #building the custom microros library for arduino IDE
   docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:jazzy -p esp32
   ```
7. **adding the pre-compiled library to Arduino libraries folder**:
   ```bash
   #make sure you installed Arduino IDE and required packages for esp32 board

   mkdir -p ~/Arduino/libraries/micro_ros_arduino

   #copy the contents of the compiled library to Arduino library folder
   rsync -av --exclude=".*" ~/micro_ros_arduino/.  ~/Arduino/libraries/micro_ros_arduino/.
   
   #note: don't update the library using Arduino libraries manager, since it will replace the custom packages with default.
   ```
8. **Setting-up Micro-ROS-agent for Microros communication**:
You can follow [this instructions for setting up the micro-ros-agent](https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca) locally or 

you can always use the ready to deploy Docker image for micro-ros-agent
```bash
   # udp4 micro-ROS Agent, make sure you have used the right ROS_DISTRO
   docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 -p 8888 
```

## Usage

### ROS2 Client node

1. **Run the ROS2 node:**
   
   ```bash
   #assuming the package is in ros2_ws, source the setup

   source ~/ros2_ws/install/setup.bash
   #for actions client
   ros2 run rl_pid_uros pid_tuner
   
   #for service client
   ros2 run rl_pid_uros pid_tuner_service

   #for launching action client and Micro-ros-agent
   ros2 launch rl_pid_uros rl_action

   #for launching service client and Micro-ros-agent
   ros2 launch rl_pid_uros rl_service

   #select the agent from the options and the server node for running the server.
   ```
   
### Server Node (ROS2 or Micro-ROS)
   for the server node , we have two options - either you can [run the server in a Raspberry-pi using ROS2](https://github.com/knighthawk121/rl_pid_raspi_server.git) or Micro-ROS with ESP32. (Micro-ros uses MicroXRCEdds agent as middle ware for communications, which has an issue of deserialization while running a action server, so it is recommended to use service server instead ) 
   
   **For running the server inside a raspberry-pi follow this [repo](https://github.com/knighthawk121/rl_pid_raspi_server.git) for more details.**


 **Configure the ESP32:**
use the [rl_pid_actions_wifi_udp.ino](/uros/arduino_ide_esp32/rl_pid_actions_wifi_udp.ino) for actions server and [rl_pid_server.ino](/uros/arduino_ide_esp32/rl_pid_server.ino) for service server files inside the uros/arduino_ide_esp32 folder for server node. compile and upload the code. make sure you have replaced the following parameters with your own.
   
   ```cpp
      const char *ssid = "Your_SSID";
      const char *password = "Your_Password";
      const char *agent_ip = "Your_Computer_IP";
      const int agent_port = 8888;
   ```
   ```diff
   - the Micro-ros action server has an issue of deserialization of goal messages while transporting, so its unusable until its fixed
   + you can use the raspberry pi server instead
   ```



## Configuration

   1.**PID Parameters:** Initial PID values are set in [main scripts files](/scripts). Adjust kp, ki, and kd as needed.
    
   2.**Q-learning Parameters:** Modify alpha, gamma, and epsilon to tune the learning behavior.


## License

This project is licensed under the Apache-2.0 License. See the LICENSE file for details. 
