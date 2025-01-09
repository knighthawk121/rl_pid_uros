#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # ... (LogInfo statements)
        LogInfo(msg="Launching micro-ROS agent in a separate terminal..."),
        # Run micro_ros_agent
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '-p', '8888'],
            output='screen'
        ),

        # Add a delay before launching pid_tuner
        LogInfo(msg="Launching PID tuner node..."),
        TimerAction(
            period=5.0,  # Wait for 5 seconds
            actions=[
                Node(
                    package='rl_pid_uros',
                    executable='pid_tuner',
                    output='screen'
                )
            ]
        )
    ])