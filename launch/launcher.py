from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run your custom package node
        Node(
            package='rl_pid_uros',
            executable='pid_tuner',
            output='screen'  # Optionally, set output to 'screen' to see node output in the terminal
        ),

        # Run micro_ros_agent in a separate terminal
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888'],
            output='screen'
        )
    ])