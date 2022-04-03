from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import platform

def generate_launch_description():
    device = '/dev/ttyUSB0'             # device on PC-Linux
    if platform.machine() == 'aarch64':
        if platform.version().find('Ubuntu') >= 0:
            device = '/dev/ttyS0'       # device on pi with ubuntu
        else:
            device = 'dev/ttyAMA0'      # device on pi with raspbian

    return LaunchDescription([
        # start micro-ros agent as separate process
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', device],
            output='screen',
        ),
    ])