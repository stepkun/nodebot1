# A launchfile to start te central control node for nodebot1

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import platform

def generate_launch_description():
    return LaunchDescription([
        # start control node
        Node(
            package='nodebot1',
            executable='control_py',
            parameters=[]
        ),
    ])