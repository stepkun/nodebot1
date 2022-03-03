import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create a joint_state_publisher_gui node
    params = {'use_sim_time': use_sim_time}
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        namespace='',
        executable='joint_state_publisher_gui',
        name='joint_state_pubisher_gui',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_joint_state_publisher_gui
    ])