import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package path
    pkg_path = os.path.join(get_package_share_directory('nodebot1')) #<--- CHANGE ME
    
    # Create a rviz2 node
    params = {'use_sim_time': use_sim_time}
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[params],
        arguments=['-d' + os.path.join(pkg_path, 'config', 'view_bot.rviz')]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_rviz
    ])