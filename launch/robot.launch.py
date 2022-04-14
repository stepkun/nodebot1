import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='nodebot1' #<--- CHANGE ME

    # Include the micro-ROS-Agent launch file, provided by our own package
    agent = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','micro-ros-agent.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
             )

    # Launch them all!
    return LaunchDescription([
        agent,
    ])
