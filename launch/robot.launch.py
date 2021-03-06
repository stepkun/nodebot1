import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='nodebot1' #<--- CHANGE ME

    # Include central control node
    control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','control.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
             )
    
    # Include cam control node
    cam_control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','cam_control.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
             )
    
    # Include ld06 lidar node
    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','ld06_control.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
             )

    # Include the micro-ROS-Agent launch file, provided by our own package
    agent = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','micro-ros-agent.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
             )

    # Launch them all!
    return LaunchDescription([
        control,
        cam_control,
        lidar,
        agent,
    ])
