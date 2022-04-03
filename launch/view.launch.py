import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package path
    pkg_path = os.path.join(get_package_share_directory('nodebot1')) #<--- CHANGE ME
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_path + '/launch/rsp.launch.py'))

    joint_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_path + '/launch/jspg.launch.py'))

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                pkg_path + '/launch/rviz.launch.py'))

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])