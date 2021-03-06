import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='nodebot1' #<--- CHANGE ME

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
            )

    # Include the Gamepad launch file, provided by our own package
    gamepad = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','gamepad_sim.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
             )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'nodebot1'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        rsp,
        gamepad,
        gazebo,
        spawn_entity,
    ])
