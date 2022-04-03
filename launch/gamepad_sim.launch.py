# A launchfile to start gamepad as input device for chassis
# tested with Logitech F710 with Galactic on Ubuntu 20.04.3
#
# logitech F710 gamepad needs to be in mode "X"

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import platform

def generate_launch_description():
    return LaunchDescription([
        # start joy node
        # this will publish the topic /joy
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',        # input device
                'joy_config': 'xbox',
                'deadzone': 0.01,               # depends on quality of sticks
                'autorepeat_rate': 2.0,         # in Hz
                'coalesce_interval': 0.5,       # in seconds (default 0.001)
                'default_trig_val': False,      # read buttons on startup
             }]
        ),
        # convert /joy to /cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[{
                'require_enable_button': True,  # we need to press security button
                'enable_button': 5,             # right bumper-button is 5
                'enable_turbo_button': 4,      # left bummper-button is 4
                'axis_linear.x': 4,             # right sticks vertical axis
                'scale_linear.x': 0.5,
                'scale_linear_turbo.x': 1.25,
                'axis_angular.yaw': 0,          # left sticks horizontal axis
                'scale_angular.yaw': 1.0,
                'scale_angular_turbo.yaw': 2.5,
            }],
        )
    ])