#!/usr/bin/env python3
"""
Launch file for the DD24 altitude PID controller node.

Usage
-----
ros2 launch pid_controller pid_controller_node.launch.py range_topic:=<RANGE_TOPIC>

Parameters
----------
range_topic       : str   Topic publishing sensor_msgs/Range from the bottom rangefinder.
                           MUST be set to the actual topic on your robot.
                           Placeholder: 'RANGE_TOPIC'
setpoint_z        : float Desired hover altitude in metres (default: 0.25 m).
control_frequency : float Setpoint publishing rate in Hz (default: 20.0).
                           Must be > 2 Hz to maintain PX4 OFFBOARD mode.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default to the bottom rangefinder topic of the robot running this
    # container. VEHICLE_NAME is exported in the robot environment; fall back
    # to 'drone01' for local/dev runs. Override with range_topic:=... .
    default_range_topic = '/{}/bottom_tof_driver_node/range'.format(
        os.environ.get('VEHICLE_NAME', 'drone01')
    )

    range_topic_arg = DeclareLaunchArgument(
        'range_topic',
        default_value=default_range_topic,
        description='Topic publishing sensor_msgs/Range from the bottom rangefinder.',
    )

    setpoint_z_arg = DeclareLaunchArgument(
        'setpoint_z',
        default_value='0.25',
        description='Desired hover altitude in metres.',
    )

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='20.0',
        description='Setpoint publishing rate in Hz (must be > 2 Hz for OFFBOARD).',
    )

    thrust_cap_arg = DeclareLaunchArgument(
        'thrust_cap',
        default_value='0.55',
        description='Maximum thrust the PID can command while flying, from 0.0 to 1.0.',
    )

    pid_controller_node = Node(
        package='pid_controller',
        executable='pid_controller_node',
        name='altitude_pid_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'range_topic': LaunchConfiguration('range_topic'),
            'setpoint_z': LaunchConfiguration('setpoint_z'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'thrust_cap': LaunchConfiguration('thrust_cap'),
        }],
    )

    return LaunchDescription([
        range_topic_arg,
        setpoint_z_arg,
        control_frequency_arg,
        thrust_cap_arg,
        pid_controller_node,
    ])
