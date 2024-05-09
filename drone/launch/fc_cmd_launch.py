#!/usr/bin/env python3

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='drone',
            executable='fc_cmd.py',
            name='FCCommander',
            output='screen', # to see the output of the node
        ),
        Node(
            package='drone',
            executable='Controller_node',
            output='screen',  # Direct output to screen (terminal)
        ),
    ])

