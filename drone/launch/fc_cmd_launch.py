from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone',
            node_executable='fc_cmd.py',
            node_namespace='FCCommander',
            output='screen', # to see the output of the node
        ),
        Node(
            package='drone',
            node_executable='Controller_node',
            node_namespace='controlnode',
            output='screen',  # Direct output to screen (terminal)
        ),
    ])

