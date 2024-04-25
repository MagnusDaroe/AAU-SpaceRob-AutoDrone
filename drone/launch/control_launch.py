import launch
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone',
            executable='test_new',
            output='screen',  # Direct output to screen (terminal)
            parameters=[{
                'ros__parameters': {
                    'rosout__console__format': '[${severity}] [${time}] [${node}] [${logger}] [${message}]'
                }
            }],
        ),
    ])
