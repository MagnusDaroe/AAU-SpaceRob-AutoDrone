import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='drone',
            executable='fc_cmd.py',
            name='FCCommander',
            parameters=[
                {'test_type': 'pitch'},
                {'test_pitch_value': '200'},  # Set the test pitch value
                {'test_roll_value': '200'},   # Set the test roll value
                {'test_yaw_value': '200'},    # Set the test yaw value
                {'test_thrust_value': '300'}, # Set the test thrust value
            ],
            output='screen', # to see the output of the node
        ),
        Node(
            package='drone',
            executable='Controller_node',
            output='screen',  # Direct output to screen (terminal)
            parameters=[{
                'ros__parameters': {
                    'rosout__console__format': '[${severity}] [${time}] [${node}] [${logger}] [${message}]'
                }
            }],
        ),
    ])
