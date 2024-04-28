import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='drone',
            executable='clock.py',
            name='ServerClock',
        ),
        launch_ros.actions.Node(
            package='drone',
            executable='GCS.py',
            name='GroundControlStation'
        ),
        launch_ros.actions.Node(
            package='drone',
            executable='manual_cmd.py',
            name='ManualCommand',
        )
    ])

