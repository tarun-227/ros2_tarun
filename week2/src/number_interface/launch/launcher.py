import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='number_interface',
            executable='number_publisher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='number_interface',
            executable='number_subscriber',
            output='screen'
        ),
    ])

