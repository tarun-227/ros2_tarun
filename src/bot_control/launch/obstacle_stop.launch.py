from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_control',
            executable='obstacle_stop',
            name='obstacle_stopper',
            output='screen'
        )
    ])

