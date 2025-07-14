from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('bot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    world_path = os.path.join(pkg_path, 'worlds', 'my_world.world')

    # Expand the xacro file to robot_description
    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'four_wheel_bot'],
            output='screen'
        )
    ])
