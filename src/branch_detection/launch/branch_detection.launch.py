from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='branch_detection',
            executable='service_compute_world_coordinates',
            name='compute_world_position',
            output='screen',
        ),
        Node(
            package='branch_detection',
            executable='node_detect_branch',
            name='detect_branch',
            output='screen',
        ),
    ])
