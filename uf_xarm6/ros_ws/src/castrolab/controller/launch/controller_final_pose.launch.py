from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.207', description='IP address of the xArm6'),
        Node(
            package='controller',
            executable='node_controller_final_pose',
            output='screen',
        ),
    ])
