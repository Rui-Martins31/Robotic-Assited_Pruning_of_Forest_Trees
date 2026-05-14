import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_robot_state_publisher(_context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('scout_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'scout_v2.xacro')

    result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)

    urdf = result.stdout.replace('/meshes/scout_v2/', '/meshes/')

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf,
            'use_sim_time': False,
        }],
    )]


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('scout_description'), 'rviz', 'scout_v2.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),

        OpaqueFunction(function=_launch_robot_state_publisher),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])