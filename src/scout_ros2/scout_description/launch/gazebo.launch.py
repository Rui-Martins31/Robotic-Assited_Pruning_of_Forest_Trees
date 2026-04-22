import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_robot_state_publisher(_context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('scout_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'scout_v2.xacro')

    result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
    
    urdf = result.stdout
    urdf = urdf.replace('package://scout_description', f'file://{pkg_share}')
    urdf = urdf.replace('/meshes/scout_v2/', '/meshes/')

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
    )]


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world'), ' --render-engine ogre'],
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_scout',
        output='screen',
        arguments=[
            '-name', 'scout_v2',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world SDF file to load',
        ),

        gazebo,
        OpaqueFunction(function=_launch_robot_state_publisher),
        spawn_robot,
        clock_bridge,
    ])
