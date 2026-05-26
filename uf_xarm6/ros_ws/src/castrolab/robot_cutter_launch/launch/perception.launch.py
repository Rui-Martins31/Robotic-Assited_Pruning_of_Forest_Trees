from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Camera Debugger
    camera_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense_view'),
            'launch',
            'camera_view.launch.py',
        ]))
    )

    # Branch detection
    branch_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('branch_detection'),
            'launch',
            'branch_detection_by_color.launch.py',
        ]))
    )

    return LaunchDescription([
        camera_view,
        branch_detection,
    ])
