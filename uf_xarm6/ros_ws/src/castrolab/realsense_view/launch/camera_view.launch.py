import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('realsense_view'),
        'config',
        'camera_view.rviz',
    )

    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable the depth pointcloud topic',
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
            'pointcloud.ordered_pc': False,
            'enable_sync': True,
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x15',
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        enable_pointcloud_arg,
        realsense_node,
        rviz_node,
    ])
