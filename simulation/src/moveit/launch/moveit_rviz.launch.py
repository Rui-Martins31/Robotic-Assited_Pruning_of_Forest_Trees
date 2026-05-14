from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    tf_prefix  = LaunchConfiguration("tf_prefix").perform(context)
    mappings = {"robot_name": robot_name, "tf_prefix": tf_prefix, "virtual_joint_child_link": tf_prefix + "base_link"}
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="moveit")
        .robot_description(mappings=mappings)
        .robot_description_semantic(file_path="config/ur.srdf.xacro", mappings=mappings)
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ur"),
        DeclareLaunchArgument("tf_prefix",  default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
