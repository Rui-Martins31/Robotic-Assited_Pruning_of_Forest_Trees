from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def launch_setup(context, *args, **kwargs):
    robot_name             = LaunchConfiguration("robot_name").perform(context)
    tf_prefix              = LaunchConfiguration("tf_prefix").perform(context)
    joint_limits_file      = LaunchConfiguration("joint_limits_file").perform(context)
    moveit_controllers_file = LaunchConfiguration("moveit_controllers_file").perform(context)
    mappings = {"robot_name": robot_name, "tf_prefix": tf_prefix, "virtual_joint_child_link": tf_prefix + "base_link"}
    builder = (
        MoveItConfigsBuilder("ur", package_name="moveit")
        .robot_description(mappings=mappings)
        .robot_description_semantic(file_path="config/ur.srdf.xacro", mappings=mappings)
    )
    if joint_limits_file:
        builder.joint_limits(file_path=joint_limits_file)
    if moveit_controllers_file:
        builder.trajectory_execution(file_path=moveit_controllers_file)
    moveit_config = builder.to_moveit_configs()
    return generate_move_group_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name",              default_value="ur"),
        DeclareLaunchArgument("tf_prefix",               default_value=""),
        DeclareLaunchArgument("joint_limits_file",       default_value=""),
        DeclareLaunchArgument("moveit_controllers_file", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
