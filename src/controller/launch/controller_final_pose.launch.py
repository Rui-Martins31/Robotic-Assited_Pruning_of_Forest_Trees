import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="moveit")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_py_config = os.path.join(
        get_package_share_directory("controller"), "config", "moveit_py_config.yaml"
    )

    moveit_py_node = Node(
        package="controller",
        executable="node_controller_final_pose",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_config,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([moveit_py_node])