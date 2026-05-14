from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_file = PathJoinSubstitution(
        [FindPackageShare("robot_cutter"), "urdf", "robot_cutter.urdf.xacro"]
    )

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", description_file]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"zeros": {
            "ur_shoulder_pan_joint":  -0.75,
            "ur_shoulder_lift_joint": -0.75,
            "ur_elbow_joint":         -0.90,
            "ur_wrist_1_joint":       -1.57,
            "ur_wrist_2_joint":       -1.57,
            "ur_wrist_3_joint":        0.0,
        }}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("manipulator"), "rviz", "view_robot.rviz"]
        )],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
