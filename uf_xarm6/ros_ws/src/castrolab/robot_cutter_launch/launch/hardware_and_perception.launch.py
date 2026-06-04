from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.207',
        description='IP address of the xArm6 robot',
    )

    # MoveIt Visualizer
    # Can also control the arm
    # through RViz
    moveit_visualizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'),
            'launch',
            'xarm6_moveit_realmove.launch.py',
        ])),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'report_type': 'rich',
        }.items(),
    )

    # MoveIt Planner
    moveit_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_planner'),
            'launch',
            '_robot_planner.launch.py',
            # 'xarm6_planner_realmove.launch.py',
        ])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'add_gripper': 'true'
        }.items(),
    )

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

    # Initial pose
    goto_initial_pose = Node(
        package='controller',
        executable='node_goto_initial_pose',
        output='screen',
    )

    # Service configure robot
    srv_robot_configuration = Node(
        package    = 'system_monitoring',
        executable = 'service_robot_configuration',
        output     = 'screen',
    )

    return LaunchDescription([
        robot_ip_arg,
        moveit_visualizer,
        moveit_planner,
        camera_view,
        branch_detection,
        goto_initial_pose,
        srv_robot_configuration,
    ])
