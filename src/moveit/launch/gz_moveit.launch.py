import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



# Helpers
# from moveit.config.robot_description import get_robot_description


def generate_launch_description():

    # Variables
    use_sim_time = True
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    use_gazebo   = "true"

    # Package
    pkg_name_manipulator = "manipulator"
    pkg_name_moveit      = "moveit"
    pkg_name_gazebo      = "ros_gz_sim"
    pkg_name_controller  = "controller_manager"

    # World
    gazebo_world_path = os.path.join(
        get_package_share_directory(pkg_name_manipulator),
        'worlds',
        'world'
    )
    gazebo_world = "forest_pruning_world"


    # Robot Description
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare(pkg_name_moveit), "config", "ur.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    description_file   = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "gazebo:=",
            use_gazebo
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Environment
    gazebo_resource_path_env = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare(pkg_name_manipulator), ".."]),
        separator=':'
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name_gazebo), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={
            'gz_args': f'-r {gazebo_world_path}.sdf --render-engine ogre',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Bridges
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Robot
    spawn_robot = Node(
        package=pkg_name_gazebo,
        executable="create",
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur',
            '-world', gazebo_world
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Join Broadcaster
    joint_state_broadcaster_spawner = Node(
        package=pkg_name_controller,
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Joint Controller
    joint_trajectory_controller_spawner = Node(
        package=pkg_name_controller,
        executable="spawner",
        arguments=[
            "ur5_arm_controller",#"joint_trajectory_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name_moveit), 'launch', 'move_group.launch.py')]
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time)
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name_moveit), 'launch', 'moveit_rviz.launch.py')]
        ),
        launch_arguments={
            # 'rviz_config': "./src/moveit/config/moveit.rviz",
            'rviz_config': PathJoinSubstitution([FindPackageShare(pkg_name_moveit), "config/moveit.rviz"]),
            'use_sim_time': str(use_sim_time)
        }.items()
    )


    # Launch
    return LaunchDescription(
        declared_arguments +
        [
            set_sim_time,
            gazebo_resource_path_env,
            gazebo,

            clock_bridge,
            image_bridge,
            
            node_robot_state_publisher,
            spawn_robot,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,

            # Delay moveit start so that all the controllers have time to start up before it
            # TO BE REMOVED LATER
            TimerAction(
                period=5.0,
                actions=[moveit, rviz]
            )
            # moveit,
            # rviz
        ]
    )