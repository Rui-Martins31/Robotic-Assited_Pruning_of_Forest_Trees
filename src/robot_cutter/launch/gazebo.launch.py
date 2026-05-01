import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction#, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Globals
    robot_cutter_urdf_path: str = "gazebo.urdf.xacro"

    # Variables
    use_gazebo:   bool   = True
    use_sim_time: bool   = True
    robot_name:   str    = "robot_cutter"
    tf_prefix:    str    = "robot_cutter_arm_"

    # Packages
    pkg_name_cutter:      str = "robot_cutter"
    pkg_name_manipulator: str = "manipulator"
    pkg_name_moveit:      str = "moveit"
    pkg_name_gazebo:      str = "ros_gz_sim"
    pkg_name_controller:  str = "controller_manager"
    pkg_name_scout_des:   str = "scout_description"
    pkg_name_robot_state: str = "robot_state_publisher"

    # Env Variables
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory(pkg_name_scout_des)) +
        ':' + 
        os.path.dirname(get_package_share_directory(pkg_name_manipulator))
    )

    # Robot Description
    description_file = PathJoinSubstitution(
        [FindPackageShare(pkg_name_cutter), "urdf", robot_cutter_urdf_path]
    )
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", description_file,
        " gazebo:=", str(use_gazebo),
        # " initial_positions_file:=../config/initial_positions.yaml",
        " tf_prefix:=", tf_prefix,
        " robot_name:=", robot_name,
        " ros2_controllers_file:=", PathJoinSubstitution([FindPackageShare(pkg_name_cutter), "config", "ros2_controllers.yaml"])]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }
    robot_state_publisher_node = Node(
        package=pkg_name_robot_state,
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}],
    )

    ## GAZEBO
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg_name_gazebo), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf --render-engine ogre',
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Publish world->base_link
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen',
    )

    ## MOVEIT
    # Join Broadcaster
    joint_state_broadcaster_spawner = Node(
        package=pkg_name_controller,
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )

    # Joint Controller
    joint_trajectory_controller_spawner = Node(
        package=pkg_name_controller,
        executable="spawner",
        arguments=[
            "ur5_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", os.path.join(get_package_share_directory(pkg_name_cutter), "config", "ros2_controllers.yaml")
        ],
        output="screen",
    )

    # MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name_moveit), 'launch', 'move_group.launch.py')]
        ),
        launch_arguments={
            'use_sim_time':              str(use_sim_time),
            'robot_name':                robot_name,
            'tf_prefix':                 tf_prefix,
            'joint_limits_file':         os.path.join(get_package_share_directory(pkg_name_cutter), 'config', 'joint_limits.yaml'),
            'moveit_controllers_file':   os.path.join(get_package_share_directory(pkg_name_cutter), 'config', 'moveit_controllers.yaml'),
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg_name_moveit), 'launch', 'moveit_rviz.launch.py')]
        ),
        launch_arguments={
            'rviz_config': PathJoinSubstitution([FindPackageShare(pkg_name_moveit), "config/moveit.rviz"]),
            'use_sim_time': str(use_sim_time),
            'robot_name': robot_name,
            'tf_prefix':  tf_prefix,
        }.items()
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        gz_resource_path,
        
        robot_state_publisher_node,
        
        gazebo,
        spawn_robot,
        
        bridge,
        world_tf,

        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        TimerAction(
            period=5.0,
            actions=[moveit, rviz]
        )
        # moveit,
        # rviz
    ])
