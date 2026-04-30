import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Globals
    robot_cutter_pkg_path       = FindPackageShare('robot_description_cutter')
    robot_cutter_urdf_path: str = "gazebo.urdf.xacro"

    use_sim_time: bool = True

    # Env Variables
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('scout_description')) +
        ':' + os.path.dirname(get_package_share_directory('manipulator'))
    )

    # Robot Description
    description_file = PathJoinSubstitution(
        [robot_cutter_pkg_path, "urdf", robot_cutter_urdf_path]
    )
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", description_file,
        " gazebo:=true",
        " initial_positions_file:=../config/initial_positions.yaml",
        " tf_prefix:=robot_cutter_arm_"]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str),
        "use_sim_time": use_sim_time,
    }
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
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
            '-name', 'robot_cutter',
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

    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher_node,
        gazebo,
        spawn_robot,
        bridge,
    ])
