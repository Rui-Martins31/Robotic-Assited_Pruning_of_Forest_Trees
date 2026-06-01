from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RECORD_TOPICS = [
    # Planned (MoveIt)
    '/display_planned_path',                          # MoveIt planned trajectory
    '/motion_plan_request',                           # the planning request that produced it
    '/xarm6_traj_controller/joint_trajectory',        # trajectory commanded to the controller
    '/trajectory_execution_event',                    # execution start/stop/abort events

    # Actual
    '/xarm6_traj_controller/state',                   # desired vs actual vs error
    '/joint_states',                                  # actual angles, velocity, effort (joint torque)
    '/xarm/joint_states',                             # xarm driver's own joint states
    '/xarm/robot_states',                             # xarm state / err / warn / mode
    '/xarm/uf_ftsensor_ext_states',                   # end-effector force/torque (external)
    '/xarm/uf_ftsensor_raw_states',                   # end-effector force/torque (raw)
    '/tf',
    '/tf_static',

    # Monitor node
    '/monitoring/collision_caused_abnormal_current',  # collision events from node_monitor
]


def generate_launch_description():

    # Output dir
    output_dir: str      = 'output/'

    # Timestamp
    default_bag_dir: str = output_dir + 'monitoring_bags/monitor_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    arg_record = DeclareLaunchArgument(
        'record',
        default_value='true',
        description='Whether to record a rosbag alongside the monitor node.',
    )
    arg_bag_dir = DeclareLaunchArgument(
        'bag_dir',
        default_value=default_bag_dir,
        description='Output directory for the rosbag.',
    )

    monitor_node = Node(
        package='system_monitoring',
        executable='node_monitor',
        name='monitor',
        output='screen',
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_dir')] + RECORD_TOPICS,
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription([
        arg_record,
        arg_bag_dir,
        monitor_node,
        rosbag_record,
    ])
