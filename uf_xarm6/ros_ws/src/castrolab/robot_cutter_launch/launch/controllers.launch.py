from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Robot state monitor
    robot_state_monitor = Node(
        package='error_monitoring',
        executable='node_error_monitor',
        name='error_monitor',
        output='screen',
    )

    # Action server
    move_to_point = Node(
        package='controller',
        executable='node_move_to_point',
        name='move_to_point',
        output='screen',
    )

    # State machine
    state_machine = Node(
        package='controller',
        executable='node_state_machine',
        name='state_machine',
        output='screen',
    )

    return LaunchDescription([
        robot_state_monitor,
        move_to_point,
        state_machine,
    ])