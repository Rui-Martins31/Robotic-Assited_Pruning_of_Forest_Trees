from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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
        move_to_point,
        state_machine,
    ])