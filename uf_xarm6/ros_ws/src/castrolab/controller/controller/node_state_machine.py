import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from enum import Enum

from custom_interfaces.action import MoveToPoint
from custom_interfaces.msg import BufferPoints

# Constants
NODE_NAME: str            = 'state_machine'

SUB_TOPIC_BUFFER: str     = '/yolo/buffer_positions'
ACTION_MOVE_TO_POINT: str = 'move_to_point'


class State(Enum):
    IDLE      = 0
    DETECTING = 1
    EXECUTING = 2
    HOME      = 3


class StateMachine(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # State
        self._state = State.IDLE

        # Execution context
        self._buffer_points: list = []
        self._current_idx:   int  = 0
        self._total:         int  = 0

        # Goals
        self._send_goal_future    = None
        self._goal_handle         = None
        self._get_result_future   = None

        self._callback_group = ReentrantCallbackGroup()

        # Subscription
        self._sub = self.create_subscription(
            BufferPoints,
            SUB_TOPIC_BUFFER,
            self._subscription_callback,
            10,
            callback_group=self._callback_group,
        )

        # Action client
        self._action_client = ActionClient(
            self,
            MoveToPoint,
            ACTION_MOVE_TO_POINT,
            callback_group=self._callback_group
        )
        self._action_client.wait_for_server()

        self.get_logger().info('[IDLE] Waiting for buffer points...')

    def _subscription_callback(self, msg: BufferPoints) -> None:
        if self._state != State.IDLE:
            return
        self._transition_to(State.DETECTING, msg)

    def _transition_to(self, new_state: State, msg=None) -> None:
        self._state = new_state
        handlers = {
            State.IDLE:      self._on_idle,
            State.DETECTING: self._on_detecting,
            State.EXECUTING: self._on_executing,
            State.HOME:      self._on_home,
        }
        handlers[new_state](msg)


    # States
    def _on_idle(self, _=None) -> None:
        self.get_logger().info('[IDLE] Waiting for buffer points...')

    def _on_detecting(self, msg: BufferPoints) -> None:
        if msg.size == 0:
            self.get_logger().warn('[DETECTING] Empty buffer, returning to IDLE.')
            self._transition_to(State.IDLE)
            return

        self._buffer_points = list(msg.points)
        self._total         = msg.size
        self._current_idx   = 0

        self.get_logger().info(f'[DETECTING] Validated {msg.size} points.')
        self._transition_to(State.EXECUTING)

    def _on_executing(self, _=None) -> None:
        point = self._buffer_points[self._current_idx]
        self.get_logger().info(
            f'[EXECUTING] Point {self._current_idx + 1}/{self._total}: '
            f'({point.x:.3f}, {point.y:.3f}, {point.z:.3f})'
        )

        goal_msg       = MoveToPoint.Goal()
        goal_msg.point = point

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._move_feedback_callback,
        )
        self._send_goal_future.add_done_callback(self._move_goal_response_callback)

    def _on_home(self, _=None) -> None:
        # PLACEHOLDER: return robot to home position before next cut
        self.get_logger().info('[HOME] (placeholder)')
        if self._current_idx < (self._total):
            self._transition_to(State.EXECUTING)
        else:
            self.get_logger().info('[HOME] All points processed.')
            # self._transition_to(State.IDLE)


    # move_to_point action callbacks
    def _move_feedback_callback(self, feedback_msg) -> None:
        self.get_logger().info(f'[EXECUTING] {feedback_msg.feedback.status}')

    def _move_goal_response_callback(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('[EXECUTING] Goal rejected, aborting.')
            self._goal_handle = None
            self._transition_to(State.IDLE)
            return

        self.get_logger().info('[EXECUTING] Goal accepted.')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._move_result_callback)

    def _move_result_callback(self, future) -> None:
        result = future.result().result
        if not result.success:
            self.get_logger().error(f'[EXECUTING] Failed: {result.message}. Aborting.')
            self._transition_to(State.IDLE)
            return

        self.get_logger().info(f'[EXECUTING] Point {self._current_idx + 1} succeeded.')
        self._current_idx += 1
        self._transition_to(State.HOME)


def main():
    rclpy.init()
    node     = StateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()