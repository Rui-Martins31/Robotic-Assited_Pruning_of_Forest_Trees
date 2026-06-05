import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import MoveToPoint
from custom_interfaces.msg import BufferPoints
from custom_interfaces.msg import Error
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

# Constants
NODE_NAME: str            = 'state_machine'

SUB_TOPIC_BUFFER:     str = '/yolo/buffer_positions'
SUB_TOPIC_ERROR:      str = '/monitoring/collision_caused_abnormal_current'
ACTION_MOVE_TO_POINT: str = '/move_to_point'
SRV_GOTO_INIT:        str = '/goto_initial_pose'

SRV_TIMEOUT:          float = 5.0


class State(Enum):
    IDLE      = 0
    DETECTING = 1
    EXECUTING = 2
    HOME      = 3
    ERROR     = 4


class StateMachine(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # State
        self._state = State.IDLE

        # Execution context
        self._buffer_points: list = []
        self._current_idx:   int  = 0
        self._total:         int  = 0

        # Handle goal
        self._goal_handle  = None
        self._error_active = False
        self._goal_lock    = threading.Lock()

        # Callback groups
        self._cb_group_buffer = MutuallyExclusiveCallbackGroup()
        self._cb_group_error  = MutuallyExclusiveCallbackGroup()
        self._cb_group_act    = MutuallyExclusiveCallbackGroup()
        self._cb_group_srv    = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.subscription_buffer = self.create_subscription(
            BufferPoints,
            SUB_TOPIC_BUFFER,
            self._subscription_callback_buffer,
            10,
            callback_group=self._cb_group_buffer,
        )
        self.subscription_error = self.create_subscription(
            Error,
            SUB_TOPIC_ERROR,
            self._subscription_callback_error,
            10,
            callback_group=self._cb_group_error,
        )

        # Action client
        self.action_client = ActionClient(
            self,
            MoveToPoint,
            ACTION_MOVE_TO_POINT,
            callback_group=self._cb_group_act,
        )
        while not self.action_client.wait_for_server(timeout_sec=SRV_TIMEOUT):
            self.get_logger().info(f'Action {ACTION_MOVE_TO_POINT} not available, waiting...')

        # Goto_initial_pose service
        self.srv_client_goto_init = self.create_client(
            Trigger,
            SRV_GOTO_INIT,
            callback_group=self._cb_group_srv,
        )
        while not self.srv_client_goto_init.wait_for_service(timeout_sec=SRV_TIMEOUT):
            self.get_logger().info(f'Service {SRV_GOTO_INIT} not available, waiting...')

        # Start
        self._transition_to(State.IDLE)

    # Subscriptions
    def _subscription_callback_buffer(self, msg: BufferPoints) -> None:
        if self._state != State.IDLE:
            return
        self._transition_to(State.DETECTING, msg)

    def _subscription_callback_error(self, msg: Error) -> None:
        if msg.error:
            # Flag the error and cancel any in-flight motion. on_executing keys
            # off the flag (not the final status) because MoveIt may ABORT the
            # goal before our cancel is processed.
            with self._goal_lock:
                self._error_active = True
                handle = self._goal_handle
            if handle is not None:
                self.get_logger().warn('[ERROR] Collision detected, canceling motion...')
                handle.cancel_goal_async()
            else:
                self._transition_to(State.ERROR)
        else:
            with self._goal_lock:
                self._error_active = False
            self.get_logger().info(f'{msg.message}')
            self._transition_to(State.HOME)

    def _transition_to(self, new_state: State, msg=None) -> None:
        self._state = new_state
        handlers = {
            State.IDLE:      self.on_idle,
            State.DETECTING: self.on_detecting,
            State.EXECUTING: self.on_executing,
            State.HOME:      self.on_home,
            State.ERROR:     self.on_error,
        }
        handlers[new_state](msg)

    # States
    def on_idle(self, _=None) -> None:
        self.get_logger().info('[IDLE] Waiting for buffer points...')

    def on_detecting(self, msg: BufferPoints) -> None:
        if msg.size == 0:
            self.get_logger().warn('[DETECTING] Empty buffer, returning to IDLE.')
            self._transition_to(State.IDLE)
            return

        self._buffer_points = list(msg.points)
        self._total         = msg.size
        self._current_idx   = 0

        self.get_logger().info(f'[DETECTING] Validated {msg.size} points.')
        self._transition_to(State.EXECUTING)

    def on_executing(self, _=None) -> None:
        point = self._buffer_points[self._current_idx]
        self.get_logger().info(
            f'[EXECUTING] Point {self._current_idx + 1}/{self._total}: '
            f'({point.x:.3f}, {point.y:.3f}, {point.z:.3f})'
        )

        goal_msg       = MoveToPoint.Goal()
        goal_msg.point = point

        # Send the goal async
        # Can cancel it from the error callback
        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._move_feedback_callback,
        )
        goal_handle = self._await_future(send_future)

        if not goal_handle.accepted:
            self.get_logger().error('[EXECUTING] Goal rejected, going HOME.')
            self._transition_to(State.HOME)
            return
        self.get_logger().info('[EXECUTING] Goal accepted.')

        with self._goal_lock:
            self._goal_handle = goal_handle

        result_response = self._await_future(goal_handle.get_result_async())

        with self._goal_lock:
            self._goal_handle  = None
            error_active       = self._error_active

        # If an error triggered the stop, go to ERROR regardless of the final
        # status (CANCELED if our cancel won the race, ABORTED if MoveIt did).
        if error_active:
            self.get_logger().warn('[EXECUTING] Motion stopped by error. Going to ERROR.')
            self._transition_to(State.ERROR)
            return

        status = result_response.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'[EXECUTING] Motion not successful (status={status}). Going HOME.'
            )
            self._transition_to(State.HOME)
            return

        result = result_response.result
        if not result.success:
            self.get_logger().error(f'[EXECUTING] Failed: {result.message}. Going HOME.')
            self._transition_to(State.HOME)
            return

        self.get_logger().info(f'[EXECUTING] Point {self._current_idx + 1} succeeded.')
        self._current_idx += 1
        self._transition_to(State.HOME)

    def on_home(self, _=None) -> None:
        self.get_logger().info('[HOME] Requesting to go to initial position!')

        while True:
            response = self.srv_client_goto_init.call(Trigger.Request())
            if response.success:
                break
            self.get_logger().warn(
                f"[HOME] Couldn't perform the motion. Message: {response.message}. Retrying..."
            )

        self.get_logger().info('[HOME] Got to initial position!')
        if self._current_idx < self._total:
            self._transition_to(State.EXECUTING)
        else:
            self.get_logger().info('[HOME] All points processed.')
            ## Note:
            ## Uncomment this to restart the loop
            # self._transition_to(State.IDLE)

    def on_error(self, _=None) -> None:
        self.get_logger().info('[ERROR] Error detected!')

    # Helpers
    def _await_future(self, future):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()
        return future.result()

    # Action feedback
    def _move_feedback_callback(self, feedback_msg) -> None:
        self.get_logger().info(f'[EXECUTING] {feedback_msg.feedback.status}')


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