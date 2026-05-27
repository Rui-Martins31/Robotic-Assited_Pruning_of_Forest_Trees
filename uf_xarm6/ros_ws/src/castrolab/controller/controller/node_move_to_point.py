import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading

import numpy as np

from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Point, Pose
from custom_interfaces.action import MoveToPoint

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Constants
NODE_NAME: str            = 'move_to_point'

ACTION_MOVE_TO_POINT: str = NODE_NAME

SERVICE_PLAN_POSE: str    = 'xarm_pose_plan'
SERVICE_EXEC_PLAN: str    = 'xarm_exec_plan'

ARM_JOINT_NAME_BASE: str  = 'link_base'
ARM_JOINT_NAME_CAM:  str  = 'link_eef'


class MoveToPointServer(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.callback_group = ReentrantCallbackGroup()

        # Service clients
        self._plan_client = self.create_client(PlanPose, SERVICE_PLAN_POSE, callback_group=self.callback_group)
        self._exec_client = self.create_client(PlanExec, SERVICE_EXEC_PLAN, callback_group=self.callback_group)

        self.get_logger().info(f'Waiting for {SERVICE_PLAN_POSE} and {SERVICE_EXEC_PLAN} services...')
        self._plan_client.wait_for_service()
        self._exec_client.wait_for_service()
        self.get_logger().info('Services ready.')

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Waiting for TF tree ({ARM_JOINT_NAME_BASE} -> {ARM_JOINT_NAME_CAM})...')
        while rclpy.ok():
            if self.tf_buffer.can_transform(
                ARM_JOINT_NAME_BASE,
                ARM_JOINT_NAME_CAM,
                rclpy.time.Time()
            ):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('TF tree ready.')

        # Action server
        self._action_server = ActionServer(
            self,
            MoveToPoint,
            ACTION_MOVE_TO_POINT,
            execute_callback = self.execute_callback,
            goal_callback    = self.goal_callback,
            cancel_callback  = self.cancel_callback,
            callback_group   = self.callback_group,
        )
        self.get_logger().info('Action server ready.')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return MoveToPoint.Result(success=False, message='Canceled')

        goal_handle.publish_feedback(MoveToPoint.Feedback(status='planning'))
        success = self._plan_and_execute(goal_handle.request.point)

        if not success:
            goal_handle.abort()
            return MoveToPoint.Result(success=False, message='Plan/execute failed')

        goal_handle.succeed()
        return MoveToPoint.Result(success=True, message='Motion complete')

    def _plan_and_execute(
        self,
        point: Point
    ) -> bool:
        
        self.get_logger().info(f'Target: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})')

        # Compute target pose
        current_position   = self._get_current_eef_position()
        target_position    = np.array([point.x, point.y, point.z])
        direction          = target_position - current_position
        target_orientation = self._direction_to_quaternion(direction)

        safe_z = float(target_position[2]) if target_position[2] >= 0.2 else 0.2

        target_pose = Pose()
        target_pose.position.x    = float(target_position[0])
        target_pose.position.y    = float(target_position[1])
        target_pose.position.z    = safe_z
        target_pose.orientation.x = float(target_orientation[0])
        target_pose.orientation.y = float(target_orientation[1])
        target_pose.orientation.z = float(target_orientation[2])
        target_pose.orientation.w = float(target_orientation[3])

        # Plan
        plan_request        = PlanPose.Request()
        plan_request.target = target_pose

        plan_future = self._plan_client.call_async(plan_request)
        self._block_until_done(plan_future)

        plan_result = plan_future.result()
        if plan_result is None or not plan_result.success:
            self.get_logger().error('Planning failed.')
            return False

        self.get_logger().info('Planning succeeded, executing...')

        # Execute
        exec_request      = PlanExec.Request()
        exec_request.wait = True

        exec_future = self._exec_client.call_async(exec_request)
        self._block_until_done(exec_future)

        exec_result = exec_future.result()
        if exec_result is None or not exec_result.success:
            self.get_logger().error('Execution failed.')
            return False

        self.get_logger().info('Execution succeeded.')
        return True

    def _block_until_done(self, future) -> None:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

    def _direction_to_quaternion(
        self,
        direction: np.ndarray,
        reference: np.ndarray = np.array([0, 0, 1])
    ) -> np.ndarray:

        direction_norm = np.linalg.norm(direction)
        if direction_norm >= 1e-6:
            direction = direction / direction_norm
        else:
            self.get_logger().warning('Target too close to current position, using identity orientation.')
            return np.array([0.0, 0.0, 0.0, 1.0])

        reference = reference / np.linalg.norm(reference)
        cross     = np.cross(reference, direction)
        dot       = np.dot(reference, direction)

        if np.linalg.norm(cross) < 1e-6:
            if dot > 0:
                return np.array([0.0, 0.0, 0.0, 1.0])
            else:
                perp = np.array([1, 0, 0]) if abs(reference[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(reference, perp)
                axis /= np.linalg.norm(axis)
                return np.array([*axis, 0.0])

        w = 1.0 + dot
        q = np.array([*cross, w])
        return q / np.linalg.norm(q)

    def _get_current_eef_position(self) -> np.ndarray:
        for _ in range(5):
            try:
                t = self.tf_buffer.lookup_transform(
                    ARM_JOINT_NAME_BASE,
                    ARM_JOINT_NAME_CAM,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                p = t.transform.translation
                self.get_logger().info(f'Current EEF position: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
                return np.array([float(p.x), float(p.y), float(p.z)])

            except TransformException as ex:
                self.get_logger().warn(f'Could not get EEF position: {ex}')

        return np.array([0.0, 0.0, 0.0])


def main():
    rclpy.init()
    node     = MoveToPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()