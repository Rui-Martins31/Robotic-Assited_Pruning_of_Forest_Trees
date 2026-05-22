import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np

from xarm_msgs.srv import PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Point, Pose
from custom_interfaces.msg import BufferPoints

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Constants
NODE_NAME: str             = 'controller_final_pose'
SUB_TOPIC_NAME_BUFFER: str = '/yolo/buffer_positions'
SERVICE_PLAN_POSE:  str    = '/xarm_pose_plan'
SERVICE_PLAN_JOINT: str    = '/xarm_joint_plan'
SERVICE_EXEC_PLAN:  str    = '/xarm_exec_plan'
ARM_JOINT_NAME_BASE: str   = 'link_base'
ARM_JOINT_NAME_CAM:  str   = 'link_eef'
DEFAULT_JOINT_ANGLES: list = [-2.0944,-0.785398,-0.785398,0.0,0.0,0.0] # rads

class ArmController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.callback_group = ReentrantCallbackGroup()

        # Service clients
        self._plan_pose_client  = self.create_client(PlanPose,  SERVICE_PLAN_POSE,  callback_group=self.callback_group)
        self._plan_joint_client = self.create_client(PlanJoint, SERVICE_PLAN_JOINT, callback_group=self.callback_group)
        self._exec_client       = self.create_client(PlanExec,  SERVICE_EXEC_PLAN,  callback_group=self.callback_group)

        self.get_logger().info(f'Waiting for {SERVICE_PLAN_POSE}, {SERVICE_PLAN_JOINT} and {SERVICE_EXEC_PLAN} services...')
        self._plan_pose_client.wait_for_service()
        self._plan_joint_client.wait_for_service()
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

        # Subscriber
        self.subscription = self.create_subscription(
            BufferPoints,
            SUB_TOPIC_NAME_BUFFER,
            self.subscription_callback,
            10,
            callback_group=self.callback_group
        )
        self.buffer_size:   int         = 0
        self.buffer_points: list[Point] = []

        self.is_executing:  bool        = False

    def subscription_callback(self, msg: BufferPoints) -> None:
        if self.is_executing:
            return

        self.is_executing  = True
        self.buffer_size   = msg.size
        self.buffer_points = msg.points

        try:
            for idx in range(self.buffer_size):
                self.get_logger().info(f'Processing point {idx + 1}/{self.buffer_size}')

                # Point
                success = self.plan_and_execute(self.buffer_points[idx])
                if not success:
                    self.get_logger().error(f'Failed at point {idx + 1}, aborting.')
                    break

                # Initial pose
                success = self.goto_initial_position()
                if not success:
                    self.get_logger().error(f'Failed at point {idx + 1}, aborting.')
                    break
                
        finally:
            # self.is_executing = False
            pass

    def plan_and_execute(self, msg: Point) -> bool:
        self.get_logger().info(f'Target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')

        # Compute pose
        current_position   = self.get_current_joint_position()
        target_position    = np.array([msg.x, msg.y, msg.z])
        direction          = target_position - current_position
        target_orientation = self.get_direction_to_quaternion(direction)
        safe_z             = float(target_position[2]) if target_position[2] >= 0.2 else 0.2

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

        plan_future = self._plan_pose_client.call_async(plan_request)
        self.executor.spin_until_future_complete(plan_future)

        plan_result = plan_future.result()
        if plan_result is None or not plan_result.success:
            self.get_logger().error('Planning failed.')
            return False

        self.get_logger().info('Planning succeeded, executing...')

        # Execute
        exec_request      = PlanExec.Request()
        exec_request.wait = True

        exec_future = self._exec_client.call_async(exec_request)
        self.executor.spin_until_future_complete(exec_future)

        exec_result = exec_future.result()
        if exec_result is None or not exec_result.success:
            self.get_logger().error('Execution failed.')
            return False

        self.get_logger().info('Execution succeeded.')
        return True

    def goto_initial_position(self) -> bool:
        self.get_logger().info(f'Going back to initial position. Joint angles: {DEFAULT_JOINT_ANGLES}')

        plan_request        = PlanJoint.Request()
        plan_request.target = DEFAULT_JOINT_ANGLES

        plan_future = self._plan_joint_client.call_async(plan_request)
        self.executor.spin_until_future_complete(plan_future)

        plan_result = plan_future.result()
        if plan_result is None or not plan_result.success:
            self.get_logger().error('Planning to home failed.')
            return False

        exec_request      = PlanExec.Request()
        exec_request.wait = True

        exec_future = self._exec_client.call_async(exec_request)
        self.executor.spin_until_future_complete(exec_future)

        exec_result = exec_future.result()
        if exec_result is None or not exec_result.success:
            self.get_logger().error('Execution to home failed.')
            return False

        self.get_logger().info('Returned to initial position.')
        return True

    def get_direction_to_quaternion(
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

    def get_current_joint_position(
        self,
        joint_name_base: str   = ARM_JOINT_NAME_BASE,
        joint_name_target: str = ARM_JOINT_NAME_CAM
    ) -> np.ndarray:

        for _ in range(5):
            try:
                t = self.tf_buffer.lookup_transform(
                    joint_name_base,
                    joint_name_target,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                p = t.transform.translation
                self.get_logger().info(f'Current position: {[float(p.x), float(p.y), float(p.z)]}')
                return np.array([float(p.x), float(p.y), float(p.z)])

            except TransformException as ex:
                self.get_logger().warn(f'Could not get EEF position: {ex}')

        return np.array([0.0, 0.0, 0.0])


def main():
    rclpy.init()
    node     = ArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
