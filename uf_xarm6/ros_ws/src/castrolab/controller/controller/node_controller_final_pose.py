import rclpy
from rclpy.node import Node

import numpy as np

from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Point, Pose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Constants
NODE_NAME: str = 'controller_final_pose'

SUB_TOPIC_NAME_WORLD_POSITION: str = '/yolo/position_vector_world_frame'

SERVICE_PLAN_POSE: str = 'xarm_pose_plan'
SERVICE_EXEC_PLAN: str = 'xarm_exec_plan'

ARM_JOINT_NAME_BASE: str = 'link1'
ARM_JOINT_NAME_CAM:  str = 'link_eef'


class ArmController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Service clients
        self._plan_client = self.create_client(PlanPose, SERVICE_PLAN_POSE)
        self._exec_client = self.create_client(PlanExec, SERVICE_EXEC_PLAN)

        self.get_logger().info(f'Waiting for {SERVICE_PLAN_POSE} and {SERVICE_EXEC_PLAN} services...')
        self._plan_client.wait_for_service()
        self._exec_client.wait_for_service()
        self.get_logger().info('Services ready.')

        # Subscriber
        self.subscription = self.create_subscription(
            Point,
            SUB_TOPIC_NAME_WORLD_POSITION,
            self.subscription_callback,
            10
        )
        self.is_executing: bool = False

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def subscription_callback(self, msg: Point) -> None:
        if not self.is_executing:
            self.is_executing = True
            self.plan_and_execute(msg)

    def plan_and_execute(self, msg: Point) -> None:
        self.get_logger().info(f'Received target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')

        # Compute pose
        current_position   = self.get_current_joint_position()
        target_position    = np.array([msg.x, msg.y, msg.z])
        direction          = target_position - current_position
        target_orientation = self.get_direction_to_quaternion(direction)

        safe_z = float(target_position[2]) if target_position[2] >= 0.2 else 0.2

        # Build the pose goal
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
        plan_future.add_done_callback(self._plan_response_callback)

    def _plan_response_callback(self, future) -> None:
        # Planning result
        result = future.result()
        if result is None:
            self.get_logger().error('Planning service call failed.')
            # self.is_executing = False
            return

        if not result.success:
            self.get_logger().error('Planning failed.')
            # self.is_executing = False
            return

        self.get_logger().info('Planning succeeded, executing...')

        # Execute
        # exec_request = PlanExec.Request()
        # exec_request.wait = True

        # exec_future = self._exec_client.call_async(exec_request)
        # exec_future.add_done_callback(self._exec_response_callback)

    def _exec_response_callback(self, future) -> None:
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Execution failed.')
        else:
            self.get_logger().info('Execution succeeded.')
        # self.is_executing = False

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

        cross = np.cross(reference, direction)
        dot   = np.dot(reference, direction)

        # Handle antiparallel case
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
        joint_name_base: str = ARM_JOINT_NAME_BASE,
        joint_name_target: str = ARM_JOINT_NAME_CAM
    ) -> np.ndarray:
        try:
            t = self.tf_buffer.lookup_transform(
                joint_name_base,
                joint_name_target,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            p = t.transform.translation
            return np.array([float(p.x), float(p.y), float(p.z)])

        except TransformException as ex:
            self.get_logger().warn(f'Could not get EEF position: {ex}')
            return np.array([0.0, 0.0, 0.0])


def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
