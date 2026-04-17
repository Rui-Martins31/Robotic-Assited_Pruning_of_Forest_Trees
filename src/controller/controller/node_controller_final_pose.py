import rclpy
from rclpy.node import Node

import numpy as np

from moveit.planning import MoveItPy

from geometry_msgs.msg import Point, PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Constants
NODE_NAME: str = 'controller_final_pose'

SUB_TOPIC_NAME_WORLD_POSITION: str = '/yolo/position_vector_world_frame'

POSE_JOINT_NAME: str = "wrist_3_link"

class ArmController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        
        # MoveItPy
        self.arm            = MoveItPy(node_name="moveit_py")
        self.planning_group = self.arm.get_planning_component("ur5_arm")

        # Subscriber
        self.subscription   = self.create_subscription(
            Point,
            SUB_TOPIC_NAME_WORLD_POSITION,
            self.plan_and_execute,
            10
        )

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def plan_and_execute(
        self,
        msg: Point
    ) -> None:
        
        ## DEBUG
        self.get_logger().info(f"Received new point. ---------------------------")

        # Direction
        current_position = self.get_current_joint_position(POSE_JOINT_NAME)
        target_position  = np.array([msg.x, msg.y, msg.z])
        direction        = target_position - current_position

        # Quaternion
        target_orientation: np.ndarray = self.get_direction_to_quaternion(direction)
        
        # PoseStamped
        goal_pose = PoseStamped()

        goal_pose.header.frame_id    = "world"
        goal_pose.pose.position.x    = target_position[0]
        goal_pose.pose.position.y    = target_position[1]
        goal_pose.pose.position.z    = target_position[2]

        goal_pose.pose.orientation.x = target_orientation[0]
        goal_pose.pose.orientation.y = target_orientation[1]
        goal_pose.pose.orientation.z = target_orientation[2]
        goal_pose.pose.orientation.w = target_orientation[3]

        # Goal State
        self.planning_group.set_goal_state(
            pose_stamped_msg=goal_pose,
            pose_link=POSE_JOINT_NAME
        )
        
        # Plan
        plan_result = self.planning_group.plan()
        
        # Execute
        if plan_result:
            self.get_logger().info("Plan successful, executing...")
            self.arm.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("Planning failed!")

    def get_direction_to_quaternion(
        self,
        direction: np.ndarray,
        reference: np.ndarray = np.array([0, 0, 1])
    ) -> np.ndarray:
        
        # Normalize
        direction_norm   = np.linalg.norm(direction)
        if direction_norm >= 1e-6:
            direction    = direction / direction_norm
        else:
            self.get_logger().warning("Target too close to current position")
            return np.array([0.0, 0.0, 0.0, 1.0])
        
        reference = reference / np.linalg.norm(reference)

        cross = np.cross(reference, direction)
        dot   = np.dot(reference, direction)

        # Handle antiparallel case
        if np.linalg.norm(cross) < 1e-6:
            if dot > 0:
                return np.array([0.0, 0.0, 0.0, 1.0])  # identity
            else:
                perp = np.array([1, 0, 0]) if abs(reference[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(reference, perp)
                axis /= np.linalg.norm(axis)
                return np.array([*axis, 0.0])  # 180° rotation

        w = 1.0 + dot
        q = np.array([*cross, w])
        return q / np.linalg.norm(q)

    def get_current_joint_position(
        self,
        joint_name: str
    ) -> np.ndarray:
        try:
            now = rclpy.time.Time()
            t   = self.tf_buffer.lookup_transform(
                'world',
                joint_name,
                now
            )

            # Get rotation
            p = t.transform.translation

            return np.array([
                float(p.x),
                float(p.y),
                float(p.z)
            ])

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

            return np.array([
                float(0.0),
                float(0.0),
                float(0.0),
            ])

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()