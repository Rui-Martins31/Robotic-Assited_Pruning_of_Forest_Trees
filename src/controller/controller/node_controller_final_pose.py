import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Point, PoseStamped, PoseStamped

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

    def plan_and_execute(self, msg: Point):
        ## DEBUG
        print("\n")
        self.get_logger().info(f"Received new point.")

        # Convert Point to PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id    = "world"
        goal_pose.pose.position.x    = msg.x
        goal_pose.pose.position.y    = msg.y
        goal_pose.pose.position.z    = msg.z
        goal_pose.pose.orientation.w = 1.0

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

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()