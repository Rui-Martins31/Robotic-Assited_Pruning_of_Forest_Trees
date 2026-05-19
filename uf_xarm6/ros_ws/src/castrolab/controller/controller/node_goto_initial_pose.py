import rclpy
from rclpy.node import Node

from xarm_msgs.srv import PlanJoint, PlanExec

NODE_NAME: str = 'goto_initial_pose'

SERVICE_PLAN_JOINT: str = 'xarm_joint_plan'
SERVICE_EXEC_PLAN:  str = 'xarm_exec_plan'

DEFAULT_JOINT_ANGLES: list = [-2.0944,-0.785398,-0.785398,0.0,0.0,0.0] # rads


class GotoInitialPose(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self._plan_client = self.create_client(PlanJoint, SERVICE_PLAN_JOINT)
        self._exec_client = self.create_client(PlanExec,  SERVICE_EXEC_PLAN)

        self.get_logger().info(f'Waiting for {SERVICE_PLAN_JOINT} and {SERVICE_EXEC_PLAN} services...')
        self._plan_client.wait_for_service()
        self._exec_client.wait_for_service()
        self.get_logger().info('Services ready.')

        self._send_plan()

    def _send_plan(self) -> None:
        angles: list = DEFAULT_JOINT_ANGLES
        self.get_logger().info(f'Planning to joint angles: {angles}')

        request = PlanJoint.Request()
        request.target = angles

        future = self._plan_client.call_async(request)
        future.add_done_callback(self._plan_response_callback)

    def _plan_response_callback(self, future) -> None:
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Planning failed.')
            rclpy.shutdown()
            return

        self.get_logger().info('Planning succeeded, executing...')

        request = PlanExec.Request()
        request.wait = True

        future = self._exec_client.call_async(request)
        future.add_done_callback(self._exec_response_callback)

    def _exec_response_callback(self, future) -> None:
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Execution failed.')
        else:
            self.get_logger().info('Execution succeeded. Shutting down.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GotoInitialPose()
    rclpy.spin(node)
    node.destroy_node()
