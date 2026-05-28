import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from xarm_msgs.srv import PlanJoint, PlanExec
from std_srvs.srv import Trigger

NODE_NAME: str = 'goto_initial_pose'

SRV_PLAN_JOINT: str = 'xarm_joint_plan'
SRV_EXEC_PLAN:  str = 'xarm_exec_plan'
SRV_GOTO_INIT:  str = NODE_NAME

# DEFAULT_JOINT_ANGLES: list = [-2.0944,-0.785398,-0.785398,0.0,0.0,0.0] # rads
DEFAULT_JOINT_ANGLES: list = [-2.0944,-1.309,-0.523599,0.0,0.610865,0.0] # rads


class GotoInitialPose(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        callback_group    = ReentrantCallbackGroup()

        # X-arm services
        self._plan_client = self.create_client(PlanJoint, SRV_PLAN_JOINT, callback_group=callback_group)
        self._exec_client = self.create_client(PlanExec,  SRV_EXEC_PLAN,  callback_group=callback_group)

        self.get_logger().info(f'Waiting for {SRV_PLAN_JOINT} and {SRV_EXEC_PLAN} services...')
        self._plan_client.wait_for_service()
        self._exec_client.wait_for_service()
        self.get_logger().info('Services ready.')

        # Go to initial pose service
        self.src_goto_initial_pose = self.create_service(
            Trigger,
            SRV_GOTO_INIT,
            self._srv_goto_initial_pose_callback,
            callback_group=callback_group
        )
        self._is_executing: bool = False

        # Start up
        self._init_timer = self.create_timer(
            0.1,
            self._on_startup,
            callback_group=callback_group
        )

    # Start up
    def _on_startup(self) -> None:
        self._init_timer.cancel()
        request  = Trigger.Request()
        response = Trigger.Response()
        self._srv_goto_initial_pose_callback(request, response)

    # Service callback
    def _srv_goto_initial_pose_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:

        # Check
        if self._is_executing:
            response.success = False
            response.message = 'Executing another service call.'
            return response

        # Plan
        self.get_logger().info(f'Planning to joint angles: {DEFAULT_JOINT_ANGLES}')
        plan_request        = PlanJoint.Request()
        plan_request.target = DEFAULT_JOINT_ANGLES

        result = self._plan_client.call(plan_request)
        if result is None or not result.success:
            response.success = False
            response.message = 'Planning failed.'
            self.get_logger().error(response.message)
            self._is_executing = False
            return response

        # Execute
        self.get_logger().info('Planning succeeded, executing...')
        exec_request      = PlanExec.Request()
        exec_request.wait = True

        result = self._exec_client.call(exec_request)
        if result is None or not result.success:
            response.success = False
            response.message = 'Execution failed.'
            self.get_logger().error(response.message)
        else:
            response.success = True
            response.message = 'Execution succeeded.'
            self.get_logger().info(response.message)

        self._is_executing = False
        return response


def main():
    rclpy.init()
    node = GotoInitialPose()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()