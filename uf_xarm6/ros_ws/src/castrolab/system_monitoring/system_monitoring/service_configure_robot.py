## Notes:
# Error/Warn codes: uf_xarm/ros_ws/src/xarm_sdk/cxx/doc/xarm_api_code.md
# State/Modes list: uf_xarm/ros_ws/src/xarm_msgs/msg/RobotMsg.md
# Collision range:  uf_xarm/ros_ws/src/xarm_sdk/cxx/doc/xarm_cplus_api.md

## TODOS
# 1. Add error/warn reset.
# 2. Add mode reset


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.timer import Timer

from xarm_msgs.srv import SetInt16
from xarm_msgs.srv import Call
from custom_interfaces.srv import RobotConfig

# Constants
SRV_NAME:                  str    = 'robot_configuration'

SRV_COLLISION_SENSITIVITY: str    = '/xarm/set_collision_sensitivity'
SRV_SET_STATE:             str    = '/xarm/set_state'
SRV_CLEAN_ERROR:           str    = '/xarm/clean_error'
SRV_CLEAN_WARN:            str    = '/xarm/clean_warn'

SRV_TIMEOUT:               float  = 5.0

# Configuration
CONFIG_COLLISION_SENSITIVITY: int = 5
CONFIG_STATE:                 int = 0
CONFIG_MODE:                  int = 1

class ConfigureRobot(Node):
    def __init__(self):
        super().__init__(SRV_NAME)

        # Callback groups
        self._cb_group_srv_robot_config = MutuallyExclusiveCallbackGroup()
        self._cb_group_clients          = MutuallyExclusiveCallbackGroup()

        # One-shot timer
        self._startup_timer: Timer 

        # Service
        self.srv_robot_configuration = self.create_service(
            RobotConfig,
            SRV_NAME,
            self._robot_configuration_callback,
            callback_group = self._cb_group_srv_robot_config
        )
        self.request:  RobotConfig.Request  = RobotConfig.Request()
        self.response: RobotConfig.Response = RobotConfig.Response()

        # Collision configuration
        self.client_set_collision = self.create_client(SetInt16, SRV_COLLISION_SENSITIVITY, callback_group = self._cb_group_clients)
        self.client_set_state     = self.create_client(SetInt16, SRV_SET_STATE,             callback_group = self._cb_group_clients)
        self.client_clean_error   = self.create_client(Call,     SRV_CLEAN_ERROR,           callback_group = self._cb_group_clients)
        self.client_clean_warn    = self.create_client(Call,     SRV_CLEAN_WARN,            callback_group = self._cb_group_clients)

    # Start up procedure
    def start_service(
        self,
        timeout_sec: float = SRV_TIMEOUT
    ) -> bool:
        
        # Check services
        self.get_logger().info('Waiting for services...')

        if not self.client_set_state.wait_for_service(timeout_sec):
            self.get_logger().warn("/xarm/set_state service not available.")
            return False
        
        if not self.client_set_collision.wait_for_service(timeout_sec):
            self.get_logger().warn(
                "/xarm/set_collision_sensitivity service not available."
                "Is it enabled in xarm_api/config/xarm_params.yaml?"
            )
            return False
        
        if not self.client_clean_error.wait_for_service(timeout_sec):
            self.get_logger().warn("/xarm/clean_error service not available.")
            return False
        
        if not self.client_clean_warn.wait_for_service(timeout_sec):
            self.get_logger().warn("/xarm/clean_warn service not available.")
            return False
        
        self.get_logger().info('Services ready.')

        return True

    # Startup configuration (one-shot)
    def start_configuration(self) -> None:

        self.get_logger().info('Running startup configuration...')

        req  = RobotConfig.Request()
        resp = RobotConfig.Response()
        resp = self._robot_configuration_callback(req, resp)

        if resp.success:
            self.get_logger().info(f'Startup configuration OK: {resp.message}')
        else:
            self.get_logger().error(f'Startup configuration FAILED: {resp.message}')

        # One-shot timer
        self._startup_timer.cancel()

    # Robot configuration
    def _robot_configuration_callback(
        self,
        request: RobotConfig.Request,
        response: RobotConfig.Response,
    ) -> RobotConfig.Response:

        # Get parsed request
        request_dict = self.parse_request(request)

        # Set State
        req                          = SetInt16.Request()
        req.data                     = CONFIG_STATE # reset
        ret_state: SetInt16.Response = self.client_set_state.call(req)
        if ret_state.ret != 0:
            self.response.success = False
            self.response.message = f"/xarm/set_state failed (ret={ret_state.message})"
            return self.response

        # Set Collision
        req                              = SetInt16.Request()
        req.data                         = request_dict["collision_sensitivity"]
        ret_collision: SetInt16.Response = self.client_set_collision.call(req)
        if ret_collision.ret != 0:
            self.response.success = False
            self.response.message = f"/xarm/set_collision_sensitivity failed (ret={ret_collision.message})"
            return self.response
        
        # Set State
        req                          = SetInt16.Request()
        req.data                     = request_dict["state"]
        ret_state: SetInt16.Response = self.client_set_state.call(req)
        if ret_state.ret != 0:
            self.response.success = False
            self.response.message = f"/xarm/set_state failed (ret={ret_state.message})"
            return self.response
        
        # Return
        self.response.success = True
        self.response.message = "Robot configured."
        return self.response
    
    # Parse request
    def parse_request(
        self,
        request: RobotConfig.Request
    ) -> dict: # dict[str, int]
        
        # Default dict
        request_dict: dict = {
            "collision_sensitivity": CONFIG_COLLISION_SENSITIVITY,
            "state": CONFIG_STATE,
            "mode": CONFIG_MODE,
        }

        ## DEBUG
        self.get_logger().info(
            "Received:"
            f"  {request.collision_sensitivity = }"
            f"  {request.state = }"
            f"  {request.mode = }"
        )
        self.get_logger().info(" ")

        # Check values
        if (    (request.collision_sensitivity in [0, 1, 2, 3, 4 ,5])
            and (request.collision_sensitivity != CONFIG_COLLISION_SENSITIVITY)):
            request_dict["collision_sensitivity"] = request.collision_sensitivity

        if (    (request.state in [0, 3, 4])
            and (request.state != CONFIG_STATE)):
            request_dict["state"] = request.state

        if (    (request.mode in [0, 1, 2])
            and (request.mode != CONFIG_MODE)):
            # request_dict["mode"] = request.mode
            request_dict["mode"] = CONFIG_MODE ## TO BE REMOVED LATER. FOR DEBUG PURPOSES

        ## DEBUG
        self.get_logger().info(
            "Sent:"
            f"  {request_dict['collision_sensitivity'] = }"
            f"  {request_dict['state'] = }"
            f"  {request_dict['mode'] = }"
        )
        self.get_logger().info(" ")

        return request_dict

def main():
    rclpy.init()
    node = ConfigureRobot()
    try:
        # Start service
        if not node.start_service():
            node.get_logger().error('Required services unavailable. Shutting down.')
            return

        # Start configuration
        node._startup_timer = node.create_timer(
            0.1,
            node.start_configuration,
        )

        # Spin service
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()