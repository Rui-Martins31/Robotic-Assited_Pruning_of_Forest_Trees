## Notes:
# Error/Warn codes: uf_xarm/ros_ws/src/xarm_sdk/cxx/doc/xarm_api_code.md
# State/Modes list: uf_xarm/ros_ws/src/xarm_msgs/msg/RobotMsg.md
# Collision range:  uf_xarm/ros_ws/src/xarm_sdk/cxx/doc/xarm_cplus_api.md

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import SetInt16ById
from xarm_msgs.srv import Call
from custom_interfaces.msg import Error
from custom_interfaces.srv import RobotConfig

# Constants
NODE_NAME: str = 'monitor'

SUB_TOPIC_ROBOT_STATES:    str = '/xarm/robot_states'
PUB_TOPIC_ERROR_COLLISION: str = '/monitoring/collision_caused_abnormal_current'

SRV_CLEAN_ERROR:   str = '/xarm/clean_error'
SRV_CLEAN_WARN:    str = '/xarm/clean_warn'
SRV_MOTION_ENABLE: str = '/xarm/motion_enable'
SRV_ROBOT_CONFIG:  str = '/robot_configuration'


class MonitorNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # Flags
        self.flag_error_detected: bool = False

        # Callback groups
        self._cb_group_pubsub  = MutuallyExclusiveCallbackGroup()
        self._cb_group_clients = MutuallyExclusiveCallbackGroup()

        # X-arm services
        self.client_clean_error   = self.create_client(Call,         SRV_CLEAN_ERROR,   callback_group=self._cb_group_clients)
        self.client_clean_warn    = self.create_client(Call,         SRV_CLEAN_WARN,    callback_group=self._cb_group_clients)
        self.client_motion_enable = self.create_client(SetInt16ById, SRV_MOTION_ENABLE, callback_group=self._cb_group_clients)
        self.client_robot_config  = self.create_client(RobotConfig,  SRV_ROBOT_CONFIG,  callback_group=self._cb_group_clients)

        self.get_logger().info('Waiting for services...')
        self.client_clean_error.wait_for_service()
        self.client_clean_warn.wait_for_service()
        self.client_motion_enable.wait_for_service()
        self.client_robot_config.wait_for_service()
        self.get_logger().info('Services ready.')

        # Publishers
        self.pub_error_collision = self.create_publisher(
            Error,
            PUB_TOPIC_ERROR_COLLISION,
            10,
            callback_group=self._cb_group_pubsub,
        )

        # Robot States subscription
        self.create_subscription(
            RobotMsg,
            SUB_TOPIC_ROBOT_STATES,
            self._robot_states_callback,
            10,
            callback_group=self._cb_group_pubsub,
        )
        self.get_logger().info('Monitoring Errors!')

    # Robot States callback
    def _robot_states_callback(self, msg: RobotMsg):

        # Normal monitoring
        self.get_logger().info(' ')
        if msg.err != 0: self._handle_error(msg.err)
        if msg.warn != 0: self._handle_warn(msg.warn)
        self._handle_state(msg.state)
        self._handle_mode(msg.mode)

    def _handle_error(self, err: int) -> None:
        error_codes: dict[int, str] = {
            1: "The Emergency Stop Button is pushed",
            2: "The Emergency IO of the Control Box is triggered",
            3: "The Emergency Stop Button of the Three-state Switch is pressed",
            10: "Servo motor error",
            11: "Servo motor 1 error",
            12: "Servo motor 2 error",
            13: "Servo motor 3 error",
            14: "Servo motor 4 error",
            15: "Servo motor 5 error",
            16: "Servo motor 6 error",
            17: "Servo motor 7 error",
            18: "Force Torque Sensor Communication Error",
            19: "End Module Communication Error",
            21: "Kinematic Error",
            22: "Self-Collision Error",
            23: "Joints Angle Exceed Limit",
            24: "Speed Exceeds Limit",
            25: "Planning Error",
            26: "Linux RT Error",
            27: "Command Reply Error",
            28: "End Module Communication Error",
            29: "Other Errors",
            30: "Feedback Speed Exceeds limit",
            31: "Collision Caused Abnormal Current",
            32: "Three-point drawing circle calculation error",
            33: "Controller GPIO error",
            34: "Recording Timeout",
            35: "Safety Boundary Limit",
            36: "The number of delay commands exceeds the limit",
            37: "Abnormal movement in Manual Mode",
            38: "Abnormal Joint Angle",
            39: "Abnormal Communication Between Master and Slave IC of Power Board",
            40: "No IK available",
            50: "Six-axis Force Torque Sensor read error",
            51: "Six-axis Force Torque Sensor set mode error",
            52: "Six-axis Force Torque Sensor set zero error",
            53: "Six-axis Force Torque Sensor is overloaded or the reading exceeds the limit",
            110: "Robot Arm Base Board Communication Error",
            111: "Control Box External 485 Device Communication Error",
        }
        self.get_logger().error(f'Robot error code: {error_codes[err]}')

        if err == 31:
            if not self.flag_error_detected:
                # Error message for state machine
                msg_error: Error  = Error()
                msg_error.error   = True
                msg_error.message = f'Robot error code: {error_codes[err]}'
                self.pub_error_collision.publish(msg_error)

                # Reconfigure robot
                req: RobotConfig.Request  = RobotConfig.Request()
                req.collision_sensitivity = -1 # Trigger default value
                req.state                 = -1 # Trigger default value
                req.mode                  = -1 # Trigger default value
                ret_config = self.client_robot_config.call(req)
                if not ret_config.success:
                    self.get_logger().error(f"{ret_config.message}")
                else: 
                    self.get_logger().info(f"{ret_config.message}")
                    self.flag_error_detected = True

    def _handle_warn(self, warn: int) -> None:
        warn_codes: dict[int, str] = {
            11: "uxbus queue is full",
            12: "parameter error",
            13: "the instruction does not exist",
            14: "command has no solution",
            15: "modbus cmd full",
        }

        self.get_logger().warn(f'Robot warning code: {warn_codes[warn]}')

    def _handle_state(self, state: int) -> None:
        # State: current robot state
        #   1: RUNNING        — actively executing a motion command
        #   2: SLEEPING       — idle, ready to accept commands
        #   3: PAUSED         — paused mid-motion (can be resumed)
        #   4: STOPPED        — not ready; requires re-enable before motion
        #   5: CONFIG_CHANGED — system configuration changed; not ready for motion

        states: dict[int, str] = {
            1: "RUNNING",
            2: "SLEEPING",
            3: "PAUSED",
            4: "STOPPED",
            5: "CONFIG_CHANGED",
        }
        self.get_logger().info(f'Robot state: {states[state]}')

        if (    states[state] == "SLEEPING"
            and self.flag_error_detected):
            msg_error: Error  = Error()
            msg_error.error   = False
            msg_error.message = f'Error cleared.'
            self.pub_error_collision.publish(msg_error)
            self.flag_error_detected = False

        if (    states[state] == "CONFIG_CHANGED"
            and self.flag_error_detected):
            # Reconfigure robot
            req: RobotConfig.Request  = RobotConfig.Request()
            req.collision_sensitivity = -1 # Trigger default value
            req.state                 = -1 # Trigger default value
            req.mode                  = -1 # Trigger default value
            self.client_robot_config.call(req)


    def _handle_mode(self, mode: int) -> None:
        # Mode: current control mode
        #   0: POSITION mode    — standard position control via controller box API
        #   1: SERVOJ mode      — immediate joint-space step execution (no buffering)
        #   2: TEACHING_JOINT   — gravity-compensated teaching mode (manual guidance)

        modes: dict[int, str] = {
            0: "POSITION",
            1: "SERVOJ",
            2: "TEACHING_JOINT",
        }
        self.get_logger().info(f'Robot mode: {modes[mode]}')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    try:
        # Spin node
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()