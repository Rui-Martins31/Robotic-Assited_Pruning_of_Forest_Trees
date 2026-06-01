import rclpy
from rclpy.node import Node

from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import SetInt16
from xarm_msgs.srv import SetInt16ById
from xarm_msgs.srv import Call
from std_msgs.msg import Empty

# Constants
NODE_NAME: str = 'monitor'

SUB_TOPIC_ROBOT_STATES:    str = '/xarm/robot_states'
PUB_TOPIC_ERROR_COLLISION: str = '/monitoring/collision_caused_abnormal_current'

SRV_SET_STATE:     str = '/xarm/set_state'
SRV_SET_MODE:      str = '/xarm/set_mode'
SRV_SET_COLLISION: str = '/xarm/set_collision_sensitivity'
SRV_CLEAN_ERROR:   str = '/xarm/clean_error'
SRV_CLEAN_WARN:    str = '/xarm/clean_warn'
SRV_MOTION_ENABLE: str = '/xarm/motion_enable'


class MonitorNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # X-arm services
        self.client_set_state     = self.create_client(SetInt16, SRV_SET_STATE)
        self.client_set_mode      = self.create_client(SetInt16, SRV_SET_MODE)
        self.client_set_collision = self.create_client(SetInt16, SRV_SET_COLLISION)
        self.client_clean_error   = self.create_client(Call, SRV_CLEAN_ERROR)
        self.client_clean_warn    = self.create_client(Call, SRV_CLEAN_WARN)
        self.client_motion_enable = self.create_client(SetInt16ById, SRV_MOTION_ENABLE)

        self.get_logger().info('Waiting for services...')
        self.client_set_state.wait_for_service()
        self.client_set_mode.wait_for_service()
        self.client_set_collision.wait_for_service()
        self.client_clean_error.wait_for_service()
        self.client_clean_warn.wait_for_service()
        self.client_motion_enable.wait_for_service()
        self.get_logger().info('Services ready.')

        # Publishers
        self.pub_error_collision = self.create_publisher(
            Empty,
            PUB_TOPIC_ERROR_COLLISION,
            10
        )

        # Robot States subscription
        self.create_subscription(
            RobotMsg,
            SUB_TOPIC_ROBOT_STATES,
            self._robot_states_callback,
            10,
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
            self.pub_error_collision.publish(Empty())

    def _handle_warn(self, warn: int) -> None:
        # Warn: warning code from the robot controller (0 = no warning)
        #   Non-zero values indicate a recoverable condition.
        #   Full warning code table: xarm SDK docs / xarm_api error_warn.md

        self.get_logger().warn(f'Robot warning code: {warn}')

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()