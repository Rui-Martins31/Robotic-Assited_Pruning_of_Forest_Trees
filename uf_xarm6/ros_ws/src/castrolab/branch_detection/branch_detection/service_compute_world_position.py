# Command to quickly test the service:
# ros2 service call /compute_world_position custom_interfaces/srv/YOLOPoint "{x_pixel: 100.0, y_pixel: 100.0, depth: 2.0}"

from . import _globals

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.spatial.transform import Rotation as R

from custom_interfaces.srv import YOLOPoint
from tf2_msgs.msg import TFMessage

# Constants
NODE_NAME: str = 'compute_world_position'

SRV_NAME: str  = 'compute_world_position'
SUB_TOPIC_NAME_TF: str = '/tf'

class ComputeWorldPosition(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        
        # Service
        self.service = self.create_service(
            YOLOPoint, 
            SRV_NAME,
            self.service_yolo_result_callback
        )

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def service_yolo_result_callback(self, request: YOLOPoint.Request, response: YOLOPoint.Response) -> YOLOPoint.Response:
        # Get current joint pose
        curr_joint_pose = self.get_current_joint_pose()

        # Compute coordinates
        point: np.ndarray = self.pixel_to_world_coord(
            x_pixel           = request.x_pixel,
            y_pixel           = request.y_pixel,
            depth             = request.depth,
            H_camera_to_world = curr_joint_pose
        ).flatten()

        response.x_world = float(point[0])
        response.y_world = float(point[1])
        response.z_world = float(point[2])

        return response

    # Utils
    def get_current_joint_pose(self):
        try:
            now = rclpy.time.Time()
            t   = self.tf_buffer.lookup_transform(
                'world',
                'camera_link',
                now
            )

            # Get rotation and position
            pos = t.transform.translation
            rot = t.transform.rotation
            self.get_logger().info(f'Joint is at: {pos.x}, {pos.y}, {pos.z}')

            return self.get_homogeneous_matrix(pos, rot)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

            return np.eye(4)

    def get_homogeneous_matrix(self, pos, rot):
        # Create the rotation matrix from the quaternion (x, y, z, w)
        quat = [rot.x, rot.y, rot.z, rot.w]
        rot_matrix = R.from_quat(quat).as_matrix()

        # Create a homogeneous matrix
        T           = np.eye(4)
        T[0:3, 0:3] = rot_matrix
        T[0:3, 3]   = [pos.x, pos.y, pos.z]

        return T

    def pixel_to_world_coord(
        self,
        x_pixel: float,
        y_pixel: float,
        depth: float,
        H_camera_to_world: np.ndarray = np.eye(4),

        fx: float = _globals.CAMERA_FX,
        fy: float = _globals.CAMERA_FY,
        cu: float = _globals.CAMERA_CU,
        cv: float = _globals.CAMERA_CV,
        s: float  = 0.0,
        camera_max_dist: float = _globals.CAMERA_MAX_DIST,
        camera_min_dist: float = _globals.CAMERA_MIN_DIST
    ) -> np.ndarray:

        # Check inputs
        if (x_pixel < 0.0): return np.zeros((3, 1))
        if (y_pixel < 0.0): return np.zeros((3, 1))
        if (depth < 0.0):   return np.zeros((3, 1))

        if (depth < camera_min_dist): depth = camera_min_dist
        if (depth > camera_max_dist): depth = camera_max_dist

        # Point in pixel frame
        point_pixel: np.ndarray = np.asarray(
            [[x_pixel],
            [y_pixel],
            [1]]
        )

        # Pixel frame to camera frame
        K_matrix: np.ndarray = np.asarray(
            [[fx, s,  cu],
            [0,  fy, cv],
            [0,  0,  1 ]]
        )

        # Point in camera optical frame
        point_camera_optical: np.ndarray = depth * (np.linalg.inv(K_matrix) @ point_pixel)

        # Camera optical frame to camera_link body frame
        R_optical_to_body: np.ndarray = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ], dtype=float)
        point_camera: np.ndarray = R_optical_to_body @ point_camera_optical

        # Camera frame to world frame
        point_camera_h: np.ndarray = np.vstack([point_camera, [[1.0]]])  # homogeneous

        # Point in world frame
        point_world_h:  np.ndarray = H_camera_to_world @ point_camera_h

        return point_world_h[0:3]


def main(args=None):
    rclpy.init(args=args)
    compute_world_position = ComputeWorldPosition()
    rclpy.spin(compute_world_position)
    compute_world_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()