from . import _globals

import rclpy
from rclpy.node import Node

import numpy as np

from interfaces.srv import YOLOPoint


# Constants
NODE_NAME: str = 'compute_world_position'

SRV_NAME: str  = 'compute_world_position'

class ComputeWorldPosition(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.srv = self.create_service(
            YOLOPoint, 
            SRV_NAME,
            self.listener_yolo_result_callback
        )

    def listener_yolo_result_callback(self, request: YOLOPoint.Request, response: YOLOPoint.Response):
        point: np.ndarray = pixel_to_world_coord(
            x_pixel = request.x_pixel,
            y_pixel = request.y_pixel,
            depth   = request.depth
        ).flatten()

        response.x_world = float(point[0])
        response.y_world = float(point[1])
        response.z_world = float(point[2])

        return response


# Utils
def pixel_to_world_coord(
    x_pixel: float,
    y_pixel: float,
    depth: float,

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

    # Point in camera frame
    point_camera: np.ndarray = depth * (np.linalg.inv(K_matrix) @ point_pixel)

    return point_camera

    # Camera frame to world frame
    # TODO_


def main(args=None):
    rclpy.init(args=args)
    compute_world_position = ComputeWorldPosition()
    rclpy.spin(compute_world_position)
    compute_world_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()