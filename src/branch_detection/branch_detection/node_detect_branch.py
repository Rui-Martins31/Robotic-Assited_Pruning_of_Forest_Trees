import os

import cv2
from cv_bridge import CvBridge

import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from . import _globals


# Constants
NODE_NAME: str        = 'detect_branch'

SUB_TOPIC_NAME_IMAGE_RGB: str       = 'camera/image'
SUB_TOPIC_NAME_IMAGE_DEPTH: str     = 'camera/depth_image'
PUB_TOPIC_NAME_POS_IMAGE_FRAME: str = 'yolo/position_vector_image_frame'
PUB_TOPIC_NAME_POS_WORLD_FRAME: str = 'yolo/position_vector_world_frame'
TIMER_DELAY: float    = 0.05# 1.0

PATH_SAVE_IMAGE: str  = './output/image_detection/'
BOOL_SAVE_IMAGE: bool = True

YOLO_MODEL_NAME: str  = 'yolo_tree_detection.pt' #'yolov8n.pt'

class CameraImageSubscriber(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # Subscriber
        self.subscription_image_rgb = self.create_subscription(
            Image,
            SUB_TOPIC_NAME_IMAGE_RGB,
            self.listener_image_rgb_callback,
            10
        )
        self.subscription_image_depth = self.create_subscription(
            Image,
            SUB_TOPIC_NAME_IMAGE_DEPTH,
            self.listener_image_depth_callback,
            10
        )
        self.subscription_image_rgb # prevent unused variable warning
        self.subscription_image_depth

        # Publisher
        self.publisher_image_frame = self.create_publisher(
            Point,
            PUB_TOPIC_NAME_POS_IMAGE_FRAME,
            10
        )
        self.publisher_world_frame = self.create_publisher(
            Point,
            PUB_TOPIC_NAME_POS_WORLD_FRAME,
            10
        )
        self.publisher_image_frame # prevent unused variable warning
        self.publisher_world_frame

        # Service
        

        # Timer
        self.timer = self.create_timer(
            TIMER_DELAY,
            self.timer_callback
        )
        self.timer

        # Image
        os.makedirs(PATH_SAVE_IMAGE, exist_ok=True) # create dir
        self.image_rgb: Image | None   = None
        self.image_depth: Image | None = None
        self.image_count: int          = 0

        self.bridge: CvBridge = CvBridge()

        # YOLO
        model_path: str       = os.path.join(get_package_share_directory('branch_detection'), 'models', YOLO_MODEL_NAME)
        self.yolo_model: YOLO = YOLO(model_path)


    def listener_image_rgb_callback(self, msg: Image):
        # self.get_logger().info(f"Receiving image: {msg.data}")
        self.image_rgb = msg

    def listener_image_depth_callback(self, msg: Image):
        self.image_depth = msg

    def timer_callback(self):
        if (self.image_rgb is None) or (self.image_depth is None):
            return
        
        try:
            # Convert to OpenCV
            cv_image_rgb   = self.bridge.imgmsg_to_cv2(self.image_rgb, desired_encoding='bgr8')
            cv_image_depth = self.bridge.imgmsg_to_cv2(self.image_depth, desired_encoding='passthrough')
            
            # Image processing
            # Resize
            cv_image_rgb = cv2.resize(
                cv_image_rgb,
                (_globals.IMAGE_WIDTH, _globals.IMAGE_HEIGHT),
                interpolation=cv2.INTER_LINEAR
            )
            cv_image_depth = cv2.resize(
                cv_image_depth,
                (_globals.IMAGE_WIDTH, _globals.IMAGE_HEIGHT),
                interpolation=cv2.INTER_LINEAR
            )

            # Equalization
            # img_yuv        = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2YUV)
            # img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
            # cv_image_rgb       = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

            # YOLO inference
            results = self.yolo_model(
                cv_image_rgb,
                conf    = 0.5,
                verbose = False
            )

            # Default values
            cx: float      = float(_globals.IMAGE_WIDTH/2)
            cy: float      = float(_globals.IMAGE_HEIGHT/2)
            detected: bool = False

            # Iterate over detections
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get coordinates, confidence and class
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy, w, h   = box.xywh[0]
                    conf           = box.conf[0]
                    cls            = box.cls[0]
                    self.get_logger().info(f"Detected class {cls} with {conf:.2f} confidence.")
                    self.get_logger().info(f"Center of bounding box: ({cx},{cy})")

                    # Draw bounding box and center point
                    cv2.rectangle(cv_image_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(cv_image_rgb, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                    # DEBUG
                    cv2.line(
                        cv_image_rgb,
                        (int(_globals.IMAGE_WIDTH/2), int(_globals.IMAGE_HEIGHT/2)),
                        (int(cx), int(cy)),
                        (255, 0, 0),
                        5
                    ) # Line between the bounding box and the center of the image

                    # Text label
                    label = f"{self.yolo_model.names[int(cls)]} {conf:.2f}"
                    cv2.putText(cv_image_rgb, label, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # DEBUG
                    detected = True
                    break # Stop at first box

            # Save
            if BOOL_SAVE_IMAGE:
                self.image_count += 1
                cv2.imwrite(f'{PATH_SAVE_IMAGE}img_{self.image_count}.png', cv_image_rgb)
                self.get_logger().info("Saved image.")

            # Publish
            if detected:
                # Image frame
                pub_msg: Point = Point()
                pub_msg.x = float((cx - _globals.IMAGE_WIDTH/2)/_globals.IMAGE_WIDTH)
                pub_msg.y = float((cy - _globals.IMAGE_HEIGHT/2)/_globals.IMAGE_HEIGHT)
                pub_msg.z = 0.0
                self.publisher_image_frame.publish(pub_msg)
                self.get_logger().info(f"Publishing (image_frame): ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})")

                # World frame
                # pub_msg: Point = Point()
                world_coords: np.ndarray = pixel_to_world_coord(
                    x_pixel = cx,
                    y_pixel = cy,
                    depth   = cv_image_depth[int(cy), int(cx)]
                )
                pub_msg.x = float(world_coords[0][0])
                pub_msg.y = float(world_coords[1][0])
                pub_msg.z = float(world_coords[2][0])
                self.publisher_world_frame.publish(pub_msg)
                self.get_logger().info(f"Publishing (world_frame): ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})\n")

        except Exception as e:
            self.get_logger().error(f"Error: {e}\n")
            return

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
    if (x_pixel < 0.0): return np.asarray([0.0, 0.0, 0.0])
    if (y_pixel < 0.0): return np.asarray([0.0, 0.0, 0.0])
    if (depth < 0.0):   return np.asarray([0.0, 0.0, 0.0])

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

    camera_image_subscriber = CameraImageSubscriber()

    rclpy.spin(camera_image_subscriber)

    camera_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()