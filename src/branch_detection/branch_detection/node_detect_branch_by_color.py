import os

import cv2
from cv_bridge import CvBridge

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from custom_interfaces.srv import YOLOPoint

from . import _globals


### TODO_
## Later use the fitted line to find the cutting points.


# Constants
NODE_NAME:       str   = 'detect_branch_by_color'

SUB_TOPIC_NAME_IMAGE_RGB:       str = 'camera/image'
SUB_TOPIC_NAME_IMAGE_DEPTH:     str = 'camera/depth_image'
PUB_TOPIC_NAME_POS_IMAGE_FRAME: str = 'yolo/position_vector_image_frame'
PUB_TOPIC_NAME_POS_WORLD_FRAME: str = 'yolo/position_vector_world_frame'
TIMER_DELAY:     float = 0.05

PATH_SAVE_IMAGE: str   = './output/image_detection/'
BOOL_SAVE_IMAGE: bool  = True

COLOR_BRANCH_LOWER: np.ndarray = np.array([20,  20,  20])
COLOR_BRANCH_UPPER: np.ndarray = np.array([110, 116, 109])

MAX_BRANCH_DEPTH: float = 10.0  # meters

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
        self.client_compute_world_position = self.create_client(YOLOPoint, 'compute_world_position')

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

            # Get branch mask by color (BGR thresholds)
            mask_branch = cv2.inRange(
                cv_image_rgb,
                COLOR_BRANCH_LOWER,
                COLOR_BRANCH_UPPER
            )

            # Depth map masks color mask
            # Avoids pixels with huge depth values
            depth_valid   = (
                np.isfinite(cv_image_depth) &
                (cv_image_depth > 0) &
                (cv_image_depth < MAX_BRANCH_DEPTH)
            ).astype(np.uint8) * 255
            combined_mask = cv2.bitwise_and(mask_branch, depth_valid)
            if cv2.countNonZero(combined_mask) > 0:
                mask_branch = combined_mask

            ## DEBUG
            # Draw mask
            overlay = cv_image_rgb.copy()
            overlay[mask_branch > 0] = (0, 255, 0)
            cv2.addWeighted(overlay, 0.4, cv_image_rgb, 0.6, 0, cv_image_rgb)
            contours, _ = cv2.findContours(mask_branch, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(cv_image_rgb, contours, -1, (0, 255, 0), 2)

            # Default values
            cx: float      = float(_globals.IMAGE_WIDTH / 2)
            cy: float      = float(_globals.IMAGE_HEIGHT / 2)
            detected: bool = False

            # Mask's centroid
            M = cv2.moments(mask_branch)
            if M['m00'] != 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']

                # Snap centroid to nearest mask pixel if it falls outside
                if mask_branch[int(cy), int(cx)] == 0:
                    yx      = np.argwhere(mask_branch > 0)
                    dists   = np.sum((yx - np.array([cy, cx])) ** 2, axis=1)
                    nearest = yx[np.argmin(dists)]
                    cy, cx  = float(nearest[0]), float(nearest[1])

                detected = True

                # Fit line to mask
                line_pt1, line_pt2 = self.fit_line_to_mask(mask_branch)
                cv2.line(cv_image_rgb, line_pt1, line_pt2, (0, 165, 255), 2)

                ## DEBUG
                self.get_logger().info(f"Centroid: ({cx:.1f}, {cy:.1f})")
                self.get_logger().info(f"Depth: {float(cv_image_depth[int(cy), int(cx)]):.3f}")

                ## DEBUG
                # Center point and line to image center
                cv2.circle(cv_image_rgb, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.line(
                    cv_image_rgb,
                    (int(_globals.IMAGE_WIDTH/2), int(_globals.IMAGE_HEIGHT/2)),
                    (int(cx), int(cy)),
                    (255, 0, 0),
                    5
                )

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
                request = YOLOPoint.Request()
                request.x_pixel = float(cx)
                request.y_pixel = float(cy)
                request.depth   = float(cv_image_depth[int(cy), int(cx)])

                future = self.client_compute_world_position.call_async(request)
                future.add_done_callback(self._world_position_callback)

        except Exception as e:
            self.get_logger().error(f"Error: {e}\n")
            return

    def _world_position_callback(self, future: rclpy.task.Future) -> None:
        try:
            result: YOLOPoint.Response = future.result()
            pub_msg   = Point()
            pub_msg.x = result.x_world
            pub_msg.y = result.y_world
            pub_msg.z = result.z_world
            
            self.publisher_world_frame.publish(pub_msg)
            self.get_logger().info(f"Publishing (world_frame): ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})\n")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def fit_line_to_mask(
            self,
            mask_branch: np.ndarray,
        ) -> tuple[tuple[int,int], tuple[int,int]]:

        # Scatter points across the mask
        yx     = np.argwhere(mask_branch > 0)
        points = yx[:, ::-1].reshape(-1, 1, 2).astype(np.float32)

        # Fit line to points
        vx, vy, x0, y0 = cv2.fitLine(
            points,
            cv2.DIST_L2,
            0,
            0.01,
            0.01
        ).flatten()

        scale: int           = max(mask_branch.shape[:2])
        pt1: tuple[int, int] = (int(x0 - vx * scale), int(y0 - vy * scale))
        pt2: tuple[int, int] = (int(x0 + vx * scale), int(y0 + vy * scale))

        return pt1, pt2


def main(args=None):
    rclpy.init(args=args)

    camera_image_subscriber = CameraImageSubscriber()

    rclpy.spin(camera_image_subscriber)

    camera_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()