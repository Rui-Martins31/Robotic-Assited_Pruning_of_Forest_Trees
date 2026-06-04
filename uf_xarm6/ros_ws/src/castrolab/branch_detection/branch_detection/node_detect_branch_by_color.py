import os

import cv2
from cv_bridge import CvBridge

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from custom_interfaces.msg import BufferPoints

from custom_interfaces.srv import YOLOPoint, BufferYOLOPoint

from . import _globals


### TODO_
## Later use the fitted line to find the cutting points.


# Constants
NODE_NAME:       str   = 'detect_branch_by_color'

SUB_TOPIC_NAME_IMAGE_RGB:       str = '/camera/color/image_raw'
# SUB_TOPIC_NAME_IMAGE_DEPTH:     str = '/camera/aligned_depth_to_color/image_raw'
SUB_TOPIC_NAME_IMAGE_DEPTH:     str = '/camera/depth/image_rect_raw'
PUB_TOPIC_NAME_POS_IMAGE_FRAME: str = '/yolo/position_vector_image_frame'
PUB_TOPIC_NAME_POS_WORLD_FRAME: str = '/yolo/position_vector_world_frame'
PUB_TOPIC_NAME_POS_BUFFER:      str = '/yolo/buffer_positions'
PUB_TOPIC_NAME_IMAGE_DEBUG:     str = '/yolo/image_debug'

SRV_NAME_POINT:    str = 'compute_world_position'
SRV_NAME_BUFFER:   str = 'compute_world_position_buffer'

TIMER_DELAY:     float = 0.05

PATH_SAVE_IMAGE: str   = './output/image_detection/'
BOOL_SAVE_IMAGE: bool  = False

# Blue HSV
COLOR_BRANCH_LOWER = np.array([80, 90, 70])
COLOR_BRANCH_UPPER = np.array([130, 255, 255])

MAX_BRANCH_DEPTH: float = 10.0  # meters

POINT_PER_BRANCH: int   = 5

class CameraImageSubscriber(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # Subscriber
        self.subscription_image_rgb = self.create_subscription(
            Image,
            SUB_TOPIC_NAME_IMAGE_RGB,
            self.listener_image_rgb_callback,
            qos_profile_sensor_data
        )
        self.subscription_image_depth = self.create_subscription(
            Image,
            SUB_TOPIC_NAME_IMAGE_DEPTH,
            self.listener_image_depth_callback,
            qos_profile_sensor_data
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
        self.publisher_image_debug = self.create_publisher(
            Image,
            PUB_TOPIC_NAME_IMAGE_DEBUG,
            10
        )
        self.publisher_buffer = self.create_publisher(
            BufferPoints,
            PUB_TOPIC_NAME_POS_BUFFER,
            10
        )
        self.publisher_image_frame # prevent unused variable warning
        self.publisher_world_frame
        self.publisher_image_debug
        self.publisher_buffer

        # Service
        self.client_compute_world_position        = self.create_client(YOLOPoint, SRV_NAME_POINT)
        self.client_compute_world_position_buffer = self.create_client(BufferYOLOPoint, SRV_NAME_BUFFER)

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
        # self.get_logger().info(f"Receiving image RGB...")
        self.image_rgb = msg

    def listener_image_depth_callback(self, msg: Image):
        # self.get_logger().info(f"Receiving image Depth...")
        self.image_depth = msg

    def timer_callback(self):
        # self.get_logger().info(f"Timer...")
        if (self.image_rgb is None) or (self.image_depth is None):
            return

        try:
            # Convert to OpenCV
            cv_image_rgb   = self.bridge.imgmsg_to_cv2(self.image_rgb, desired_encoding='bgr8')
            cv_image_depth = self.bridge.imgmsg_to_cv2(self.image_depth, desired_encoding='16UC1')
            cv_image_depth = cv_image_depth.astype(np.float32) / 1000.0
            
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

            # Get branch mask by color
            cv_image_hsv = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
            mask_branch  = cv2.inRange(
                cv_image_hsv,
                COLOR_BRANCH_LOWER,
                COLOR_BRANCH_UPPER
            )

            # Depth map masks color mask
            # Avoids pixels with huge depth values
            depth_valid   = (
                np.isfinite(cv_image_depth) &
                (cv_image_depth > 0.001) &
                (cv_image_depth < MAX_BRANCH_DEPTH)
            ).astype(np.uint8) * 255
            combined_mask = cv2.bitwise_and(mask_branch, depth_valid)
            if cv2.countNonZero(combined_mask) > 0:
                mask_branch = combined_mask

            # Keep only the largest patch in the mask
            mask_branch = self.mask_filter_largest_patch(mask_branch)

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
                # cv2.line(cv_image_rgb, line_pt1, line_pt2, (0, 165, 255), 2)

                # Scatter points along the line
                line_mask = np.zeros_like(cv_image_rgb)
                cv2.line(
                    line_mask,
                    line_pt1, line_pt2,
                    (0, 165, 255), 2
                )
                line_mask      = cv2.bitwise_and(line_mask, line_mask, mask = mask_branch)
                line_points    = self.line_extract_n_point(
                    mask       = cv2.cvtColor(line_mask, cv2.COLOR_BGR2GRAY),
                    num_points = POINT_PER_BRANCH
                )
    
                ## DEBUG
                cv_image_rgb = cv2.add(cv_image_rgb, line_mask)
                for (px, py) in line_points:
                    cv2.circle(cv_image_rgb, (px, py), 5, (0, 255, 255), -1)

                ## DEBUG
                # self.get_logger().info(f"Centroid: ({cx:.1f}, {cy:.1f})")
                # self.get_logger().info(f"Depth: {float(cv_image_depth[int(cy), int(cx)]):.3f}")

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
                cv2.putText(
                    img       = cv_image_rgb,
                    text      = "Branch",
                    org       = (int(cx), int(cy - 20.0)),
                    fontFace  = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 1,
                    color     = (0, 255,0)
                )

            # Save
            if BOOL_SAVE_IMAGE:
                self.image_count += 1
                cv2.imwrite(f'{PATH_SAVE_IMAGE}img_{self.image_count}.png', cv_image_rgb)
                self.get_logger().info("Saved image.")

            # Publish debug image
            self.publisher_image_debug.publish(self.bridge.cv2_to_imgmsg(cv_image_rgb, encoding='bgr8'))
            # self.publisher_image_debug.publish(self.bridge.cv2_to_imgmsg(mask_branch, encoding='mono8'))

            # Publish
            if detected:
                # Image frame
                pub_msg: Point = Point()
                pub_msg.x = float((cx - _globals.IMAGE_WIDTH/2)/_globals.IMAGE_WIDTH)
                pub_msg.y = float((cy - _globals.IMAGE_HEIGHT/2)/_globals.IMAGE_HEIGHT)
                pub_msg.z = 0.0
                self.publisher_image_frame.publish(pub_msg)
                # self.get_logger().info(f"Publishing (image_frame): ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})")

                # World frame
                request = YOLOPoint.Request()
                request.x_pixel = float(cx)
                request.y_pixel = float(cy)
                request.depth   = float(cv_image_depth[int(cy), int(cx)])

                future = self.client_compute_world_position.call_async(request)
                future.add_done_callback(self._world_position_callback)

                # Buffer
                valid_x_pixels: list = []
                valid_y_pixels: list = []
                valid_depths:   list = []

                for (px, py) in line_points:
                    px_c = int(np.clip(px, 0, _globals.IMAGE_WIDTH  - 1))
                    py_c = int(np.clip(py, 0, _globals.IMAGE_HEIGHT - 1))
                    d    = float(cv_image_depth[py_c, px_c])
                    # Check depth value
                    if not np.isfinite(d) or d <= 0.001 or d >= MAX_BRANCH_DEPTH:
                        continue

                    valid_x_pixels.append(float(px_c))
                    valid_y_pixels.append(float(py_c))
                    valid_depths.append(d)

                if valid_x_pixels:
                    buf_req          = BufferYOLOPoint.Request()
                    buf_req.x_pixels = valid_x_pixels
                    buf_req.y_pixels = valid_y_pixels
                    buf_req.depths   = valid_depths
                    buf_future       = self.client_compute_world_position_buffer.call_async(buf_req)
                    buf_future.add_done_callback(self._buffer_position_callback)

        except Exception as e:
            self.get_logger().error(f"Error: {e}\n")
            return

    def _world_position_callback(
        self,
        future: rclpy.task.Future
    ) -> None:
        
        try:
            result: YOLOPoint.Response = future.result()
            pub_msg   = Point()
            pub_msg.x = result.x_world
            pub_msg.y = result.y_world
            pub_msg.z = result.z_world
            
            self.publisher_world_frame.publish(pub_msg)
            # self.get_logger().info(f"Publishing (world_frame): ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})\n")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def _buffer_position_callback(
        self,
        future: rclpy.task.Future
    ) -> None:
        try:
            result: BufferYOLOPoint.Response = future.result()
            pub_msg        = BufferPoints()
            pub_msg.size   = result.size
            pub_msg.points = list(result.points)
            self.publisher_buffer.publish(pub_msg)
            # self.get_logger().info(f"Publishing (buffer): {pub_msg.size} points: {pub_msg.points}")

        except Exception as e:
            self.get_logger().error(f"Buffer service call failed: {e}")

    def fit_line_to_mask(
        self,
        mask_branch: np.ndarray,
    # ) -> tuple[tuple[int,int], tuple[int,int]]:
    ):

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
    
    def line_extract_n_point(
        self,
        mask: np.ndarray,
        num_points: int
    ) -> np.ndarray:
        # Check mask's values
        pixels = cv2.findNonZero(mask)
        if pixels is None:
            return []
        
        pts = pixels.reshape(-1, 2)
        
        # Extremas
        pt1 = pts[np.argmin(pts[:, 0])]
        pt2 = pts[np.argmax(pts[:, 0])]
        
        # Get N points
        x_vals = np.linspace(pt1[0], pt2[0], num_points)
        y_vals = np.linspace(pt1[1], pt2[1], num_points)
        
        # Coordinate pairs
        # round to nearest pixel integer
        sampled_points = np.stack((x_vals, y_vals), axis=-1).astype(int)
        return sampled_points
    
    def mask_filter_largest_patch(
        self,
        mask: np.ndarray
    ) -> np.ndarray:
        # Connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        
        # No patches found
        if num_labels <= 1:
            return np.zeros_like(mask)
        
        # Area
        areas              = stats[:, cv2.CC_STAT_AREA]
        largest_label_id   = np.argmax(areas[1:]) + 1
        
        # New mask from patch
        largest_patch_mask = np.zeros_like(mask)
        largest_patch_mask[labels == largest_label_id] = 255
        
        return largest_patch_mask


def main(args=None):
    rclpy.init(args=args)

    camera_image_subscriber = CameraImageSubscriber()

    rclpy.spin(camera_image_subscriber)

    camera_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()