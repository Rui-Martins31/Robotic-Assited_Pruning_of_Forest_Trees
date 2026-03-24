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


# Constants
NODE_NAME: str       = 'detect_branch'

SUB_TOPIC_NAME: str  = 'camera/image'
PUB_TOPIC_NAME: str  = 'yolo/position_vector'
TIMER_DELAY: float   = 1.0

PATH_SAVE_IMAGE: str = './output/image_detection/'

YOLO_MODEL_NAME: str = 'yolov8n.pt'

class CameraImageSubscriber(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            SUB_TOPIC_NAME,
            self.listener_callback,
            10
        )
        self.subscription # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            Point,
            PUB_TOPIC_NAME,
            10
        )
        self.publisher # prevent unused variable warning

        # Timer
        self.timer = self.create_timer(
            TIMER_DELAY,
            self.timer_callback
        )
        self.timer

        # Image
        os.makedirs(PATH_SAVE_IMAGE, exist_ok=True) # create dir
        self.image: Image | None = None
        self.image_count: int    = 0

        self.bridge: CvBridge = CvBridge()

        # YOLO
        model_path = os.path.join(get_package_share_directory('branch_detection'), 'models', YOLO_MODEL_NAME)
        self.yolo_model = YOLO(model_path)


    def listener_callback(self, msg: Image):
        # self.get_logger().info(f"Receiving image: {msg.data}")
        self.image = msg

    def timer_callback(self):
        if self.image is None:
            return
        
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
            
            # Image processing
            # Resize
            IMAGE_WIDTH: int  = 400
            IMAGE_HEIGHT: int = 300
            cv_image = cv2.resize(
                cv_image,
                (IMAGE_WIDTH, IMAGE_HEIGHT),
                interpolation=cv2.INTER_LINEAR
            )

            # Equalization
            img_yuv        = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
            img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
            cv_image       = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

            # YOLO inference
            results = self.yolo_model(
                cv_image,
                conf    = 0.5,
                verbose = False
            )

            # Default values
            cx: float = float(IMAGE_WIDTH/2)
            cy: float = float(IMAGE_HEIGHT/2)

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
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(cv_image, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                    # DEBUG
                    cv2.line(
                        cv_image,
                        (int(IMAGE_WIDTH/2), int(IMAGE_HEIGHT/2)),
                        (int(cx), int(cy)),
                        (255, 0, 0),
                        5
                    ) # Line between the bounding box and the center of the image

                    # Text label
                    label = f"{self.yolo_model.names[int(cls)]} {conf:.2f}"
                    cv2.putText(cv_image, label, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # DEBUG
                    break # Stop at first box

            # Save
            self.image_count += 1
            cv2.imwrite(f'{PATH_SAVE_IMAGE}img_{self.image_count}.png', cv_image)
            self.get_logger().info("Saved image.")

            # Publish
            pub_msg: Point = Point()
            pub_msg.x = float(cx - IMAGE_WIDTH/2)
            pub_msg.y = float(cy - IMAGE_HEIGHT/2)
            pub_msg.z = 0.0
            self.publisher.publish(pub_msg)
            self.get_logger().info(f"Publishing: ({pub_msg.x}, {pub_msg.y}, {pub_msg.z})\n")

        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}\n")
            return



def main(args=None):
    rclpy.init(args=args)

    camera_image_subscriber = CameraImageSubscriber()

    rclpy.spin(camera_image_subscriber)

    camera_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()