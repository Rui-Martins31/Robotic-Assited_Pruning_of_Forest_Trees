import os

import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


# Constants
TOPIC_NAME: str    = 'camera/image'
TIMER_DELAY: float = 1.0

PATH_SAVE_IMAGE: str = './output/image_detection/'

class CameraImageSubscriber(Node):

    def __init__(self):
        super().__init__('camera_image_subscriber')

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            TOPIC_NAME,
            self.listener_callback,
            10
        )
        self.subscription # prevent unused variable warning

        # Timer
        self.timer = self.create_timer(
            TIMER_DELAY,
            self.timer_callback
        )
        self.timer

        # Image
        os.makedirs(PATH_SAVE_IMAGE, exist_ok=True) # create dir
        self.image: Image     = Image()
        self.image_count: int = 0

        self.bridge: CvBridge = CvBridge()

    def listener_callback(self, msg: Image):
        # self.get_logger().info(f"Receiving image: {msg.data}")
        self.image = msg

    def timer_callback(self):
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

            # Save
            self.image_count += 1
            cv2.imwrite(f'{PATH_SAVE_IMAGE}img_{self.image_count}.png', cv_image)

            self.get_logger().info("Saved image.")

        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return



def main(args=None):
    rclpy.init(args=args)

    camera_image_subscriber = CameraImageSubscriber()

    rclpy.spin(camera_image_subscriber)

    camera_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()