import pyrealsense2 as rs
import numpy as np
import cv2

# Configure streams
pipe   = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipe.start(config)

colorizer = rs.colorizer()

for _ in range(30):
    pipe.wait_for_frames()

cv2.startWindowThread()
cv2.namedWindow("RealSense - Color | Depth", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        frames = pipe.wait_for_frames(timeout_ms=5000)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        combined = np.hstack((color_image, depth_colormap))

        cv2.imshow("RealSense - Color | Depth", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
