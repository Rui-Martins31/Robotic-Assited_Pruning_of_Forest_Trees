import pyrealsense2 as rs
import numpy as np
import cv2

# Constant
BLUE_LOWER = np.array([100, 80, 50])
BLUE_UPPER = np.array([130, 255, 255])

# Exposure
EXPOSURE_US = 100

# Camera setup
pipe   = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipe.start(config)

# Limit exposure
# Disable auto-exposure
color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, 0)
color_sensor.set_option(rs.option.exposure, EXPOSURE_US)

# Local contrast enhancement
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# Discard first frames
# while the sensor stabilises
for _ in range(30):
    pipe.wait_for_frames()

cv2.startWindowThread()
cv2.namedWindow("Color | Mask | Masked", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        frames = pipe.wait_for_frames(timeout_ms=5000)

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        bgr = np.asanyarray(color_frame.get_data())

        # Equalize brightness
        hsv      = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v  = cv2.split(hsv)
        hsv      = cv2.merge([h, s, clahe.apply(v)])
        bgr      = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # Color mask
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

        # Clean up small noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        masked = cv2.bitwise_and(bgr, bgr, mask=mask)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((bgr, mask_bgr, masked))

        cv2.imshow("Color | Mask | Masked", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
