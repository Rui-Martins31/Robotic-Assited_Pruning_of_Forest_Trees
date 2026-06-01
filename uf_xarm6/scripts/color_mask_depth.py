import pyrealsense2 as rs
import numpy as np
import cv2

# Constants
BLUE_LOWER = np.array([100, 80, 50])
BLUE_UPPER = np.array([130, 255, 255])

EXPOSURE_US = 100

# Camera setup
pipe   = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,   30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,    30)

profile = pipe.start(config)

# Align depth to colour frame
align = rs.align(rs.stream.color)

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
cv2.namedWindow("Color | Mask | Masked | Depth", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        frames = pipe.wait_for_frames(timeout_ms=5000)
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        bgr   = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())   # uint16, mm

        # Equalize brightness
        hsv     = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        hsv     = cv2.merge([h, s, clahe.apply(v)])
        bgr     = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # Colour mask
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

        # Clean up small noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Centroid and depth
        cx, cy, depth_m = None, None, None
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            depth_mm = depth_frame.get_distance(cx, cy) * 1000 # m to mm
            depth_m  = depth_frame.get_distance(cx, cy)        # m

            # Draw centroid
            cv2.circle(bgr, (cx, cy), 6, (0, 255, 0), -1)
            cv2.circle(bgr, (cx, cy), 8, (0, 0, 0),    2)
            label = f"({cx}, {cy})  {depth_m:.3f} m"
            cv2.putText(bgr, label, (cx + 12, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            print(f"Centroid: ({cx}, {cy})  depth: {depth_m:.4f} m  ({depth_mm:.1f} mm)")

        masked   = cv2.bitwise_and(bgr, bgr, mask=mask)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Colorize depth map
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        if cx is not None:
            cv2.circle(depth_vis, (cx, cy), 6, (255, 255, 255), -1)
            cv2.circle(depth_vis, (cx, cy), 8, (0,   0,   0),    2)

        # Display windows
        combined = np.vstack((
            np.hstack((bgr,    mask_bgr)),
            np.hstack((masked, depth_vis)),
        ))

        cv2.imshow("Color | Mask | Masked | Depth", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
