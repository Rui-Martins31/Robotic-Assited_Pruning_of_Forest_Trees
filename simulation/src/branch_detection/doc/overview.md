# branch_detection - Package Overview

## Overview

This package is responsible for the detection of branches and trunks using 2D images. These images are published to a topic subscribed by the `detect_branch` node. It uses a custom YOLO segmentation model (trained on synthetic data) to detect branches/trunks and produce a polygon mask for each detection. The mask is depth-filtered to remove pixels beyond a maximum range, and its centroid is used as the target point. A line is also fitted to the mask to capture branch orientation. The pixel coordinates and depth are then converted to 3D world-frame coordinates by the `compute_world_position` service, allowing the arm to be directed towards the detected branch.

---

## Launch

```bash
ros2 launch branch_detection branch_detection.launch.py
```

This starts both nodes: `detect_branch` and `compute_world_position`.

---

## Nodes

| Node | Executable | Path |
|------|-----------|--------|
| `detect_branch` | `node_detect_branch` | `branch_detection/node_detect_branch.py` |
| `compute_world_position` | `service_compute_world_coordinates` | `branch_detection/service_compute_world_position.py` |

---

## Communication

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image` | `sensor_msgs/Image` | RGB image |
| `camera/depth_image` | `sensor_msgs/Image` | Depth image |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `yolo/position_vector_image_frame` | `geometry_msgs/Point` | Normalised branch position in image frame |
| `yolo/position_vector_world_frame` | `geometry_msgs/Point` | Branch position in world frame (metres) |

### Services

| Service | Type |
|---------|------|
| `/compute_world_position` | `interfaces/srv/YOLOPoint` |

### TF2

| Source frame | Target frame | Used by |
|---|---|---|
| `world` | `camera_link` | `compute_world_position` |

---

## Dependencies

### ROS2 / System Packages

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS2 Python client library |
| `sensor_msgs` | `Image` message type |
| `geometry_msgs` | `Point` message type |
| `cv_bridge` | OpenCV <-> ROS image conversion |
| `tf2_ros` | TF2 transform lookup |
| `python3-opencv` | Image processing |
| `python3-numpy` | Numerical computations |
| `scipy` | Quaternion -> rotation matrix conversion |

### Third-Party

| Package | Purpose |
|---------|---------|
| `ultralytics` | YOLO model inference |

### Internal

| Package | Purpose |
|---------|---------|
| `interfaces` | Defines the `YOLOPoint` service type |
