# branch_detection - Package Overview

## Overview

This package detects branches and trunks in the camera's 2D images and turns each detection into 3D target points the arm can be driven to. It offers two interchangeable detectors plus a shared coordinate-transformation service:

- **`node_detect_branch_by_color`** segments the branch by HSV color thresholding (the branch is painted a known blue in the current setup). It is the detector wired into the full system and the only one that produces the multi-point cutting buffer.
- **`node_detect_branch_yolo`** segments the branch with a custom YOLO model trained on synthetic data. It is single-point only (no buffer yet).
- **`compute_world_position`** is a service node that converts pixel and depth into world frame coordinates, used by both detectors.

---

## Launch

HSV color detector (used by the full system):

```bash
ros2 launch branch_detection branch_detection_by_color.launch.py
```

YOLO detector:

```bash
ros2 launch branch_detection branch_detection_yolo.launch.py
```

Each launch file starts the chosen detector node together with the `compute_world_position` service node.

---

## Nodes

| Node | Executable | Source | Doc |
|------|-----------|--------|-----|
| `detect_branch_by_color` | `node_detect_branch_by_color` | [`node_detect_branch_by_color.py`](../branch_detection/node_detect_branch_by_color.py) | [node_detect_branch_by_color.md](node_detect_branch_by_color.md) |
| `detect_branch` (YOLO) | `node_detect_branch_yolo` | [`node_detect_branch_yolo.py`](../branch_detection/node_detect_branch_yolo.py) | [node_detect_branch_yolo.md](node_detect_branch_yolo.md) |
| `compute_world_position` | `service_compute_world_coordinates` | [`service_compute_world_position.py`](../branch_detection/service_compute_world_position.py) | [service_compute_world_position.md](service_compute_world_position.md) |

---

## Communication

### Subscriptions

| Topic | Type | Used by |
|-------|------|---------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | color detector (RGB) |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | color detector (depth, 16-bit mm) |
| `camera/image` | `sensor_msgs/Image` | YOLO detector (RGB, relative) |
| `camera/depth_image` | `sensor_msgs/Image` | YOLO detector (depth, relative) |

### Publications

| Topic | Type | Used by |
|-------|------|---------|
| `/yolo/position_vector_image_frame` | `geometry_msgs/Point` | both (normalised centroid in image frame) |
| `/yolo/position_vector_world_frame` | `geometry_msgs/Point` | both (centroid in world frame) |
| `/yolo/buffer_positions` | `custom_interfaces/BufferPoints` | color detector (N cutting points) |
| `/yolo/image_debug` | `sensor_msgs/Image` | color detector (annotated visualisation) |

### Services

| Service | Type | Provided by | Called by |
|---------|------|-------------|-----------|
| `compute_world_position` | `custom_interfaces/srv/YOLOPoint` | `compute_world_position` node | both detectors |
| `compute_world_position_buffer` | `custom_interfaces/srv/BufferYOLOPoint` | `compute_world_position` node | color detector |

### TF2

| Source frame | Target frame | Used by |
|---|---|---|
| `link_base` | `link_eef` | `compute_world_position` (current camera/EEF pose) |