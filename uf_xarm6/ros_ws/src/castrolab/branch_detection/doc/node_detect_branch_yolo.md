# node_detect_branch_yolo

**Node Name (logger):** `detect_branch` \
**Executable:** `node_detect_branch_yolo` \
**Source:** [`branch_detection/node_detect_branch_yolo.py`](../branch_detection/node_detect_branch_yolo.py)

## Overview

The branch detector based on YOLO. It feeds the RGB image to a custom YOLO segmentation model and uses the returned polygon mask of the first detection as the branch. The mask is depth-filtered, its centroid is taken as the branch centre (with a nearest-pixel snap), a line is fitted for orientation and the centre pixel and depth are converted to a world frame point.

Compared with the color detector, this node is single-point only. It processes only the first detection, does not sample points along the line and does not publish a buffer.

---

## Configuration constants

| Constant | Value | Description |
|----------|-------|-------------|
| `TIMER_DELAY` | `0.05` | Processing period (s) |
| `YOLO_MODEL_NAME` | `yolo_syn_data_model.pt` | Segmentation model file |
| `MAX_BRANCH_DEPTH` | `10.0` | Depth cap (meters); mask pixels beyond this are discarded |
| `BOOL_SAVE_IMAGE` | `True` | Whether to write annotated frames to `PATH_SAVE_IMAGE` |
| `PATH_SAVE_IMAGE` | `./output/image_detection/` | Output directory for saved frames |

---

## Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image` | `sensor_msgs/Image` | RGB image, decoded `bgr8` (relative, remap to the camera) |
| `camera/depth_image` | `sensor_msgs/Image` | Depth image, decoded `passthrough` (relative) |

## Publications

| Topic | Type | Description |
|-------|------|-------------|
| `yolo/position_vector_image_frame` | `geometry_msgs/Point` | Centroid normalised to the image centre |
| `yolo/position_vector_world_frame` | `geometry_msgs/Point` | Centroid in the robot base frame (meters) |

## Services

| Service | Type | Purpose |
|---------|------|---------|
| `compute_world_position` | `custom_interfaces/srv/YOLOPoint` | World coordinates of the detected centre |