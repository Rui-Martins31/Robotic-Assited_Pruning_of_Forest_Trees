# node_detect_branch

**Node Name:** `detect_branch` \
**Executable:** `node_detect_branch` \
**Source:** [`branch_detection/node_detect_branch.py`](../branch_detection/node_detect_branch.py)

## Overview

This node is responsible for the branch and trunk detection. It subscribes to both the RGB and Depth image topics that publish the images captured by the camera. The RGB image will be used as the input to the YOLO model that detects and creates a bounding box that surrounds the branch or trunk. It then uses the service `service_compute_world_position` to convert the center of the bounding box to world coordinates. Finally, publishes the point coordinates in both iamge and world frames to two separate topics.

---

## Configuration Constants

| Constant | Description |
|----------|-------------|
| `TIMER_DELAY` | Inference frequency |
| `YOLO_MODEL_NAME` | YOLO model file |
| `BOOL_SAVE_IMAGE` | Whether to write annotated images to disk |
| `PATH_SAVE_IMAGE` | Directory for saved images |

---

## Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image` | `sensor_msgs/Image` | RGB image, decoded as `bgr8` |
| `camera/depth_image` | `sensor_msgs/Image` | Depth image, decoded as `bgr32` |

## Publications

### `yolo/position_vector_image_frame` (`geometry_msgs/Point`)

Normalised position of the detected branch centre relative to the image centre.

| Field | Value |
|-------|-------|
| `x` | `(cx - IMAGE_WIDTH/2) / IMAGE_WIDTH` |
| `y` | `(cy - IMAGE_HEIGHT/2) / IMAGE_HEIGHT` |
| `z` | `0.0` |

A value of `(0, 0, 0)` means the branch is at the exact centre of the image.

### `yolo/position_vector_world_frame` (`geometry_msgs/Point`)

3D branch position in the world frame (metres).

| Field | Value |
|-------|-------|
| `x` | World X coordinate (m) |
| `y` | World Y coordinate (m) |
| `z` | World Z coordinate (m) |

---

## Service Client

| Service | Type |
|---------|------|
| `/compute_world_position` | `interfaces/srv/YOLOPoint` |

The node calls this service with the pixel centre `(cx, cy)` and the depth value at that pixel. Receives back the point coordinates in the world frame.