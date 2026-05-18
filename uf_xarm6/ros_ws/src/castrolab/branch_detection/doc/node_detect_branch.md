# node_detect_branch

**Node Name:** `detect_branch` \
**Executable:** `node_detect_branch` \
**Source:** [`branch_detection/node_detect_branch.py`](../branch_detection/node_detect_branch.py)

## Overview

This node is responsible for the branch and trunk detection. It subscribes to both the RGB and Depth image topics published by the camera. The RGB image is fed to a YOLO segmentation model (`yolo_syn_data_model.pt`, trained on synthetic data) that returns a polygon mask for each detected branch. The mask is converted to a binary image and then filtered using the depth map to discard pixels beyond `MAX_BRANCH_DEPTH`. The centroid of the filtered mask is used as the branch centre (with a nearest-pixel snap if the centroid falls outside the mask). A line is also fitted to the mask to capture the branch orientation. The node calls `service_compute_world_position` to convert the centre pixel and its depth to world coordinates, then publishes both the image-frame and world-frame positions.

---

## Configuration Constants

| Constant | Description |
|----------|-------------|
| `TIMER_DELAY` | Inference frequency |
| `YOLO_MODEL_NAME` | YOLO segmentation model file (currently trained on synthetic data) |
| `MAX_BRANCH_DEPTH` | Depth threshold (metres); mask pixels beyond this distance are discarded before centroid computation |
| `BOOL_SAVE_IMAGE` | Whether to write annotated images to disk |
| `PATH_SAVE_IMAGE` | Directory for saved images |

---

## Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image` | `sensor_msgs/Image` | RGB image, decoded as `bgr8` |
| `camera/depth_image` | `sensor_msgs/Image` | Depth image, decoded as `passthrough` (float32, metres) |

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

## Methods

### `fit_line_to_mask(binary_mask)`

Fits a line to the foreground pixels of a binary mask using `cv2.fitLine` (L2 distance).

| Parameter | Type | Description |
|-----------|------|-------------|
| `binary_mask` | `np.ndarray` | Single-channel uint8 mask (255 = foreground) |

Returns `(pt1, pt2)`, two `(int, int)` endpoints, in image frame, along the fitted line direction. Will later be used to find the cutting points.

---

## Service Client

| Service | Type |
|---------|------|
| `/compute_world_position` | `interfaces/srv/YOLOPoint` |

The node calls this service with the pixel centre `(cx, cy)` and the depth value at that pixel. Receives back the point coordinates in the world frame.