# node_detect_branch_by_color

**Node Name (logger):** `detect_branch_by_color` \
**Executable:** `node_detect_branch_by_color` \
**Source:** [`branch_detection/node_detect_branch_by_color.py`](../branch_detection/node_detect_branch_by_color.py)

## Overview

The color branch detector and the detector used by the full system. It subscribes to the RealSense RGB and depth streams, segments the branch by colour, fits a line to the segmented mask, samples several equally-spaced cutting points along that line and converts both the branch centroid and the sampled points to world frame coordinates.

---

## Processing steps

1. Convert RGB (`bgr8`) and depth (`16UC1`, scaled mm to metres) and resize both to `IMAGE_WIDTH x IMAGE_HEIGHT`.
2. Threshold HSV between `COLOR_BRANCH_LOWER` and `COLOR_BRANCH_UPPER` to get the branch mask.
3. AND the colour mask with a valid depth mask (`0.001 m < depth < MAX_BRANCH_DEPTH`).
4. Keeps only the largest connected component to drop noise.
5. Centroid from image moments. If it falls on a background pixel, snap it to the nearest mask pixel.
6. `fit_line_to_mask` fits a line and intersects the drawn line with the mask.
7. `line_extract_n_point` takes the two extreme x-pixels of the line and linearly interpolates N points between them.
8. Validate each sampled point's depth, then request world coordinates and publish.

---

## Configuration constants

| Constant | Value | Description |
|----------|-------|-------------|
| `TIMER_DELAY` | `0.05` | Processing period (s) |
| `COLOR_BRANCH_LOWER` / `COLOR_BRANCH_UPPER` | `[80,90,70]` / `[130,255,255]` | HSV thresholds (blue branch) |
| `MAX_BRANCH_DEPTH` | `10.0` | Depth cap (meters); pixels beyond this are discarded |
| `POINT_PER_BRANCH` | `5` | Number of cutting points sampled along the fitted line |
| `BOOL_SAVE_IMAGE` | `False` | Whether to write annotated frames to `PATH_SAVE_IMAGE` |
| `PATH_SAVE_IMAGE` | `./output/image_detection/` | Output directory for saved frames |

---

## Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB image, decoded `bgr8` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Depth image, decoded `16UC1` (mm to meters) |

## Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/yolo/position_vector_image_frame` | `geometry_msgs/Point` | Centroid normalised to the image centre |
| `/yolo/position_vector_world_frame` | `geometry_msgs/Point` | Centroid in the robot base frame (meters) |
| `/yolo/buffer_positions` | `custom_interfaces/BufferPoints` | N cutting points in the world frame |
| `/yolo/image_debug` | `sensor_msgs/Image` | Annotated frame (mask overlay, fitted line, sampled points, centroid) |

`position_vector_image_frame` is `((cx - W/2)/W, (cy - H/2)/H, 0)`, so `(0, 0, 0)` means the branch is at the image centre.

## Services

| Service | Type | Purpose |
|---------|------|---------|
| `compute_world_position` | `custom_interfaces/srv/YOLOPoint` | World coordinates of the centroid |
| `compute_world_position_buffer` | `custom_interfaces/srv/BufferYOLOPoint` | World coordinates of all sampled points (batch) |
