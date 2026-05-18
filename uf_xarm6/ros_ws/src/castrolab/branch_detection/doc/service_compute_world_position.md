# service_compute_world_position

**Node Name:** `compute_world_position` \
**Executable:** `service_compute_world_coordinates` \
**Source:** [`branch_detection/service_compute_world_position.py`](../branch_detection/service_compute_world_position.py)

## Overview

This service node converts a detected branch's pixel coordinates and depth value into a 3D point in the world frame. It queries the current camera pose from TF2 and applies the full projection pipeline: image frame (2D) -> camera frame (3D) -> world frame (3D).

---

## Service

| Service | Type |
|---------|------|
| `/compute_world_position` | `interfaces/srv/YOLOPoint` |

### Request

| Field | Type | Description |
|-------|------|-------------|
| `x_pixel` | `float64` | Horizontal coordinate of the detected point (pixels) |
| `y_pixel` | `float64` | Vertical coordinate of the detected point (pixels) |
| `depth` | `float64` | Depth at that pixel (metres) |

### Response

| Field | Type | Description |
|-------|------|-------------|
| `x_world` | `float64` | X coordinate in world frame (metres) |
| `y_world` | `float64` | Y coordinate in world frame (metres) |
| `z_world` | `float64` | Z coordinate in world frame (metres) |
