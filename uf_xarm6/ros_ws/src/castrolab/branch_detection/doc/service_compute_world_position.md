# service_compute_world_position

**Node Name:** `compute_world_position` \
**Executable:** `service_compute_world_coordinates` \
**Source:** [`branch_detection/service_compute_world_position.py`](../branch_detection/service_compute_world_position.py)

## Overview

This node converts detected pixel coordinates and depth into 3D points in the world frame. It queries the current end-effector pose from TF2 and applies the projection pipeline:

```
pixel (2D) -> camera optical frame (3D) -> camera body frame (3D) -> world frame (3D)
```

It exposes two services: one for a single point and a batch version for the cutting buffer. Both share the same per-call pose lookup (`link_base -> link_eef`), so a batch reuses one pose for all its points.

---

## Services

### `compute_world_position` (`custom_interfaces/srv/YOLOPoint`)

Single point.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `x_pixel` | `float64` | Pixel X |
| `y_pixel` | `float64` | Pixel Y |
| `depth` | `float64` | Depth at that pixel (meters) |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `x_world` | `float64` | X in the robot base frame (meters) |
| `y_world` | `float64` | Y in the robot base frame (meters) |
| `z_world` | `float64` | Z in the robot base frame (meters) |

Quick test:

```bash
ros2 service call /compute_world_position custom_interfaces/srv/YOLOPoint \
  "{x_pixel: 100.0, y_pixel: 100.0, depth: 2.0}"
```

### `compute_world_position_buffer` (`custom_interfaces/srv/BufferYOLOPoint`)

Batch of N points.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `x_pixels` | `float64[]` | Pixel X of each point |
| `y_pixels` | `float64[]` | Pixel Y of each point |
| `depths` | `float64[]` | Depth at each pixel (meters) |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `points` | `geometry_msgs/Point[]` | Points in the robot base frame (meters) |
| `size` | `int32` | Number of points returned |

---

## Transformation details

| Step | Method | Notes |
|------|--------|-------|
| Pose lookup | `get_current_joint_pose` | TF `link_base -> link_eef` |
| Pose -> matrix | `get_homogeneous_matrix` | Quaternion -> rotation, assembled into a 4x4 homogeneous matrix |
| Pixel -> world | `pixel_to_world_coord` | Uses intrinsics from [`globals.md`](globals.md); clamps depth to `[CAMERA_MIN_DIST, CAMERA_MAX_DIST]`; applies a fixed optical-to-body rotation before the world transform |

The optical-to-body rotation applied is:

```
R = [[ 0, -1, 0],
     [ 1,  0, 0],
     [ 0,  0, 1]]
```
