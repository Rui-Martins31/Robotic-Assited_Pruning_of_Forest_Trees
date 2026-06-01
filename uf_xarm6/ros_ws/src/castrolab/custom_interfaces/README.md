# custom_interfaces - Package Overview

## Overview

This package defines the custom ROS 2 messages, services and actions that the `castrolab` packages use to exchange branch-detection data. It is a pure interface package with no nodes.

Three ideas are captured here:

- A **single detected point**
- A **buffer of N cutting points**
- A **move-to-point goal**

---

## Messages

### `msg/YOLOPoint.msg`

A single detection, in image space.

| Field | Type | Description |
|-------|------|-------------|
| `x_pixel` | `float64` | Horizontal pixel coordinate in image frame |
| `y_pixel` | `float64` | Vertical pixel coordinate in image frame |
| `depth` | `float64` | Depth at that pixel, in meters |

### `msg/BufferPoints.msg`

A set of cutting points already expressed in the robot base frame.

| Field | Type | Description |
|-------|------|-------------|
| `size` | `int32` | Number of points in `points` |
| `points` | `geometry_msgs/Point[]` | Cutting points `(x, y, z)` in the robot base frame (meters) |

---

## Services

### `srv/YOLOPoint.srv`

Convert a single pixel and depth to a world-frame point.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `x_pixel` | `float64` | Pixel X of the point |
| `y_pixel` | `float64` | Pixel Y of the point |
| `depth` | `float64` | Depth at that pixel (meters) |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `x_world` | `float64` | X in the robot base frame (meters) |
| `y_world` | `float64` | Y in the robot base frame (meters) |
| `z_world` | `float64` | Z in the robot base frame (meters) |

### `srv/BufferYOLOPoint.srv`

Batch version of the above that converts N pixels and depths to N world-frame points in a single call.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `x_pixels` | `float64[]` | Pixel X of each sampled point |
| `y_pixels` | `float64[]` | Pixel Y of each sampled point |
| `depths` | `float64[]` | Depth at each pixel (meters) |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `size` | `int32` | Number of points returned |
| `points` | `geometry_msgs/Point[]` | Points `(x, y, z)` in the robot base frame (meters) |

---

## Actions

### `action/MoveToPoint.action`

A single "move the arm to this point" request. Sends one goal per cutting point to the `move_to_point` action server, which plans and executes the motion and reports the outcome.

**Goal**

| Field | Type | Description |
|-------|------|-------------|
| `point` | `geometry_msgs/Point` | Target point in the robot base frame (meters) |

**Result**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the plan and execution completed |
| `message` | `string` | Human-readable outcome / failure reason |

**Feedback**

| Field | Type | Description |
|-------|------|-------------|
| `status` | `string` | Current phase of the motion (debug) |
