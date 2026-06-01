# _globals

**Source:** [`branch_detection/_globals.py`](../branch_detection/_globals.py)

## Overview

This file centralises the global constants shared across the `branch_detection` package. Both detector nodes resize every frame to `IMAGE_WIDTH x IMAGE_HEIGHT`, and the intrinsics are derived from that resized resolution and the camera's horizontal field of view, so the `compute_world_position` projection is consistent with what the detectors actually process.

---

## Constants

### Image dimensions

| Constant | Description |
|----------|-------------|
| `IMAGE_WIDTH` | Width to which all images are resized before processing (pixels) |
| `IMAGE_HEIGHT` | Height to which all images are resized before processing (pixels) |

### Camera intrinsics *(Used in gazebo only)*

| Constant | Description |
|----------|-------------|
| `CAMERA_FOV_HORIZONTAL` | Horizontal field of view of the camera (radians) |
| `CAMERA_FX` | Focal length along the X axis (pixels) |
| `CAMERA_FY` | Focal length along the Y axis (pixels) |
| `CAMERA_CU` | Principal point X |
| `CAMERA_CV` | Principal point Y |

### Depth

| Constant | Description |
|----------|-------------|
| `CAMERA_MAX_DIST` | Depth values above this are clamped to this limit (metres) |
| `CAMERA_MIN_DIST` | Depth values below this are clamped to this limit (metres) |