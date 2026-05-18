# _globals

**Source:** [`branch_detection/_globals.py`](../branch_detection/_globals.py)

## Overview

This file centralises all global constants, such as  camera intrinsics and image configuration, used across the `branch_detection` package.

---

## Constants

### Image Dimensions

| Constant | Description |
|----------|-------------|
| `IMAGE_WIDTH` | Width to which all images are resized before YOLO inference and depth lookup (pixels) |
| `IMAGE_HEIGHT` | Height to which all images are resized before YOLO inference and depth lookup (pixels) |

### Camera Intrinsics

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