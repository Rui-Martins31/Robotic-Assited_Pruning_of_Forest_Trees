# realsense_view - Package Overview

## Overview

This package brings up the Intel RealSense D455 camera and visualises its streams in RViz2. It is the sensing frontend of the cutting pipeline, because the color and depth images it publishes are what [`branch_detection`](../branch_detection) subscribes to.

It is a launch-only package with no nodes of its own. The launch file starts three things:

1. the RealSense driver (`realsense2_camera_node`),
2. a static TF that attaches the camera frame to the arm's end-effector
3. RViz2 preloaded with a camera view.

---

## Launch

```bash
ros2 launch realsense_view camera_view.launch.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_pointcloud` | `true` | Enable the depth point-cloud topic |

---

## What it starts

### `realsense2_camera_node`

The RealSense driver, configured for synchronised color and depth:

| Parameter | Value |
|-----------|-------|
| `enable_color` | `true` |
| `enable_depth` | `true` |
| `enable_sync` | `true` |
| `rgb_camera.profile` | `640x480x30` |
| `depth_module.profile` | `640x480x15` |
| `pointcloud.enable` | from `enable_pointcloud` |
| `pointcloud.ordered_pc` | `false` |

Relevant published topics:

| Topic | Description |
|-------|-------------|
| `/camera/color/image_raw` | RGB image |
| `/camera/depth/image_rect_raw` | Depth image (16-bit, mm) |
| `/camera/aligned_depth_to_color/image_raw` | Depth aligned to the color frame |


### `static_transform_publisher` (`eef_to_camera_link`)

Publishes the transform `link_eef -> camera_link`, connecting the RealSense TF tree to the robot TF tree so that detected points can be expressed in the robot base frame.

> **Calibration TODO:** this is currently the identity transform (`0 0 0  0 0 0 1`). It must be replaced with the real hand-eye extrinsics (`x y z  qx qy qz qw` of the camera relative to `link_eef`) once the camera mount is calibrated, otherwise world-frame branch positions will be offset by the true camera-to-EEF pose.

### `rviz2`

Launched with the bundled `config/camera_view.rviz` to visualise the camera streams and TF tree.
