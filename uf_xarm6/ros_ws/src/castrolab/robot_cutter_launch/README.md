# robot_cutter_launch - Package Overview

## Overview

This is the top-level launch package that brings up the branch-cutting system. It contains no nodes of its own and only includes the launch files of the other `castrolab` packages and the UFactory `xarm` stack, wiring the full pipeline together:

```
camera -> branch detection -> world-position service -> state machine -> move_to_point -> xArm6 motion
```

The system is split into three launch files so that each layer can be brought up (and debugged) independently: perception only, the robot together with perception, and the controllers.

---

## Launch files

### `perception.launch.py` (vision only)

Starts just the sensing stack, with no robot connection. This is useful for tuning detection or working without hardware.

```bash
ros2 launch robot_cutter_launch perception.launch.py
```

### `hardware_and_perception.launch.py` (robot and vision)

Brings up the real robot and the sensing stack, and homes the arm. This is the base layer to run before starting the controllers.

```bash
ros2 launch robot_cutter_launch hardware_and_perception.launch.py robot_ip:=192.168.1.207
```

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | `192.168.1.207` | IP address of the xArm6 |

### `controllers.launch.py` (motion control)

Starts the controller layer that actually moves the arm to detected branches. Run this after `hardware_and_perception.launch.py`, because it needs the planner services and the `/goto_initial_pose` service to already be up.

```bash
ros2 launch robot_cutter_launch controllers.launch.py
```

---

## Typical bring-up

In two terminals, each with the workspace sourced:

```bash
# Terminal 1: robot, camera, detection, and homing
ros2 launch robot_cutter_launch hardware_and_perception.launch.py robot_ip:=192.168.1.207

# Terminal 2: motion controllers
ros2 launch robot_cutter_launch controllers.launch.py
```

For vision-only work, run `perception.launch.py` on its own.

### Choosing the detector (Color vs YOLO)

Both `perception` and `hardware_and_perception` include the color detector. To use YOLO instead, swap that include for `branch_detection_yolo.launch.py`. Note that the YOLO detector subscribes to `camera/image` and `camera/depth_image` rather than the RealSense topic names, so it needs topic remapping to run against the live camera (see [`branch_detection`](../branch_detection/doc/README.md)).
