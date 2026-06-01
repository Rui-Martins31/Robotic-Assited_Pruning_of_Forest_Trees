# controller - Package Overview

## Overview

This package drives the xArm6 towards the branch positions. The current design is built around a state machine that orchestrates three cooperating pieces:

- **`node_state_machine`**: presents four different states: `IDLE -> DETECTING -> EXECUTING -> HOME`, one state at a time.
- **`node_move_to_point`**: an action server that plans and executes a motion to a single point and reports success or failure back to the state machine.
- **`node_goto_initial_pose`**: a service that moves the arm to a fixed home pose, and also auto-homes once at startup. The shared home joint angles are `[-2.0944, -1.309, -0.523599, 0.0, 0.610865, 0.0]` (radians).

All robot motion goes through the UFactory `xarm_planner` services (`xarm_pose_plan`, `xarm_joint_plan`, `xarm_exec_plan`, defined in `xarm_msgs`), which wrap MoveIt 2 planning and execution. The controllers never command the robot directly, instead they request a plan and then request its execution.

The package also still contains two earlier single-node controllers, `node_controller_final_pose` and `node_controller_buffer`, kept for reference. They are not part of the state-machine pipeline and are not started by the launch files (see [Legacy nodes](#legacy-nodes)).

---

## Launch

The controller nodes are started by the [`robot_cutter_launch`](../robot_cutter_launch) package, which brings up the action server and the state machine together:

```bash
ros2 launch robot_cutter_launch controllers.launch.py
```

This assumes the robot, planner and detection are already running.

---

## Nodes

## `node_state_machine`

On startup it waits for the action server and the `/goto_initial_pose` service, then configures the robot (it sets the collision sensitivity and resets the controller state) before going idle. Each received buffer drives one pass of the cycle.

### States

| State | Behaviour |
|-------|-----------|
| `IDLE` | Wait for a `BufferPoints` message |
| `DETECTING` | Validate the buffer, store the points, then move to `EXECUTING` |
| `EXECUTING` | Send the current point as a `MoveToPoint` goal to the action server |
| `HOME` | Return to home pose. If points remain, go back to `EXECUTING`, otherwise finish |

After the last point the machine stays put. Uncomment the marked line in `on_home` to loop back to `IDLE` and accept the next buffer.

---

## `node_move_to_point`

The motion executor, exposed as a `MoveToPoint` action server. For each goal it:

1. Reads the current end-effector position from TF (`link_base -> link_eef`).
2. Computes `direction` and converts it to an orientation quaternion (see [Orientation](#orientation-_direction_to_quaternion)).
3. Builds a `Pose` goal at the target, clamping `z` to a safe floor of 0.2 meters.
4. Plans the pose, then executes it, blocking on each async service call.
5. Publishes `planning` feedback and returns a `success` and `message` result.

---

## `node_goto_initial_pose`

Homing node. It exposes a service named `/goto_initial_pose` that plans and executes a joint-space motion to the default joint angles stated previously, and it also auto-homes once at startup via a one-shot timer.

---

## Orientation: `_direction_to_quaternion`

The action server orients the tool to point along the vector from the current end-effector to the target. Given a unit `direction` and a reference axis (`[0, 0, 1]` by default), it builds the shortest-arc quaternion that rotates the reference onto the direction:

- Degenerate target (direction near 0): identity orientation.
- Parallel case (`direction` near `reference`): identity orientation.
- Antiparallel case (`direction` near `-reference`): 180 degree rotation about a perpendicular axis.
- General case: quaternion `[cross, 1 + dot]`, normalised.

---

## Configuration: `config/moveit_py_config.yaml`

MoveIt planning parameters:

| Parameter | Value |
|-----------|-------|
| `planning_pipelines.pipeline_names` | `["ompl"]` |
| `plan_request_params.planning_pipeline` | `ompl` |
| `plan_request_params.planning_attempts` | `1` |
| `plan_request_params.max_velocity_scaling_factor` | `0.5` |
| `plan_request_params.max_acceleration_scaling_factor` | `0.5` |

---

## Legacy nodes

`node_controller_final_pose` and `node_controller_buffer` are the earlier single-node controllers that predate the state-machine design. They subscribe directly to detection topics and call the `xarm_pose_plan`, `xarm_joint_plan` and `xarm_exec_plan` services themselves, using the same direction-to-quaternion orientation and 0.2 meters `z` floor.