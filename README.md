# Unitree Z1 ROS2 package

This is a community-driven package that enable the [Z1 Manipulator](https://shop.unitree.com/products/unitree-z1) from [Unitree](https://www.unitree.com/) to work in ROS2.

[![Humble CI](https://github.com/idra-lab/z1_ros2/actions/workflows/humble.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/humble.yml)
[![Jazzy CI](https://github.com/idra-lab/z1_ros2/actions/workflows/jazzy.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/jazzy.yml) 
[![Rolling CI](https://github.com/idra-lab/z1_ros2/actions/workflows/rolling.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/rolling.yml)

## Quick Start

To use this package in ROS2, first clone this repository in a ROS2 workspace, e.g.:
``` bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/idra-lab/z1_ros2.git
```

All external dependencies can be installed with [`rosdep`](https://wiki.ros.org/rosdep):
``` bash
rosdep update
rosdep install --from-paths ~/ros2_ws/src --ignore-src
```
Make sure that you sourced the ROS2 global workspace (`source /opt/ros/humble/setup.bash`) and then  simply build the workspace as:
``` bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Finally, make sure to source also the built workspace (`source ~/ros2_ws/install/setup.bash`).


## ROS2 packages

This repository contains different sub-packages:

- [`z1_description`](z1_description/README.md): contains the URDFs for the Z1 robot, as well as its meshes;
- [`z1_bringup`](z1_bringup/README.md): contains configuration and launch files for the Z1 manipulator;
- [`z1_hardware_interface`](z1_hardware_interface/README.md): provides the [ROS2 control](https://control.ros.org/rolling/index.html) hardware interface for the Z1 manipulator;
- [`z1_moveit`](z1_moveit/README.md): [MoveIt!](https://moveit.ai/) integration for the Z1 manipulator;
- [`z1_examples`](z1_examples/README.md): contains some simple scripts to test and validate the functionalities of the robot;


For more information for each package, please refer to the corresponding `README`.


## Architecture

The diagram below shows the full command chain from a MoveIt motion plan down to the physical motors, and which software component owns each layer.

```
 ┌─────────────────────────────────────────────────────────────────┐
 │  YOUR APPLICATION  (MoveIt, Python, MTC, RViz, CLI)             │
 └──────────────────────┬──────────────────────────────────────────┘
                        │ ROS2 action: /move_group
                        ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  MOVE_GROUP NODE  (MoveIt2)                                     │
 │                                                                 │
 │  1. IK solver (KDL)  — Cartesian pose → joint-space goal        │
 │  2. OMPL planner     — collision-free path in joint space       │
 │  3. Time parameterization — velocity/accel limits → trajectory  │
 └──────────────────────┬──────────────────────────────────────────┘
                        │ ROS2 action: FollowJointTrajectory
                        │ payload: trajectory_msgs/JointTrajectory
                        ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  JOINT_TRAJECTORY_CONTROLLER  (ros2_controllers, 500 Hz)        │
 │                                                                 │
 │  Interpolates between waypoints each cycle, writes per-joint    │
 │  position + velocity to ros2_control command interfaces.        │
 └──────────────────────┬──────────────────────────────────────────┘
                        │ command interfaces (double* in shared memory)
                        ▼
 ┌─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
 │  Z1_HARDWARE_INTERFACE  (ros2_control plugin, this repo)        │
 │                                                                 │
 │  Uses z1_sdk (libZ1_SDK.so):                                    │
 │    unitreeArm _arm                                              │
 │    write(): setArmCmd(q,qd,tau) → lowcmd, then sendRecv() UDP  │
 │    read():  sendRecv() UDP, then lowstate → state interfaces    │
 └─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┬─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
                         │ UDP (LowlevelCmd: q, qd, tau, kp, kd)
                         ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  Z1_CTRL  (Unitree daemon, from z1_controller / libZ1.so)      │
 │                                                                 │
 │  FSM state: State_LowCmd                                        │
 │  PD control law:  τ = Kp·(q_cmd − q) + Kd·(qd_cmd − qd) + τ  │
 └──────────────────────┬──────────────────────────────────────────┘
                        │ CAN bus / proprietary protocol
                        ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  MOTOR CONTROLLERS  (6 joints + gripper)                        │
 └─────────────────────────────────────────────────────────────────┘
```

**Process layout (real hardware):**

```
 ros2_control_node process              z1_ctrl process
 ┌──────────────────────┐              ┌──────────────────────┐
 │ z1_hardware_interface │    UDP       │ main.cpp             │
 │   links:              │             │   links:             │
 │   libZ1_SDK.so ───────┼────────────►│   libZ1.so           │
 │   (client)            │◄────────────┼   (server + motors)  │
 └──────────────────────┘              └──────────────────────┘
```

- **z1_sdk** (`libZ1_SDK.so`) — Unitree's client library. Provides the `unitreeArm` class, `LowlevelCmd`/`LowlevelState` structs, UDP serialization, `ArmModel` (kinematics/dynamics). Used inside the ros2_control_node process.
- **z1_controller** (`libZ1.so` + `z1_ctrl` binary) — Unitree's control daemon. Runs as a separate process, receives `lowcmd` over UDP, executes the PD servo loop, and talks to the physical motors.
- **z1_hardware_interface** — This repo's ros2_control plugin that bridges ROS2 controllers to the Unitree SDK.

Both Unitree libraries are closed-source (headers + prebuilt `.so`); they are fetched automatically at build time via CMake `FetchContent`.


## Robot in action

To get started with the Z1 manipulator in the simulation environment, you may call
```
ros2 launch z1_bringup z1.launch.py starting_controller:=joint_trajectory_controller
```
This make sure to launch the robot with the [`joint_trajectory_controller`](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html), which provide a simple motion planning facility. 

To test the proper functionality, we can use the [`waypoint_test.py`](./z1_examples/z1_examples/waypoint_test.py) script to send a default plan as follows:
```
ros2 run z1_examples waypoint_test.py
```
The outcome of the simulation shall be the following:

![](/docs/resources/gazebo-waypoint-example.gif)

By using the same bringup launch file, it becomes really easy to transition from the simulation environment to the connection with the real robot. It is necessary to launch
```
ros2 launch z1_bringup z1.launch.py starting_controller:=joint_trajectory_controller sim_ignition:=false
```
alongside with the `waypoint_test.py` executable, and the hardware interface which connects to the real robot is loaded over the simulator interface, and the robot performs the same motion.

![](/docs/resources/robot-waypoint-example.gif)

**Note:** this example uses the `position` command interface for the robot (as specified in the [configuration file](z1_bringup/config/z1_controllers.yaml) for the `joint_trajectory_controller`), and you may have some trouble replicating the simulation on your machine.
As mentioned in [this issue](https://github.com/idra-lab/z1_ros2/issues/8), a temporary fix is to delete all appearences of the `effort` command interface from the [`z1.ros2_control.xacro`](./z1_description/urdf/z1.ros2_control.xacro). 
Note that this is a problem of the ROS2 control plugin for Ignition, and not some misconfiguration of this package; the connection with the hardware does not suffer this problem.
A better fix to deleting parts from the URDF is planned but not implemented yet.

### Pose Goal Example

The [`pose_goal.launch.py`](./z1_examples/launch/pose_goal.launch.py) launch file brings up the full MoveIt stack (including `z1_bringup` and `joint_trajectory_controller`) alongside RViz, then automatically runs the `pose_goal.py` example after a short delay.

For **simulation** (Ignition):
```
ros2 launch z1_examples pose_goal.launch.py sim_ignition:=true
```

For **simulation** (Isaac Sim):
```
ros2 launch z1_examples pose_goal.launch.py sim_isaac:=true
```

For **real hardware**:
```
ros2 launch z1_examples pose_goal.launch.py sim_ignition:=false
```

The `sim_ignition` argument defaults to `true`, and `sim_isaac` defaults to `false`.  
When `sim_isaac:=true` is set, Ignition is automatically disabled in the MoveIt launch.

### Position control using Moveit2 joint_trajectory_controller
```
ros2 launch z1_moveit z1_moveit.launch.py sim_isaac:=true
ros2 launch z1_moveit z1_moveit.launch.py sim_ignition:=false
ros2 launch z1_moveit z1_moveit.launch.py sim_ignition:=true
```

```
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/rgb \
  -r camera_info:=/camera_info \
  --params-file $(ros2 pkg prefix z1_examples)/share/z1_examples/config/apriltag.yaml
```


```
ros2 run z1_examples apriltag_localizer.py --ros-args \
  -p world_frame:=world \
  -p landmark_frame:=apriltag_69_landmark \
  -p observed_tag_frame:=apriltag_69 \
  -p camera_frame:=Camera_OmniVision_OV9782_Color
```


```
ros2 topic echo /apriltag/detections --once
```
```
ros2 launch z1_examples pick_cube.launch.py sim_isaac:=true sim_ignition:=false
ros2 launch z1_task pick_cube_task.launch.py sim_isaac:=true sim_ignition:=false approach_offset_x:=-0.03 grasp_offset:=0.12
ros2 launch z1_task pick_cube_task_MTC.launch.py sim_isaac:=true sim_ignition:=false approach_offset_x:=-0.03 grasp_offset:=0.12
```



### REAL SIM
```
ros2 launch z1_task pick_cube_task_MTC.launch.py \
  sim_ignition:=false \
  sim_isaac:=false \
  image_topic:=/camera/camera/color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame
```
```
ros2 launch realsense2_camera rs_launch.py
```
```
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file $(ros2 pkg prefix z1_examples)/share/z1_examples/config/apriltag_real.yaml
```
```
ros2 run z1_examples apriltag_localizer.py --ros-args \
  -p world_frame:=world \
  -p landmark_frame:=apriltag_0_landmark \
  -p observed_tag_frame:=apriltag_0 \
  -p camera_frame:=camera_color_optical_frame
```
```
ros2 run tf2_ros static_transform_publisher 0.0 0.18 0.0 0.0 0.0 0.0 world apriltag_0_landmark
```

### REAL SIM SIMPLE
```
ros2 launch z1_task pick_cube_task_MTC_simple.launch.py \
  sim_ignition:=false \
  sim_isaac:=false \
  image_topic:=/camera/camera/color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  camera_frame:=camera_color_optical_frame \
  landmark_frame:=apriltag_4_landmark \
  observed_tag_frame:=apriltag_4 \
  tag_frame:=apriltag_0 \
  goal_orientation_tolerance:=0.5
```

### ISAAC SIM SIMPLE
```
ros2 launch z1_task pick_cube_task_MTC_simple.launch.py \
  sim_ignition:=false \
  sim_isaac:=true \
  image_topic:=/rgb \
  camera_info_topic:=/camera_info \
  camera_frame:=Camera_OmniVision_OV9782_Color \
  landmark_frame:=apriltag_69_landmark \
  observed_tag_frame:=apriltag_69 \
  tag_frame:=apriltag_21 \
  goal_orientation_tolerance:=0.5
```



`publish_child_frame` defaults to `camera_frame` (omit it when you only need `world` → camera). Set `publish_child_frame` separately if you publish `world` → some other frame under the camera (e.g. `base_link`).

TF chain the node expects:

- `world` → `apriltag_69_landmark` — static tag pose (e.g. from Isaac / your map).
- `camera_frame` → `observed_tag_frame` — tag in camera frame from `apriltag_ros` (updates as the camera moves).
- `world` → `publish_child_frame` — **published by this node**: combines the landmark pose with the inverse of the camera→tag observation so the camera (or another frame under it) is localized in `world`.



### Gripper control

The `gripper_controller` (`position_controllers/GripperActionController`) is the default gripper controller used by MoveIt.
It has **stall detection** enabled (`allow_stalling: true`) so that when the gripper closes on an object and cannot reach the target position, the action **succeeds** instead of hanging forever.
This is essential for pick-and-place: MoveIt sends a "close" goal, the gripper stalls against the object, and the pipeline continues.

A second controller, `gripper_effort_controller` (`effort_controllers/JointGroupEffortController`), is spawned **inactive** at launch and allows direct torque control of the gripper.
This is useful for force-controlled grasping where you want to command a closing torque rather than a target position.

The `gripper_control.py` example provides a tkinter GUI with:
- A **position slider** that sends `GripperCommand` action goals (active in position mode)
- An **effort slider** that publishes torque commands (active in effort mode)
- A **mode switch** button that toggles between `gripper_controller` and `gripper_effort_controller` at runtime
- **Live state** display showing the gripper's measured position, velocity, and effort

```
ros2 run z1_examples gripper_control.py
```

You can also switch controllers manually from the command line:
```bash
# Switch to effort control
ros2 control switch_controllers \
  --deactivate gripper_controller \
  --activate gripper_effort_controller

# Publish a closing torque
ros2 topic pub /gripper_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [5.0]}"

# Switch back to position control
ros2 control switch_controllers \
  --deactivate gripper_effort_controller \
  --activate gripper_controller
```

> **Note:** While `gripper_effort_controller` is active, MoveIt cannot command the gripper (the position-based `gripper_controller` is inactive). Switch back to position mode before running MoveIt pick-and-place.

### Impedance control
```
ros2 launch z1_bringup z1.launch.py starting_controller:=operational_impedance_controller sim_isaac:=true
ros2 run z1_examples impedance_marker.py --ros-args -p standalone:=true -p controller:=operational_impedance_controller
```

or
```
ros2 launch z1_bringup z1_unified.launch.py starting_controller:=operational_impedance_controller sim_isaac:=true
```

Hardware

```
ros2 launch z1_bringup z1.launch.py starting_controller:=operational_impedance_controller sim_ignition:=false
```


```
# Much faster reference tracking (biggest impact)
ros2 param set /cartesian_impedance_controller filtering.pose 0.5

# Stiffer position tracking
ros2 param set /cartesian_impedance_controller stiffness.translation_x 1000.0
ros2 param set /cartesian_impedance_controller stiffness.translation_y 1000.0
ros2 param set /cartesian_impedance_controller stiffness.translation_z 1000.0
ros2 param set /cartesian_impedance_controller stiffness.rotation_x 40.0
ros2 param set /cartesian_impedance_controller stiffness.rotation_y 40.0
ros2 param set /cartesian_impedance_controller stiffness.rotation_z 40.0

# Allow faster torque ramp-up
ros2 param set /cartesian_impedance_controller delta_tau_max 5.0
```
```
ros2 launch z1_bringup z1.launch.py starting_controller:=cartesian_impedance_controller sim_isaac:=true
ros2 launch z1_bringup z1.launch.py starting_controller:=cartesian_impedance_controller sim_ignition:=false

```
```
ros2 run z1_examples impedance_marker.py --ros-args -p standalone:=true -p controller:=cartesian_impedance_controller
```

```
ros2 topic pub --once /cartesian_impedance_controller/reference_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}"


ros2 topic pub --once /cartesian_impedance_controller/reference_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

ros2 run z1_examples impedance_marker.py



# Switch from impedance -> trajectory (to go home via MoveIt)
ros2 control switch_controllers \
  --deactivate cartesian_impedance_controller \
  --activate joint_trajectory_controller

# Switch back to impedance
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate cartesian_impedance_controller

## Contributing

Everyone is welcome to contribute to this repository. 

If you want to improve something, or have some particular request, please first open an issue to disclose your idea with everyone.

As general rule, please develop your feature/bug-fix on a new branch, and create a pull request targeting the **development branch** (`devel`).
There we will make sure that the change is working as expected, and will update the reference of the `main` branch accordingly, to guarantee the stability of such branch.

## Lessons Learned & Troubleshooting

### 1. Hardware Overheating ("Motor windings overheat")
The Unitree Z1 SDK uses a PD controller for joint control: `tau = Kp * (q_cmd - q) + Kd * (qd_cmd - qd) + tau_cmd`. 
If the ROS 2 controller (like `joint_trajectory_controller`) is configured to only send `position` commands, the velocity command (`qd_cmd`) defaults to `0.0`. With the SDK's high default derivative gain (`Kd = 2000`), this creates a massive artificial damping force. The motors will fight against this damping to follow the position trajectory, drawing excessive current and quickly triggering a "Motor windings overheat" error (especially on smaller motors like Motor 4).
**Fix:** Always ensure that `velocity` is included in the `command_interfaces` of your controllers in `z1_controllers.yaml` so that feedforward velocity is properly passed to the hardware.

### 2. ROS 2 Control Interface Claiming (Gain Overwriting)
When a controller claims multiple interfaces (e.g., `position` and `velocity`), `ros2_control` passes them to the hardware interface's `perform_command_mode_switch` sequentially (e.g., `"joint1/position"`, then `"joint1/velocity"`). 
If the hardware interface processes these sequentially in a loop and blindly applies gains, a later interface can overwrite the gains of an earlier one. For example, processing `velocity` might set `Kp = 0.0`, overwriting the `Kp` set by the `position` interface just a microsecond earlier, causing the arm to go limp and drop.
**Fix:** Decouple the parsing of claimed interfaces from the application of gains. Gather all claimed interfaces first, then apply gains based on priority (e.g., if `position` is claimed at all, keep `Kp` active).

### 3. Manual `sendRecv()` Required in the Hardware Interface Read/Write Loop

The Z1 SDK provides a background thread (`sendRecvThread`) that calls `unitreeArm::sendRecv()` at 500 Hz. It might seem redundant to also call `_arm->sendRecv()` manually inside the `ros2_control` `read()` and `write()` methods — but **removing those calls causes joint 4 to overheat**.

The reason is a subtle interaction between `sendRecvThread` and `setArmCmd()`. The SDK's `unitreeArm::sendRecv()` doesn't just perform the UDP exchange; it also copies the **unitreeArm-level** command fields (`arm.q`, `arm.qd`, `arm.tau`) into `lowcmd` before sending. The hardware interface only calls `setArmCmd()` (which writes directly to `lowcmd`) and never updates those unitreeArm-level fields. So the background thread's `sendRecv()` periodically **overwrites** our commands in `lowcmd` with stale values (zeros or the position captured at startup), and those stale values are what actually get sent to the arm over UDP.

With the manual `sendRecv()` calls in `read()` and `write()`, our correct commands reach the arm at least part of the time (the manual call fires immediately after `setArmCmd()`, before the background thread can clobber `lowcmd`). Without them, the arm only ever receives stale targets and the PD controller generates sustained corrective torques, overheating the thermally weakest motor (joint 4).

The SDK's own `lowcmd_development.cpp` example shows the intended LOWCMD pattern: **shut down `sendRecvThread`**, then run a manual loop calling `setArmCmd()` + `sendRecv()`. The proper long-term fix is to either:
1. Shut down `sendRecvThread` after entering `LOWCMD` mode (restart it only for FSM transitions like `backToStart()`), or
2. Keep `sendRecvThread` running but also update `_arm->q`, `_arm->qd`, `_arm->tau` alongside `setArmCmd()` so the background thread sends consistent values.

### 4. "Startup Position = Home" — sendRecvThread Clobbering Joint Commands

**Symptom:** When the Z1 is powered on, the hardware interface treats whatever angle it starts at as "home". All subsequent ROS 2 commands (MoveIt goals, trajectory controller targets) appear to work relative to the startup position rather than in absolute joint coordinates. The arm either drifts back toward the startup pose or only partially follows commanded trajectories.

**Root cause:** This is a direct consequence of problem #3 above, but manifests as a positioning/calibration problem rather than overheating. The `unitreeArm` class has two layers of command storage:

- **unitreeArm-level fields:** `_arm->q`, `_arm->qd`, `_arm->tau`, `_arm->gripperQ`, `_arm->gripperW`, `_arm->gripperTau` (member variables on the `unitreeArm` object)
- **lowcmd fields:** `_arm->lowcmd->q`, etc. (the actual UDP command buffer sent to `z1_ctrl`)

The key to understanding this bug is that **every** call to `unitreeArm::sendRecv()` — whether from our code or the background thread — does two things in sequence:

1. **Copies** `_arm->q` → `lowcmd->q` (overwrites the UDP buffer with the member field)
2. **Sends** `lowcmd` over UDP to `z1_ctrl`

There is **one** `lowcmd` buffer shared between two concurrent writers:

- **Thread A** (ros2_control `write()` at 500 Hz): calls `setArmCmd()` which writes the correct ROS 2 target into `lowcmd`, then calls `sendRecv()`.
- **Thread B** (`sendRecvThread` at 500 Hz): calls `sendRecv()` on its own schedule.

The race looks like this:

```
Thread A (write):                        Thread B (sendRecvThread):

setArmCmd(correct_target)
  → lowcmd = [correct, e.g. 1.0 rad]
                                         sendRecv():
                                           step 1: lowcmd = _arm->q = [stale, 0.0]
                                           step 2: UDP send(lowcmd) → z1_ctrl gets 0.0  ✗
sendRecv():
  step 1: lowcmd = _arm->q = [stale, 0.0]
  step 2: UDP send(lowcmd) → z1_ctrl gets 0.0  ✗
```

The correct value written by `setArmCmd()` survives in `lowcmd` for microseconds before the next `sendRecv()` — from either thread — stomps it with `_arm->q`. Since the hardware interface never updated `_arm->q`, it still holds the startup position. **Every single UDP send delivers the stale startup position, not the commanded target.**

In `LOWCMD` mode, `z1_ctrl` directly applies these `lowcmd` values via its PD control law (`τ = Kp·(q_cmd − q) + Kd·(qd_cmd − qd) + τ`). So the motors are constantly driven toward the startup position, and the arm appears to treat its power-on pose as "home".

**Why the standalone z1_sdk demos don't have this problem:**

The SDK examples use three strategies that all avoid the clobbering:

1. **High-level FSM commands** (`MoveJ`, `backToStart`, `labelRun`): These trigger FSM state transitions in `z1_ctrl`. In states like `MOVEJ` or `BACKTOSTART`, `z1_ctrl` generates motor commands internally from its own trajectory planner and **ignores** the `lowcmd` arriving over SDK UDP. The background thread's clobbering is irrelevant because `z1_ctrl` isn't using those values.

2. **Shut down `sendRecvThread`** (`lowcmd_development.cpp`, `lowcmd_multirobots.cpp`): These examples call `sendRecvThread->shutdown()` before entering the manual control loop, eliminating the clobbering source entirely. Only the explicit `setArmCmd()` + `sendRecv()` calls in the loop send commands.

3. **Keep `_arm->q` in sync** (`highcmd_development.cpp`): The example calls `startTrack(JOINTCTRL)` which initializes `_arm->q = lowstate->getQ()`, then updates `arm.q` every iteration in the control loop. The background thread copies the correct (current) value because the user-facing field is always up to date.

**Fix:** In `write()`, sync the unitreeArm-level fields before calling `setArmCmd()`:

```cpp
_arm->q   = _arm_cmd.q;
_arm->qd  = _arm_cmd.qd;
_arm->tau  = _arm_cmd.tau;
_arm->gripperQ   = _gripper_cmd.q;
_arm->gripperW   = _gripper_cmd.qd;
_arm->gripperTau = _gripper_cmd.tau;

_arm->setArmCmd(_arm_cmd.q, _arm_cmd.qd, _arm_cmd.tau);
_arm->setGripperCmd(_gripper_cmd.q, _gripper_cmd.qd, _gripper_cmd.tau);
_arm->sendRecv();
```

Now all three senders (write, read, background thread) transmit consistent commands. The background thread copies `_arm->q` into `lowcmd`, but since `_arm->q` now matches what `setArmCmd()` wrote, the clobbering produces the same correct value.
