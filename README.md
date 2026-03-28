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
ros2 topic echo /apriltag/detections --once
```

```
ros2 run z1_examples apriltag_localizer.py --ros-args \
  -p world_frame:=world \
  -p landmark_frame:=apriltag_69_landmark \
  -p observed_tag_frame:=apriltag_69 \
  -p camera_frame:=Camera_OmniVision_OV9782_Color
```

`publish_child_frame` defaults to `camera_frame` (omit it when you only need `world` → camera). Set `publish_child_frame` separately if you publish `world` → some other frame under the camera (e.g. `base_link`).

TF chain the node expects:

- `world` → `apriltag_69_landmark` — static tag pose (e.g. from Isaac / your map).
- `camera_frame` → `observed_tag_frame` — tag in camera frame from `apriltag_ros` (updates as the camera moves).
- `world` → `publish_child_frame` — **published by this node**: combines the landmark pose with the inverse of the camera→tag observation so the camera (or another frame under it) is localized in `world`.



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
