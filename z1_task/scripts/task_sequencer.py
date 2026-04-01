#!/usr/bin/env python3
"""
task_sequencer.py  –  Sequential pose executor for the Z1 arm.

No MTC, no complexity. Define a list of steps, run them in order.
Plans via MoveGroup action, executes via JTC directly
(works with Isaac Sim, Gazebo, and real hardware).

═══════════════════════════════════════════════════════════════════════════
 QUICK START
═══════════════════════════════════════════════════════════════════════════

  1. Edit the TASKS list below with your poses.
  2. ros2 launch z1_task task_sequencer.launch.py
  3. Step-by-step mode:   ros2 launch z1_task task_sequencer.launch.py step:=true
     (or: ros2 run z1_task task_sequencer.py --ros-args -p step_mode:=true)
  4. Record poses:        ros2 run z1_task task_sequencer.py --record

═══════════════════════════════════════════════════════════════════════════
 HOW TO RECORD POSES
═══════════════════════════════════════════════════════════════════════════

  Option A)  ros2 run z1_task task_sequencer.py --record
             Teleop to desired position, press Enter to capture.
             Outputs both Pose() and Joints() formats — copy into TASKS.

  Option B)  ros2 run tf2_ros tf2_echo world link06
             Manually copy xyz + quaternion into a Pose() below.
"""

import sys
import time
import threading
from dataclasses import dataclass
from typing import List, Union

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    BoundingVolume,
)
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand as GripperCmdAction
from geometry_msgs.msg import Pose as RosPose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import tf2_ros


# ─── Step types ──────────────────────────────────────────────────────────

OPEN = -1.0
CLOSED = 0.0


@dataclass
class Pose:
    """Move gripper tip to a Cartesian pose in world frame.
    Position = where the gripper tip should end up (meters).
    Orientation = link06 orientation as quaternion (qx, qy, qz, qw).
    Tip offset from link06 is applied automatically (see TIP_OFFSET_X).
    """
    name: str
    x: float; y: float; z: float
    qx: float; qy: float; qz: float; qw: float


@dataclass
class Joints:
    """Move arm to explicit joint positions (radians).
    Useful when you record joint_states directly.
    """
    name: str
    j1: float; j2: float; j3: float
    j4: float; j5: float; j6: float


@dataclass
class Gripper:
    """Set gripper position. OPEN = -1.0, CLOSED = 0.0.
    hold_time: seconds to keep commanding before cancelling the goal.
               Cancelling stops motor effort → prevents overheating.
    """
    name: str
    position: float
    hold_time: float = 1.0


@dataclass
class Home:
    """Move arm to home (all joints zero)."""
    name: str = "home"


@dataclass
class Sleep:
    """Pause execution (seconds)."""
    name: str = "sleep"
    duration: float = 1.0


Step = Union[Pose, Joints, Gripper, Home, Sleep]


# ═════════════════════════════════════════════════════════════════════════
#  ██████  EDIT YOUR TASK SEQUENCE HERE  ██████
# ═════════════════════════════════════════════════════════════════════════
#
# Pose positions  = gripper-tip target in world frame (meters).
# Pose quaternion = link06 orientation (qx, qy, qz, qw).
#   Pointing gripper straight down ≈ quat(0, 0.707, 0, 0.707).
#
# Use --record mode to capture poses from teleop!

# TASKS: List[Step] = [
#     Gripper("close gripper",    CLOSED, hold_time=0.5),
#       Joints("home",  j1= 0.0020, j2= 0.0009, j3=-0.0046,  j4= 0.0040, j5=-0.0035, j6= 0.0066),
#     Gripper("open gripper",    OPEN),
#     Joints("hover object 1",  j1= 0.0000, j2= 1.2168, j3=-1.0722, j4= 0.6601, j5=-0.0001, j6= 0.0001),
#     Joints("pregrasp obj 1",  j1=-0.1491, j2= 2.0511, j3=-1.3304, j4= 0.9354, j5=-0.0271, j6=-0.1741),
# #   Pose("hover object 1",  x= 0.3540, y=-0.0540, z= 0.2821,  qx=-0.0144, qy= 0.7383, qz= 0.0054, qw= 0.6743),
# #   Pose("pregrasp obj 1",  x= 0.3489, y=-0.0538, z= 0.1941,  qx=-0.0138, qy= 0.7376, qz= 0.0046, qw= 0.6750),
    
#     Gripper("close gripper",   CLOSED, hold_time=0.5),
#     Joints("hover container",  j1= 0.7305, j2= 1.9240, j3=-1.7579,  j4= 1.2633, j5=-0.1359, j6= 0.7254),
# #   Pose("hover container",  x= 0.2822, y= 0.2313, z= 0.3259,  qx=-0.0190, qy= 0.7359, qz=-0.0038, qw= 0.6768),
#     Gripper("release",         OPEN,   hold_time=0.5),

#       Joints("hover object 2",  j1=-0.1259, j2= 2.3296, j3=-2.1624,  j4= 1.0896, j5= 0.1524, j6= 0.0317),
#   Joints("pregrasp obj 2",  j1=-0.1098, j2= 2.4103, j3=-2.0410,  j4= 0.8767, j5= 0.1486, j6= 0.0453),
#     Gripper("close gripper",    CLOSED, hold_time=0.5),
#     Joints("hover container",  j1= 0.7305, j2= 1.9240, j3=-1.7579,  j4= 1.2633, j5=-0.1359, j6= 0.7254),
#     Gripper("release",         OPEN,   hold_time=0.5),

#     Home(),
# ]

TASKS: List[Step] = [
    Gripper("close gripper",    CLOSED, hold_time=0.5),
      Joints("home",  j1= 0.0020, j2= 0.0009, j3=-0.0046,  j4= 0.0040, j5=-0.0035, j6= 0.0066),
    Gripper("open gripper",    OPEN),
    #   Joints("highup",  j1= 0.0753, j2= 1.6070, j3=-1.6100,  j4= 1.0323, j5=-0.0285, j6= 0.2001),

    # Joints("hover lid",  j1= 0.7482, j2= 1.9805, j3=-1.6654,  j4= 1.2958, j5=-0.0152, j6= 0.2506),
  Joints("hover lid",  j1= 0.6998, j2= 1.9592, j3=-1.7233,  j4= 1.2470, j5= 0.0988, j6= 0.7659),

  Joints("pregrasp lid",  j1= 0.7133, j2= 2.0069, j3=-1.5507,  j4= 1.1071, j5= 0.0255, j6= 0.7673),
Gripper("close gripper",   CLOSED, hold_time=0.5),
  Joints("hover lid",  j1= 0.6998, j2= 1.9592, j3=-1.7233,  j4= 1.2470, j5= 0.0988, j6= 0.7659),
      Joints("highup",  j1= 0.0753, j2= 1.6070, j3=-1.6100,  j4= 1.0323, j5=-0.0285, j6= 0.2001),
      Joints("hover object 2",  j1=-0.1259, j2= 2.3296, j3=-2.1624,  j4= 1.0896, j5= 0.1524, j6= 0.0317),
    Gripper("release",         OPEN,   hold_time=0.5),

    Joints("hover object 1",  j1= 0.0000, j2= 1.2168, j3=-1.0722, j4= 0.6601, j5=-0.0001, j6= 0.0001),
    Joints("pregrasp obj 1",  j1=-0.1491, j2= 2.0511, j3=-1.3304, j4= 0.9354, j5=-0.0271, j6=-0.1741),
Gripper("close gripper",   CLOSED, hold_time=0.5),
      Joints("highup",  j1= 0.0753, j2= 1.6070, j3=-1.6100,  j4= 1.0323, j5=-0.0285, j6= 0.2001),
    Joints("hover container",  j1= 0.7305, j2= 1.9240, j3=-1.7579,  j4= 1.2633, j5=-0.1359, j6= 0.7254),
    Gripper("release",         OPEN,   hold_time=0.5),

      Joints("highup",  j1= 0.0753, j2= 1.6070, j3=-1.6100,  j4= 1.0323, j5=-0.0285, j6= 0.2001),
    Joints("hover object 2",  j1=-0.1259, j2= 2.3296, j3=-2.1624,  j4= 1.0896, j5= 0.1524, j6= 0.0317),
    Joints("pregrasp obj 2",  j1=-0.1098, j2= 2.4103, j3=-2.0410,  j4= 0.8767, j5= 0.1486, j6= 0.0453),
    Gripper("close gripper",    CLOSED, hold_time=0.5),
      Joints("highup",  j1= 0.0753, j2= 1.6070, j3=-1.6100,  j4= 1.0323, j5=-0.0285, j6= 0.2001),
    Joints("hover container",  j1= 0.7305, j2= 1.9240, j3=-1.7579,  j4= 1.2633, j5=-0.1359, j6= 0.7254),
    Gripper("release",         OPEN,   hold_time=0.5),


    Home(),
]

# ═════════════════════════════════════════════════════════════════════════



# ─── Robot config (must match SRDF / URDF) ───────────────────────────────

ARM_GROUP = "z1_arm"
EE_LINK = "link06"
WORLD_FRAME = "world"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HOME_POSITIONS = [0.0] * 6
TIP_OFFSET_X = 0.15   # link06 → gripper-tip along link06's local X


# ═════════════════════════════════════════════════════════════════════════
#  Sequencer node
# ═════════════════════════════════════════════════════════════════════════

class TaskSequencer(Node):
    def __init__(self):
        super().__init__("z1_task_sequencer")

        self.declare_parameter("step_mode", False)
        self.declare_parameter("velocity_scaling", 0.3)
        self.declare_parameter("acceleration_scaling", 0.3)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("settle_time", 5.0)
        self.declare_parameter("gripper_max_effort", 20.0)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance", 0.1)
        self.declare_parameter("joint_tolerance", 0.1)
        self.declare_parameter("tip_offset", TIP_OFFSET_X)

        self.step_mode = self.get_parameter("step_mode").value
        self.vel_scale = self.get_parameter("velocity_scaling").value
        self.acc_scale = self.get_parameter("acceleration_scaling").value
        self.plan_time = self.get_parameter("planning_time").value
        self.plan_attempts = self.get_parameter("planning_attempts").value
        self.settle_time = self.get_parameter("settle_time").value
        self.grip_effort = self.get_parameter("gripper_max_effort").value
        self.pos_tol = self.get_parameter("position_tolerance").value
        self.ori_tol = self.get_parameter("orientation_tolerance").value
        self.jnt_tol = self.get_parameter("joint_tolerance").value
        self.tip_offset = self.get_parameter("tip_offset").value

        cb = ReentrantCallbackGroup()
        self._mg = ActionClient(
            self, MoveGroup, "move_action", callback_group=cb)
        self._jtc = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
            callback_group=cb)
        self._grip = ActionClient(
            self, GripperCmdAction,
            "/gripper_controller/gripper_cmd",
            callback_group=cb)

    # ── helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _wait_future(future, timeout_sec):
        """Block until an rclpy Future completes, with timeout."""
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout=timeout_sec)

    def _prompt(self, msg):
        """Read one line from stdin; returns '' on EOF."""
        try:
            return input(msg).strip().lower()
        except EOFError:
            return ""

    def wait_servers(self, timeout=15.0):
        self.get_logger().info("Waiting for action servers ...")
        ok = True
        for label, client in [
            ("MoveGroup", self._mg),
            ("JTC",       self._jtc),
            ("Gripper",   self._grip),
        ]:
            if not client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(f"{label} action server not found")
                ok = False
            else:
                self.get_logger().info(f"  {label} OK")
        return ok

    # ── task runner ───────────────────────────────────────────────────

    def run(self, tasks: List[Step]) -> bool:
        n = len(tasks)
        self.get_logger().info(f"Running {n} steps (step_mode={self.step_mode})")
        for i, step in enumerate(tasks):
            tag = f"[{i+1}/{n}]"
            self.get_logger().info(
                f"{tag} {type(step).__name__}: {step.name}")

            if self.step_mode:
                r = self._prompt("  Enter=exec  s=skip  q=quit: ")
                if r == "q":
                    self.get_logger().info("Aborted by user.")
                    return False
                if r == "s":
                    self.get_logger().warn(f"{tag} Skipped")
                    continue

            ok = self._dispatch(step)

            if ok:
                self.get_logger().info(f"{tag} Done: {step.name}")
            else:
                self.get_logger().error(f"{tag} FAILED: {step.name}")
                if self.step_mode:
                    if self._prompt("  Continue anyway? (y/n): ") != "y":
                        return False
                else:
                    return False

        self.get_logger().info("=== All tasks complete ===")
        return True

    def _dispatch(self, step: Step) -> bool:
        if isinstance(step, Pose):
            return self._exec_pose(step)
        if isinstance(step, Joints):
            return self._exec_joints(step)
        if isinstance(step, Gripper):
            return self._exec_gripper(step)
        if isinstance(step, Home):
            return self._exec_joints_goal(HOME_POSITIONS, "home")
        if isinstance(step, Sleep):
            time.sleep(step.duration)
            return True
        return False

    # ── Pose → MoveGroup (plan-only) → JTC ──────────────────────────

    def _exec_pose(self, p: Pose) -> bool:
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = ARM_GROUP
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        pc = PositionConstraint()
        pc.header.frame_id = WORLD_FRAME
        pc.link_name = EE_LINK
        pc.target_point_offset.x = self.tip_offset
        pc.weight = 1.0

        sphere = SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[self.pos_tol])
        target = RosPose()
        target.position.x = p.x
        target.position.y = p.y
        target.position.z = p.z
        target.orientation.w = 1.0
        pc.constraint_region = BoundingVolume(
            primitives=[sphere], primitive_poses=[target])

        oc = OrientationConstraint()
        oc.header.frame_id = WORLD_FRAME
        oc.link_name = EE_LINK
        oc.orientation.x = p.qx
        oc.orientation.y = p.qy
        oc.orientation.z = p.qz
        oc.orientation.w = p.qw
        oc.absolute_x_axis_tolerance = self.ori_tol
        oc.absolute_y_axis_tolerance = self.ori_tol
        oc.absolute_z_axis_tolerance = self.ori_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal.planning_options.plan_only = True

        return self._plan_and_send(
            goal, f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

    # ── Joint goals → MoveGroup (plan-only) → JTC ───────────────────

    def _exec_joints(self, j: Joints) -> bool:
        return self._exec_joints_goal(
            [j.j1, j.j2, j.j3, j.j4, j.j5, j.j6], j.name)

    def _exec_joints_goal(self, positions: list, desc: str) -> bool:
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = ARM_GROUP
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        c = Constraints()
        for name, val in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = self.jnt_tol
            jc.tolerance_below = self.jnt_tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal.planning_options.plan_only = True

        return self._plan_and_send(goal, desc)

    # ── shared: plan with MoveGroup → execute on JTC ─────────────────

    def _plan_and_send(self, mg_goal: MoveGroup.Goal, desc: str) -> bool:
        self.get_logger().info(f"  Planning to {desc} ...")

        send_fut = self._mg.send_goal_async(mg_goal)
        if not self._wait_future(send_fut, 5.0):
            self.get_logger().error("  MoveGroup send timed out")
            return False

        gh = send_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("  MoveGroup goal rejected")
            return False

        result_fut = gh.get_result_async()
        if not self._wait_future(result_fut, self.plan_time + 10.0):
            self.get_logger().error("  Planning timed out")
            return False

        res = result_fut.result().result
        if res.error_code.val != 1:  # MoveItErrorCodes.SUCCESS == 1
            self.get_logger().error(
                f"  Planning failed (code {res.error_code.val})")
            return False

        jt = res.planned_trajectory.joint_trajectory
        if not jt.points:
            self.get_logger().warn("  Empty trajectory (already at goal?)")
            return True

        return self._send_jtc(jt)

    def _send_jtc(self, jt: JointTrajectory) -> bool:
        t = jt.points[-1].time_from_start
        dur = t.sec + t.nanosec * 1e-9
        self.get_logger().info(
            f"  Sending to JTC ({len(jt.points)} pts, {dur:.1f}s) ...")

        jtc_goal = FollowJointTrajectory.Goal()
        jtc_goal.trajectory = jt

        fut = self._jtc.send_goal_async(jtc_goal)
        if not self._wait_future(fut, 5.0):
            self.get_logger().error("  JTC send timed out")
            return False

        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("  JTC goal rejected")
            return False

        wait = max(dur + 1.0, self.settle_time)
        self.get_logger().info(f"  Executing ... (wait {wait:.1f}s)")
        time.sleep(wait)
        return True

    # ── Gripper ──────────────────────────────────────────────────────

    def _exec_gripper(self, g: Gripper) -> bool:
        goal = GripperCmdAction.Goal()
        goal.command.position = g.position
        goal.command.max_effort = self.grip_effort

        fut = self._grip.send_goal_async(goal)
        if not self._wait_future(fut, 3.0):
            self.get_logger().error("  Gripper send timed out")
            return False

        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("  Gripper goal rejected")
            return False

        label = 'OPEN' if g.position < -0.5 else 'CLOSED'
        self.get_logger().info(f"  Gripper -> {label}")
        time.sleep(g.hold_time)

        self.get_logger().info(
            f"  Cancelling gripper goal (stop motor after {g.hold_time}s)")
        cancel_fut = gh.cancel_goal_async()
        self._wait_future(cancel_fut, 2.0)
        return True


# ═════════════════════════════════════════════════════════════════════════
#  Record mode  (ros2 run z1_task task_sequencer.py --record)
# ═════════════════════════════════════════════════════════════════════════

def record_mode():
    rclpy.init()
    node = rclpy.create_node("pose_recorder")

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)

    latest_js = {}

    def _js_cb(msg: JointState):
        for n, p in zip(msg.name, msg.position):
            latest_js[n] = p

    node.create_subscription(JointState, "/joint_states", _js_cb, 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print()
    print("=== Pose Recorder ===")
    print(f"Frame: {WORLD_FRAME} -> {EE_LINK}  "
          f"(tip offset: {TIP_OFFSET_X}m)")
    print("Teleop the arm, press Enter to capture.  Ctrl-C to quit.")
    print()

    time.sleep(1.5)

    try:
        while rclpy.ok():
            input("  Press Enter to record ...")

            try:
                t = tf_buf.lookup_transform(
                    WORLD_FRAME, EE_LINK, rclpy.time.Time())
            except Exception as e:
                print(f"  TF error: {e}")
                continue

            p = t.transform.translation
            q = t.transform.rotation

            print()
            print(f'  Pose("TODO",  '
                  f'x={p.x: .4f}, y={p.y: .4f}, z={p.z: .4f},  '
                  f'qx={q.x: .4f}, qy={q.y: .4f}, '
                  f'qz={q.z: .4f}, qw={q.w: .4f}),')

            vals = [latest_js.get(j) for j in ARM_JOINTS]
            if all(v is not None for v in vals):
                print(f'  Joints("TODO",  '
                      f'j1={vals[0]: .4f}, j2={vals[1]: .4f}, '
                      f'j3={vals[2]: .4f},  '
                      f'j4={vals[3]: .4f}, j5={vals[4]: .4f}, '
                      f'j6={vals[5]: .4f}),')
            print()
    except (KeyboardInterrupt, EOFError):
        pass

    print("\nDone.")
    rclpy.shutdown()
    return 0


# ═════════════════════════════════════════════════════════════════════════
#  Main
# ═════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = TaskSequencer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if not node.wait_servers():
        rclpy.shutdown()
        return 1

    time.sleep(2.0)

    ok = node.run(TASKS)

    time.sleep(1.0)
    rclpy.shutdown()
    spin_thread.join(timeout=3.0)
    return 0 if ok else 1


if __name__ == "__main__":
    if "--record" in sys.argv:
        sys.argv.remove("--record")
        sys.exit(record_mode())
    sys.exit(main())
