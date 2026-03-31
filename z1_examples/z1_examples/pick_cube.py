#!/usr/bin/env python3

"""
Pick a cube detected via AprilTag 21.

Sequence:
    1. Look up apriltag_21 in world frame via TF2
    2. Move to pre-grasp (standoff above cube along tag Z)
    3. Open gripper
    4. Hover directly above grasp (>=10 cm world-Z clearance)
    5. Descend along tag Z to grasp pose
    6. Close gripper (stalls on cube)
    7. Lift

The tag's Z axis is assumed to point OUT of the tag surface (standard
AprilTag convention).  For a tag on top of a horizontal cube the Z axis
points upward, so "approach along Z" means descend from above.

All offsets are relative to the tag pose and measured along its Z axis.
Tune the constants at the top of this file for your setup.

Prerequisites:
    - move_group running  (ros2 launch z1_moveit z1_moveit.launch.py)
    - joint_trajectory_controller + gripper_controller active
    - apriltag_ros publishing TF for apriltag_21
    - TF chain:  world → … → camera_optical → apriltag_21

Usage:
    ros2 launch z1_examples pick_cube.launch.py
"""

import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit.planning import MoveItPy


# ── Robot / MoveIt constants ─────────────────────────────────────────────
PLANNING_GROUP = "z1_arm"
EE_LINK = "link06"
WORLD_FRAME = "world"

# ── AprilTag ─────────────────────────────────────────────────────────────
TAG_FRAME = "apriltag_21"

# ── Gripper settings ─────────────────────────────────────────────────────
GRIPPER_OPEN = -1.0       # rad  (SRDF "open" state)
GRIPPER_CLOSE = 0.0       # rad  (fully closed / stall on object)
GRIPPER_EFFORT = 20.0     # Nm   max effort while grasping

# ── Pick offsets (metres, along tag Z axis) ──────────────────────────────
# Positive = further away from tag surface (outward along Z).
# Negative = below the tag surface (into the cube).
# Note: EE_LINK is link06, and the gripper tip is ~0.13m along EE X axis.
# So we add 0.13m to the desired tip offsets.
APPROACH_OFFSET = 0.25    # pre-grasp standoff above the tag (0.12 + 0.13)
GRASP_OFFSET = 0.115      # descend this far below the tag center (-0.015 + 0.13)
HOVER_CLEARANCE = 0.10    # min world-Z clearance above grasp before final descent
HOVER_X_RETREAT = -0.02   # retreat along tag-X before descent (negative = away from tag face)
LIFT_HEIGHT = 0.3        # lift distance above the grasp pose


# ── Quaternion helpers (x, y, z, w) ─────────────────────────────────────
Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def _qnorm(q: Quat) -> Quat:
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _qconj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _qmul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qrot(q: Quat, v: Vec3) -> Vec3:
    qn = _qnorm(q)
    vq: Quat = (v[0], v[1], v[2], 0.0)
    r = _qmul(_qmul(qn, vq), _qconj(qn))
    return (r[0], r[1], r[2])


def _pose_stamped(pos: Vec3, quat: Quat) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = WORLD_FRAME
    p.pose.position.x, p.pose.position.y, p.pose.position.z = pos
    q = _qnorm(quat)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


# ── Helper node: TF lookup + gripper action client ──────────────────────

class _Helper(Node):

    def __init__(self):
        super().__init__("pick_cube_helper")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)
        self._grip = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self._jtc = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self._status_pub = self.create_publisher(String, "/pick_cube/status", 10)
        self._step = 0

    def status(self, msg: str) -> None:
        """Publish a status string to /pick_cube/status and log it."""
        self._step += 1
        text = f"[{self._step}] {msg}"
        self.get_logger().info(text)
        m = String()
        m.data = text
        self._status_pub.publish(m)

    # -- Controllers --

    def wait_for_controllers(self, timeout: float = 60.0) -> bool:
        """Block until both JTC and gripper action servers are reachable."""
        ok = self._jtc.wait_for_server(timeout_sec=timeout)
        ok = ok and self._grip.wait_for_server(timeout_sec=timeout)
        return ok

    # -- TF --

    def lookup_tag(self, frame: str, timeout: float = 10.0) -> Optional[PoseStamped]:
        try:
            ok = self.tf_buffer.can_transform(
                WORLD_FRAME, frame, rclpy.time.Time(),
                timeout=Duration(seconds=timeout),
            )
            if not ok:
                return None
            t = self.tf_buffer.lookup_transform(
                WORLD_FRAME, frame, rclpy.time.Time(),
            )
        except TransformException:
            return None

        p = PoseStamped()
        p.header = t.header
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        return p

    # -- Gripper (fire-and-forget) --

    def gripper(self, position: float, max_effort: float = 20.0,
                timeout: float = 5.0) -> bool:
        if not self._grip.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Gripper action server not available")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        deadline = time.monotonic() + timeout
        send_future = self._grip.send_goal_async(goal)
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            self.get_logger().error("Gripper send timed out")
            return False

        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        self.get_logger().info(
            f"Gripper goal accepted (target={position:.2f})")
        return True


# ── Arm motion wrapper ───────────────────────────────────────────────────
MOVE_SETTLE = 5.0  # seconds to let the arm reach the goal before next step
_jtc_goal_handle = None  # active JTC goal handle (if any)


def _cancel_jtc(helper: _Helper) -> None:
    """Cancel any in-flight JTC goal so the next one starts clean."""
    global _jtc_goal_handle
    if _jtc_goal_handle is None:
        return
    try:
        cancel_future = _jtc_goal_handle.cancel_goal_async()
        deadline = time.monotonic() + 3.0
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
    except Exception:
        pass
    _jtc_goal_handle = None


def _move(arm, target: PoseStamped, helper: _Helper, label: str) -> bool:
    global _jtc_goal_handle

    _cancel_jtc(helper)

    helper.status(f"{label}: planning …")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=target, pose_link=EE_LINK)
    result = arm.plan()
    if not result:
        helper.status(f"{label}: PLANNING FAILED")
        return False

    p = target.pose.position
    helper.status(
        f"{label}: executing → ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

    # Send the trajectory to the JTC ourselves instead of using
    # moveit.execute(), which hangs waiting for JTC feedback in Isaac Sim.
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = result.trajectory.get_robot_trajectory_msg().joint_trajectory

    send_future = helper._jtc.send_goal_async(goal)
    deadline = time.monotonic() + 5.0
    while not send_future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not send_future.done():
        helper.status(f"{label}: JTC send timed out")
        return False

    handle = send_future.result()
    if not handle.accepted:
        helper.status(f"{label}: JTC goal rejected")
        return False

    _jtc_goal_handle = handle
    helper.status(f"{label}: JTC goal accepted, waiting {MOVE_SETTLE}s …")
    time.sleep(MOVE_SETTLE)
    helper.status(f"{label}: done")
    return True


# ── Main pick sequence ───────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # Spin a lightweight helper node in the background so TF and the
    # gripper action client stay alive while MoveItPy does its thing.
    helper = _Helper()
    executor = SingleThreadedExecutor()
    executor.add_node(helper)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(2.0)

    moveit = MoveItPy(node_name="pick_cube_moveit")
    arm = moveit.get_planning_component(PLANNING_GROUP)

    helper.status("MoveItPy ready — waiting for controllers …")
    if not helper.wait_for_controllers():
        helper.status("FAIL: controller action servers not available")
        rclpy.shutdown()
        return
    helper.status("Controllers connected")

    # ── 1. Find the tag ──────────────────────────────────────────────────
    helper.status(f"Waiting for TF: {WORLD_FRAME} → {TAG_FRAME} …")
    tag = helper.lookup_tag(TAG_FRAME)
    if tag is None:
        helper.status(f"FAIL: {TAG_FRAME} not found")
        rclpy.shutdown()
        return

    tp = tag.pose.position
    to = tag.pose.orientation
    tag_pos: Vec3 = (tp.x, tp.y, tp.z)
    tag_q: Quat = (to.x, to.y, to.z, to.w)
    tag_x: Vec3 = _qrot(tag_q, (1.0, 0.0, 0.0))
    tag_z: Vec3 = _qrot(tag_q, (0.0, 0.0, 1.0))

    helper.status(
        f"Tag at ({tp.x:.3f}, {tp.y:.3f}, {tp.z:.3f})  "
        f"Z-axis = ({tag_z[0]:.3f}, {tag_z[1]:.3f}, {tag_z[2]:.3f})"
    )

    # End-effector orientation: we want the EE X axis (which points along the gripper)
    # to point TOWARD the tag surface (i.e. -Tag Z).
    # We can do this by rotating the tag frame by 90 degrees around its Y axis.
    # This makes EE X = -Tag Z, EE Y = Tag Y, EE Z = Tag X.
    # Quaternion for Ry(90): (0.0, 0.70710678, 0.0, 0.70710678)
    ee_q: Quat = _qnorm(_qmul(tag_q, (0.0, 0.70710678, 0.0, 0.70710678)))

    def _offset(base: Vec3, dist: float) -> Vec3:
        return (
            base[0] + dist * tag_z[0],
            base[1] + dist * tag_z[1],
            base[2] + dist * tag_z[2],
        )

    pre_grasp_pos = _offset(tag_pos, APPROACH_OFFSET)
    grasp_pos = _offset(tag_pos, GRASP_OFFSET)
    lift_pos = _offset(grasp_pos, LIFT_HEIGHT)

    hover_z = max(grasp_pos[2] + HOVER_CLEARANCE, pre_grasp_pos[2])
    hover_pos: Vec3 = (
        grasp_pos[0] + HOVER_X_RETREAT * tag_x[0],
        grasp_pos[1] + HOVER_X_RETREAT * tag_x[1],
        hover_z,
    )

    pre_grasp = _pose_stamped(pre_grasp_pos, ee_q)
    hover = _pose_stamped(hover_pos, ee_q)
    grasp = _pose_stamped(grasp_pos, ee_q)
    lift = _pose_stamped(lift_pos, ee_q)

    helper.status(
        f"Waypoints: pre-grasp=({pre_grasp_pos[0]:.3f}, {pre_grasp_pos[1]:.3f}, {pre_grasp_pos[2]:.3f}) "
        f"hover=({hover_pos[0]:.3f}, {hover_pos[1]:.3f}, {hover_pos[2]:.3f}) "
        f"grasp=({grasp_pos[0]:.3f}, {grasp_pos[1]:.3f}, {grasp_pos[2]:.3f}) "
        f"lift=({lift_pos[0]:.3f}, {lift_pos[1]:.3f}, {lift_pos[2]:.3f})"
    )

    # ── 2. Move to pre-grasp ─────────────────────────────────────────────
    if not _move(arm, pre_grasp, helper, "pre-grasp"):
        rclpy.shutdown()
        return

    # ── 3. Open gripper ──────────────────────────────────────────────────
    helper.status("Opening gripper …")
    helper.gripper(GRIPPER_OPEN)
    helper.status("Gripper open command sent")
    time.sleep(0.2)

    # ── 4. Hover directly above grasp (world-Z clearance) ───────────────
    if not _move(arm, hover, helper, "hover"):
        rclpy.shutdown()
        return

    # ── 5. Descend to grasp ──────────────────────────────────────────────
    if not _move(arm, grasp, helper, "grasp-approach"):
        rclpy.shutdown()
        return

    # ── 6. Close gripper ─────────────────────────────────────────────────
    helper.status("Closing gripper …")
    helper.gripper(GRIPPER_CLOSE, max_effort=GRIPPER_EFFORT)
    helper.status("Gripper close command sent")
    time.sleep(0.2)

    # ── 7. Lift ──────────────────────────────────────────────────────────
    if not _move(arm, lift, helper, "lift"):
        rclpy.shutdown()
        return

    helper.status("Pick complete!")
    time.sleep(1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
