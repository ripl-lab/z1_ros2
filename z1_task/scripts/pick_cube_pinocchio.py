#!/usr/bin/env python3
"""
Pinocchio pick-and-place for the Z1 arm.

No MoveIt.  No MTC.  No JTC action server.

Uses Pinocchio for inverse kinematics and publishes joint positions
directly to position_controller (JointGroupPositionController).
Gripper is driven via the existing GripperActionController.

IK strategy: position-only from many random starts, then picks the
solution whose EE orientation is closest to the desired one.  The Z1's
joint limits prevent exact orientation matching in many configurations,
so this best-effort approach is more reliable than trying to enforce
a 6-DOF pose the arm physically cannot reach.

Sequence:
  1. Open gripper
  2. Move to approach pose (offset from tag)
  3. Descend to grasp
  4. Close gripper
  5. Lift straight up
  6. Move above place location
  7. Lower
  8. Open gripper
  9. Retreat upward
 10. Return home

Prerequisites:
  - position_controller + gripper_controller active
  - joint_state_broadcaster active
  - apriltag_ros publishing TF for the target tag

Usage:
  ros2 launch z1_task pick_cube_pinocchio.launch.py
"""

import os
import time

import numpy as np
import pinocchio as pin
import rclpy
import tf2_ros
import xacro
from ament_index_python.packages import get_package_share_path
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_OPEN = -1.0
GRIPPER_CLOSED = 0.0
HOME = np.zeros(6)


def _ry(angle):
    """Rotation matrix about Y axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Z1PickPlace(Node):
    def __init__(self):
        super().__init__("z1_pick_place_pin")

        # ── parameters ───────────────────────────────────────────────────
        self.declare_parameter("tag_frame", "apriltag_0")
        self.declare_parameter("approach_offset_x", 0.0)
        self.declare_parameter("approach_offset_y", 0.0)
        self.declare_parameter("approach_offset_z", 0.4)
        self.declare_parameter("grasp_offset", 0.135)
        self.declare_parameter("lift_height", 0.30)
        self.declare_parameter("place_offset_x", 0.0)
        self.declare_parameter("place_offset_y", 0.15)
        self.declare_parameter("place_lower_distance", 0.10)
        self.declare_parameter("move_duration", 3.0)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("gripper_max_effort", 20.0)
        self.declare_parameter("ik_starts", 80)

        # ── pinocchio model ──────────────────────────────────────────────
        urdf_path = os.path.join(
            str(get_package_share_path("z1_description")),
            "urdf",
            "z1.urdf.xacro",
        )
        urdf_xml = xacro.process_file(
            urdf_path,
            mappings={
                "sim_ignition": "false",
                "sim_isaac": "false",
                "with_gripper": "true",
            },
        ).toxml()

        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()

        self.ee_id = self.model.getFrameId("link06")
        assert self.ee_id < self.model.nframes, "link06 not found in URDF"

        # link06 -> gripperTip (0.051 stator + 0.099 tip along local X)
        self._tip_offset = np.array([0.15, 0.0, 0.0])
        self.T_ee_tip = pin.SE3.Identity()
        self.T_ee_tip.translation = self._tip_offset.copy()

        # Map joint names -> pinocchio q/v indices (arm only)
        self._q_idx = []
        self._v_idx = []
        for name in ARM_JOINTS:
            jid = self.model.getJointId(name)
            assert jid < self.model.njoints, f"{name} not in model"
            self._q_idx.append(self.model.joints[jid].idx_q)
            self._v_idx.append(self.model.joints[jid].idx_v)

        self.q_lo = np.array(
            [self.model.lowerPositionLimit[i] for i in self._q_idx]
        )
        self.q_hi = np.array(
            [self.model.upperPositionLimit[i] for i in self._q_idx]
        )

        # ── ROS I/O ─────────────────────────────────────────────────────
        self.arm_pub = self.create_publisher(
            Float64MultiArray, "/position_controller/commands", 10
        )
        self.grip_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        self._q_arm = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf, self)

    # ── joint state ──────────────────────────────────────────────────────

    def _js_cb(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if all(j in lookup for j in ARM_JOINTS):
            self._q_arm = np.array([lookup[j] for j in ARM_JOINTS])

    def _wait_joints(self, timeout=10.0):
        t0 = time.time()
        while self._q_arm is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._q_arm is None:
            raise RuntimeError("No /joint_states within timeout")
        return self._q_arm.copy()

    # ── pinocchio helpers ────────────────────────────────────────────────

    def _full_q(self, q_arm):
        q = pin.neutral(self.model)
        for i, idx in enumerate(self._q_idx):
            q[idx] = q_arm[i]
        return q

    def _extract_arm(self, q):
        return np.array([q[i] for i in self._q_idx])

    def _fk(self, q_arm):
        """Return (ee_SE3, tip_position) for a given arm configuration."""
        q = self._full_q(q_arm)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMi = self.data.oMf[self.ee_id]
        tip = oMi.act(self._tip_offset)
        return oMi, tip

    def _tip_fk(self, q_arm):
        """FK -> SE3 of the gripper tip in world frame."""
        oMi, _ = self._fk(q_arm)
        return oMi * self.T_ee_tip

    # ── inverse kinematics ───────────────────────────────────────────────

    def _ik_pos(self, tip_target, q_init, max_iter=500, eps=5e-4):
        """
        Position-only IK for the gripper tip.
        Uses the world-aligned Jacobian with tip-offset correction.
        Returns (q_arm, pos_err) or (None, pos_err).
        """
        q = self._full_q(q_init)

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMi = self.data.oMf[self.ee_id]
            tip = oMi.act(self._tip_offset)
            err = tip_target - tip
            enorm = np.linalg.norm(err)

            if enorm < eps:
                qa = self._extract_arm(q)
                if np.all(qa >= self.q_lo - 0.01) and np.all(
                    qa <= self.q_hi + 0.01
                ):
                    return np.clip(qa, self.q_lo, self.q_hi), enorm
                return None, enorm

            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id, pin.LOCAL_WORLD_ALIGNED
            )
            Ja = J[:, self._v_idx]
            r = oMi.rotation @ self._tip_offset
            J_tip = Ja[:3, :] + pin.skew(r) @ Ja[3:, :]

            damp = max(1e-8, 1e-4 * enorm)
            dq = J_tip.T @ np.linalg.solve(
                J_tip @ J_tip.T + damp * np.eye(3), err
            )

            vnorm = np.linalg.norm(dq)
            if vnorm > 0.3:
                dq *= 0.3 / vnorm

            for i, idx in enumerate(self._q_idx):
                q[idx] = np.clip(
                    q[idx] + dq[i], self.q_lo[i], self.q_hi[i]
                )

        return None, enorm

    def solve_ik(self, target_tip, q_init=None):
        """
        Multi-start position IK, picks the solution whose achieved EE
        orientation is closest to the desired one.

        target_tip: desired SE3 of the gripper tip in world.
        Returns 6-element ndarray or None.
        """
        tip_pos = target_tip.translation
        oMdes = target_tip * self.T_ee_tip.inverse()
        n_starts = self.get_parameter("ik_starts").value

        solutions = []

        for trial in range(n_starts):
            if trial == 0 and q_init is not None:
                q0 = q_init
            else:
                q0 = self.q_lo + np.random.random(6) * (
                    self.q_hi - self.q_lo
                )

            qa, perr = self._ik_pos(tip_pos, q0)
            if qa is None:
                continue

            oMi, _ = self._fk(qa)
            rot_err = np.linalg.norm(
                pin.log3(oMdes.rotation @ oMi.rotation.T)
            )
            solutions.append((qa, perr, rot_err))

            if rot_err < 0.01:
                break

        if not solutions:
            self.get_logger().error(
                f"IK: no position solution in {n_starts} attempts "
                f"(tip target unreachable?)"
            )
            return None

        solutions.sort(key=lambda x: x[2])
        best_q, best_perr, best_rerr = solutions[0]

        oMi, tip = self._fk(best_q)
        ee_x = oMi.rotation[:, 0]
        des_x = target_tip.rotation[:, 0]
        angle_deg = np.degrees(
            np.arccos(np.clip(np.dot(ee_x, des_x), -1, 1))
        )

        self.get_logger().info(
            f"IK solved: pos_err={best_perr*1000:.1f}mm  "
            f"orient_err={angle_deg:.1f}deg  "
            f"({len(solutions)}/{n_starts} feasible)"
        )

        if angle_deg > 60.0:
            self.get_logger().warn(
                f"Large orientation error ({angle_deg:.0f} deg) — "
                f"arm joint limits prevent better alignment"
            )

        return best_q

    # ── motion primitives ────────────────────────────────────────────────

    def move_joints(self, q_target, duration=None):
        """Cosine-interpolated move via position_controller topic."""
        if duration is None:
            duration = self.get_parameter("move_duration").value
        hz = self.get_parameter("rate_hz").value
        steps = max(int(duration * hz), 1)
        dt = 1.0 / hz

        q0 = self._wait_joints()
        self.get_logger().info(
            f"joints -> [{', '.join(f'{x:.2f}' for x in q_target)}] "
            f"({duration:.1f}s)"
        )

        msg = Float64MultiArray()
        for i in range(steps + 1):
            alpha = 0.5 * (1.0 - np.cos(np.pi * i / steps))
            msg.data = (q0 + alpha * (q_target - q0)).tolist()
            self.arm_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        msg.data = q_target.tolist()
        self.arm_pub.publish(msg)
        time.sleep(0.3)

    def move_cartesian(self, direction, distance, duration=2.0):
        """Straight-line tip motion in world frame, keeping current orientation."""
        q0 = self._wait_joints()
        tip0 = self._tip_fk(q0)
        target = pin.SE3(
            tip0.rotation, tip0.translation + direction * distance
        )

        q_goal = self.solve_ik(target, q_init=q0)
        if q_goal is None:
            raise RuntimeError(
                f"Cartesian IK failed (dir={direction}, dist={distance:.3f})"
            )
        self.move_joints(q_goal, duration=duration)
        return q_goal

    # ── gripper ──────────────────────────────────────────────────────────

    def gripper(self, position):
        effort = self.get_parameter("gripper_max_effort").value
        label = "open" if position < -0.5 else "closed"
        self.get_logger().info(f"gripper -> {label}")

        if not self.grip_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Gripper action server unavailable")

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort

        future = self.grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError("Gripper goal rejected")
        time.sleep(1.0)

    # ── AprilTag TF lookup ───────────────────────────────────────────────

    def lookup_tag(self, timeout=20.0):
        tag = self.get_parameter("tag_frame").value
        self.get_logger().info(f"Waiting for TF: world -> {tag} ...")

        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                if self.tf_buf.can_transform("world", tag, rclpy.time.Time()):
                    tf = self.tf_buf.lookup_transform(
                        "world", tag, rclpy.time.Time()
                    )
                    t = tf.transform.translation
                    r = tf.transform.rotation
                    R = pin.Quaternion(r.w, r.x, r.y, r.z).matrix()
                    return pin.SE3(R, np.array([t.x, t.y, t.z]))
            except tf2_ros.TransformException:
                pass

        raise RuntimeError(f"TF '{tag}' not found within {timeout}s")

    # ── main sequence ────────────────────────────────────────────────────

    def run(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        tag = self.lookup_tag()
        tag_pos = tag.translation
        tag_x = tag.rotation[:, 0]
        tag_y = tag.rotation[:, 1]
        tag_z = tag.rotation[:, 2]

        # EE orientation: rotate tag frame 90 deg about local Y so that
        # gripper X (extension axis) points into -tag_Z (toward the surface).
        ee_R = tag.rotation @ _ry(np.pi / 2)

        approach_pos = (
            tag_pos
            + p("approach_offset_x") * tag_x
            + p("approach_offset_y") * tag_y
            + p("approach_offset_z") * tag_z
        )
        grasp_pos = tag_pos + p("grasp_offset") * tag_z

        descent = grasp_pos - approach_pos
        desc_dist = np.linalg.norm(descent)
        desc_dir = descent / desc_dist

        place_pos = np.array(
            [
                tag_pos[0] + p("place_offset_x"),
                tag_pos[1] + p("place_offset_y"),
                approach_pos[2],
            ]
        )

        self.get_logger().info(
            f"Tag       ({tag_pos[0]:.3f}, {tag_pos[1]:.3f}, {tag_pos[2]:.3f})"
        )
        self.get_logger().info(
            f"Approach  ({approach_pos[0]:.3f}, {approach_pos[1]:.3f}, "
            f"{approach_pos[2]:.3f})"
        )
        self.get_logger().info(
            f"Grasp     ({grasp_pos[0]:.3f}, {grasp_pos[1]:.3f}, "
            f"{grasp_pos[2]:.3f})"
        )
        self.get_logger().info(
            f"Place     ({place_pos[0]:.3f}, {place_pos[1]:.3f}, "
            f"{place_pos[2]:.3f})"
        )

        self._wait_joints()

        # ── PICK ─────────────────────────────────────────────────────────
        self.get_logger().info("=== PICK ===")
        self.gripper(GRIPPER_OPEN)

        q_approach = self.solve_ik(pin.SE3(ee_R, approach_pos))
        if q_approach is None:
            raise RuntimeError("IK failed for approach pose")
        self.move_joints(q_approach, 4.0)

        self.move_cartesian(desc_dir, desc_dist, 3.0)
        self.gripper(GRIPPER_CLOSED)
        self.move_cartesian(np.array([0.0, 0.0, 1.0]), p("lift_height"), 3.0)

        # ── PLACE ────────────────────────────────────────────────────────
        self.get_logger().info("=== PLACE ===")
        q_place = self.solve_ik(pin.SE3(ee_R, place_pos))
        if q_place is None:
            raise RuntimeError("IK failed for place pose")
        self.move_joints(q_place, 4.0)

        self.move_cartesian(
            np.array([0.0, 0.0, -1.0]), p("place_lower_distance"), 2.0
        )
        self.gripper(GRIPPER_OPEN)
        self.move_cartesian(
            np.array([0.0, 0.0, 1.0]), p("place_lower_distance"), 2.0
        )

        # ── HOME ─────────────────────────────────────────────────────────
        self.get_logger().info("=== HOME ===")
        self.move_joints(HOME, 4.0)
        self.get_logger().info("Pick and place complete!")


def main(args=None):
    rclpy.init(args=args)
    node = Z1PickPlace()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
