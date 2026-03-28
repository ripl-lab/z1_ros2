#!/usr/bin/env python3

import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w


def q_normalize(q: Quat) -> Quat:
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def q_conj(q: Quat) -> Quat:
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    qn = q_normalize(q)
    vx, vy, vz = v
    vq: Quat = (vx, vy, vz, 0.0)
    rq = q_mul(q_mul(qn, vq), q_conj(qn))
    return (rq[0], rq[1], rq[2])


def tf_from_msg(msg) -> Tuple[Vec3, Quat]:
    t = (
        float(msg.translation.x),
        float(msg.translation.y),
        float(msg.translation.z),
    )
    q = q_normalize((
        float(msg.rotation.x),
        float(msg.rotation.y),
        float(msg.rotation.z),
        float(msg.rotation.w),
    ))
    return t, q


def invert_tf(t: Vec3, q: Quat) -> Tuple[Vec3, Quat]:
    q_inv = q_conj(q_normalize(q))
    t_inv = q_rotate(q_inv, (-t[0], -t[1], -t[2]))
    return t_inv, q_inv


def compose_tf(t1: Vec3, q1: Quat, t2: Vec3, q2: Quat) -> Tuple[Vec3, Quat]:
    # T = T1 * T2
    t2_in_1 = q_rotate(q1, t2)
    t = (
        t1[0] + t2_in_1[0],
        t1[1] + t2_in_1[1],
        t1[2] + t2_in_1[2],
    )
    q = q_normalize(q_mul(q1, q2))
    return t, q


class AprilTagLocalizer(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_localizer')

        self.world_frame = self.declare_parameter(
            'world_frame', 'world').value
        self.landmark_frame = self.declare_parameter(
            'landmark_frame', 'apriltag_69_landmark').value
        self.observed_tag_frame = self.declare_parameter(
            'observed_tag_frame', 'apriltag_69_obs').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'camera_optical').value
        publish_child = self.declare_parameter(
            'publish_child_frame', '').value
        self.publish_child_frame = (
            publish_child if publish_child else self.camera_frame)
        self.rate_hz = float(self.declare_parameter(
            'publish_rate_hz', 30.0).value)
        self._tf_warn_period = float(self.declare_parameter(
            'tf_warn_throttle_sec', 2.0).value)
        self._publish_info_period = float(self.declare_parameter(
            'publish_info_throttle_sec', 2.0).value)

        self._last_tf_warn_m = 0.0
        self._last_publish_info_m = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / max(self.rate_hz, 1e-3)
        self.timer = self.create_timer(period, self.tick)

        log = self.get_logger()
        log.info(
            f'Localizing child={self.publish_child_frame!r} in parent={self.world_frame!r}; '
            f'landmark={self.landmark_frame!r}, '
            f'camera->tag observation: {self.camera_frame!r} <- {self.observed_tag_frame!r}'
        )
        log.info(
            f'Expect static TF: {self.world_frame!r} -> {self.landmark_frame!r}; '
            f'expect apriltag_ros TF: {self.camera_frame!r} -> {self.observed_tag_frame!r}; '
            f'this node publishes: {self.world_frame!r} -> {self.publish_child_frame!r}'
        )

    def _throttled_warn(self, message: str) -> None:
        now_m = time.monotonic()
        if now_m - self._last_tf_warn_m < self._tf_warn_period:
            return
        self._last_tf_warn_m = now_m
        self.get_logger().warning(message)

    def tick(self) -> None:
        now = rclpy.time.Time()
        try:
            # ^world T_landmark
            world_from_landmark = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.landmark_frame,
                now
            )
        except TransformException as ex:
            self._throttled_warn(
                f'TF lookup failed ({self.world_frame!r} <- {self.landmark_frame!r}): {ex}'
            )
            return

        try:
            # ^camera T_obs
            camera_from_obs = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.observed_tag_frame,
                now
            )
        except TransformException as ex:
            self._throttled_warn(
                f'TF lookup failed ({self.camera_frame!r} <- {self.observed_tag_frame!r}): {ex}'
            )
            return

        try:
            t_w_lmk, q_w_lmk = tf_from_msg(world_from_landmark.transform)
            t_cam_obs, q_cam_obs = tf_from_msg(camera_from_obs.transform)

            # ^obs T_camera = inverse(^camera T_obs)
            t_obs_cam, q_obs_cam = invert_tf(t_cam_obs, q_cam_obs)

            # Assume landmark frame and obs frame are the same physical tag frame
            # so ^world T_camera = ^world T_landmark * ^obs T_camera
            t_world_child, q_world_child = compose_tf(
                t_w_lmk, q_w_lmk,
                t_obs_cam, q_obs_cam
            )

            # Optional: publish some other child attached below camera_frame,
            # e.g. publish_child_frame=base_link if ^camera T_base_link is known.
            if self.publish_child_frame != self.camera_frame:
                camera_from_child = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.publish_child_frame,
                    now
                )
                t_cam_child, q_cam_child = tf_from_msg(camera_from_child.transform)
                t_world_child, q_world_child = compose_tf(
                    t_world_child, q_world_child,
                    t_cam_child, q_cam_child
                )

            out = TransformStamped()
            stamp = camera_from_obs.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = self.get_clock().now().to_msg()

            out.header.stamp = stamp
            out.header.frame_id = self.world_frame
            out.child_frame_id = self.publish_child_frame
            out.transform.translation.x = t_world_child[0]
            out.transform.translation.y = t_world_child[1]
            out.transform.translation.z = t_world_child[2]
            out.transform.rotation.x = q_world_child[0]
            out.transform.rotation.y = q_world_child[1]
            out.transform.rotation.z = q_world_child[2]
            out.transform.rotation.w = q_world_child[3]

            self.tf_broadcaster.sendTransform(out)

            tlog = time.monotonic()
            if tlog - self._last_publish_info_m >= self._publish_info_period:
                self._last_publish_info_m = tlog
                self.get_logger().info(
                    f'Published TF {self.world_frame!r} -> {self.publish_child_frame!r} '
                    f't=({t_world_child[0]:.4f}, {t_world_child[1]:.4f}, {t_world_child[2]:.4f}) '
                    f'q=({q_world_child[0]:.4f}, {q_world_child[1]:.4f}, '
                    f'{q_world_child[2]:.4f}, {q_world_child[3]:.4f})'
                )

        except TransformException as ex:
            self._throttled_warn(f'TF lookup failed (extra chain step): {ex}')


def main() -> None:
    rclpy.init()
    node = AprilTagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
