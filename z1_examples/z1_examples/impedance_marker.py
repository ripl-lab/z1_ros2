#!/usr/bin/env python3

# Copyright 2025 IDRA, University of Trento
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Interactive-marker node for the impedance controllers.

When the ``/active_controller`` topic reports an impedance controller the
marker appears at the current end-effector pose and continuously publishes
to that controller's ``reference_pose`` topic.  When a non-impedance
controller becomes active the marker is hidden and publishing stops.

Set ``standalone:=true`` to skip the ``/active_controller`` handshake and
always publish (legacy single-controller mode).

Usage (unified launch — recommended):
    Launched automatically by z1_unified.launch.py

Usage (standalone):
    ros2 run z1_examples impedance_marker.py \\
        --ros-args -p standalone:=true -p controller:=cartesian_impedance_controller
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)
from interactive_markers import InteractiveMarkerServer
from tf2_ros import Buffer, TransformListener


IMPEDANCE_CONTROLLERS = {
    "cartesian_impedance_controller",
    "operational_impedance_controller",
}

MARKER_NAME = "impedance_target"


class ImpedanceMarkerNode(Node):

    def __init__(self):
        super().__init__("impedance_marker")

        self.declare_parameter("standalone", False)
        self.declare_parameter("controller", "cartesian_impedance_controller")
        self.declare_parameter("ee_frame", "link06")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("publish_rate", 30.0)

        self._standalone = (
            self.get_parameter("standalone").get_parameter_value().bool_value
        )
        self._ee_frame = (
            self.get_parameter("ee_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._server = InteractiveMarkerServer(self, "impedance_target")

        self._latest_pose = PoseStamped()
        self._latest_pose.header.frame_id = self._base_frame
        self._latest_pose.pose.orientation.w = 1.0

        self._pub = None
        self._enabled = False
        self._marker_visible = False
        self._active_controller = ""

        if self._standalone:
            ctrl = (
                self.get_parameter("controller")
                .get_parameter_value()
                .string_value
            )
            self._active_controller = ctrl
            self._init_attempts = 0
            self._init_timer = self.create_timer(0.5, self._try_init_standalone)
        else:
            self.create_subscription(
                String, "/active_controller", self._on_active_controller, 10
            )
            self.get_logger().info(
                "Impedance marker waiting for /active_controller ..."
            )

        self._publish_timer = self.create_timer(1.0 / rate, self._tick_publish)

    # ── Standalone init (legacy) ─────────────────────────────────────────

    def _try_init_standalone(self):
        self._init_attempts += 1
        if self._pub is None:
            topic = f"/{self._active_controller}/reference_pose"
            self._pub = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing to {topic}")

        if self._read_ee_pose() or self._init_attempts >= 10:
            if self._init_attempts >= 10:
                self.get_logger().warn(
                    "Could not look up EE pose — using default"
                )
            self._init_timer.cancel()
            self._show_marker()
            self._enabled = True

    # ── Managed mode (via /active_controller) ────────────────────────────

    def _on_active_controller(self, msg: String):
        new_ctrl = msg.data
        is_impedance = new_ctrl in IMPEDANCE_CONTROLLERS

        if is_impedance and (
            not self._enabled or new_ctrl != self._active_controller
        ):
            self._active_controller = new_ctrl
            self._enable()
        elif not is_impedance and self._enabled:
            self._disable()

    def _enable(self):
        if self._pub is not None:
            self.destroy_publisher(self._pub)
        topic = f"/{self._active_controller}/reference_pose"
        self._pub = self.create_publisher(PoseStamped, topic, 10)
        self.get_logger().info(f"Enabled — publishing to {topic}")

        self._read_ee_pose()
        self._show_marker()
        self._enabled = True

    def _disable(self):
        self._enabled = False
        self._active_controller = ""
        self._hide_marker()
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        self.get_logger().info("Disabled — marker hidden")

    # ── Marker lifecycle ─────────────────────────────────────────────────

    def _show_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._base_frame
        int_marker.name = MARKER_NAME
        int_marker.description = "Impedance target"
        int_marker.scale = 0.12
        int_marker.pose = self._latest_pose.pose

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.r, sphere.color.g = 0.2, 0.6
        sphere.color.b, sphere.color.a = 1.0, 0.6

        vis = InteractiveMarkerControl()
        vis.always_visible = True
        vis.markers.append(sphere)
        int_marker.controls.append(vis)

        for axis, vec in [("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))]:
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.orientation.w = 1.0
            move.orientation.x, move.orientation.y, move.orientation.z = (
                float(v) for v in vec
            )
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(move)

            rot = InteractiveMarkerControl()
            rot.name = f"rotate_{axis}"
            rot.orientation.w = 1.0
            rot.orientation.x, rot.orientation.y, rot.orientation.z = (
                float(v) for v in vec
            )
            rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rot)

        self._server.insert(int_marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()
        self._marker_visible = True
        self.get_logger().info("Interactive marker visible — drag it in RViz")

    def _hide_marker(self):
        if self._marker_visible:
            self._server.erase(MARKER_NAME)
            self._server.applyChanges()
            self._marker_visible = False

    # ── Feedback / publish ───────────────────────────────────────────────

    def _on_feedback(self, feedback):
        self._latest_pose.pose = feedback.pose

    def _tick_publish(self):
        if not self._enabled or self._pub is None:
            return
        self._latest_pose.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._latest_pose)

    # ── TF helper ────────────────────────────────────────────────────────

    def _read_ee_pose(self) -> bool:
        try:
            t = self._tf_buf.lookup_transform(
                self._base_frame,
                self._ee_frame,
                rclpy.time.Time(),
                Duration(seconds=0.5),
            )
            p = t.transform.translation
            r = t.transform.rotation
            self._latest_pose.pose.position = Point(x=p.x, y=p.y, z=p.z)
            self._latest_pose.pose.orientation = Quaternion(
                x=r.x, y=r.y, z=r.z, w=r.w
            )
            self.get_logger().info(
                f"EE at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
            return True
        except Exception:
            self.get_logger().info("Waiting for TF ...")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
