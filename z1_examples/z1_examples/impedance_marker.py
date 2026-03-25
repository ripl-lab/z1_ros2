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
Interactive-marker node for the Cartesian impedance controller.

Spawns a 6-DOF interactive marker in RViz whose pose is continuously
published to the controller's reference_pose topic.  The marker
initialises at the current end-effector position (read from TF) so there
is no jump on activation.

Usage:
    ros2 run z1_examples impedance_marker.py
    ros2 run z1_examples impedance_marker.py --ros-args -p controller:=my_controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers import InteractiveMarkerServer
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class ImpedanceMarkerNode(Node):

    def __init__(self):
        super().__init__("impedance_marker")

        self.declare_parameter("controller", "cartesian_impedance_controller")
        self.declare_parameter("ee_frame", "link06")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("publish_rate", 30.0)

        controller = self.get_parameter("controller").get_parameter_value().string_value
        self._ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        topic = f"/{controller}/reference_pose"
        self._pub = self.create_publisher(PoseStamped, topic, 10)
        self.get_logger().info(f"Publishing to {topic}")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._server = InteractiveMarkerServer(self, "impedance_target")

        self._latest_pose = PoseStamped()
        self._latest_pose.header.frame_id = self._base_frame
        self._latest_pose.pose.orientation.w = 1.0

        # Wait briefly for TF, then create the marker
        self._init_timer = self.create_timer(0.5, self._try_init)
        self._init_attempts = 0

        self._publish_timer = self.create_timer(1.0 / rate, self._publish_pose)

    def _try_init(self):
        """Try to read the current EE pose from TF to initialise the marker."""
        self._init_attempts += 1
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame, rclpy.time.Time(), Duration(seconds=0.5)
            )
            p = t.transform.translation
            r = t.transform.rotation
            self._latest_pose.pose.position = Point(x=p.x, y=p.y, z=p.z)
            self._latest_pose.pose.orientation = Quaternion(x=r.x, y=r.y, z=r.z, w=r.w)
            self.get_logger().info(
                f"EE at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) — creating marker"
            )
        except Exception:
            if self._init_attempts < 10:
                self.get_logger().info("Waiting for TF...")
                return
            self.get_logger().warn(
                f"Could not look up {self._ee_frame} in {self._base_frame} "
                "— using default pose"
            )

        self._init_timer.cancel()
        self._create_marker()

    def _create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._base_frame
        int_marker.name = "impedance_target"
        int_marker.description = "Impedance target"
        int_marker.scale = 0.12
        int_marker.pose = self._latest_pose.pose

        # Translucent sphere to show the target
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
        sphere.color.r = 0.2
        sphere.color.g = 0.6
        sphere.color.b = 1.0
        sphere.color.a = 0.6

        vis_control = InteractiveMarkerControl()
        vis_control.always_visible = True
        vis_control.markers.append(sphere)
        int_marker.controls.append(vis_control)

        # 6-DOF controls (3 translation + 3 rotation rings)
        for axis, vec in [("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))]:
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.orientation.w = 1.0
            move.orientation.x = float(vec[0])
            move.orientation.y = float(vec[1])
            move.orientation.z = float(vec[2])
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(move)

            rot = InteractiveMarkerControl()
            rot.name = f"rotate_{axis}"
            rot.orientation.w = 1.0
            rot.orientation.x = float(vec[0])
            rot.orientation.y = float(vec[1])
            rot.orientation.z = float(vec[2])
            rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(rot)

        self._server.insert(int_marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()
        self.get_logger().info("Interactive marker ready — drag it in RViz")

    def _on_feedback(self, feedback):
        self._latest_pose.pose = feedback.pose

    def _publish_pose(self):
        self._latest_pose.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._latest_pose)


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
