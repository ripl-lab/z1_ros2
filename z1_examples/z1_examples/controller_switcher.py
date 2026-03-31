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
Controller switcher with an RViz right-click menu.

Spawns a small marker near the robot base.  Right-click it in RViz to
choose which ros2_control controller should be active.  The node calls
``controller_manager/switch_controller`` and publishes the active
controller name on ``/active_controller`` so other nodes (e.g. the
impedance marker) can react.

Usage:
    ros2 run z1_examples controller_switcher.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)
from interactive_markers import InteractiveMarkerServer, MenuHandler
from controller_manager_msgs.srv import SwitchController, ListControllers


SWITCHABLE = [
    ("joint_trajectory_controller", "Joint Trajectory (MoveIt)"),
    ("cartesian_impedance_controller", "Cartesian Impedance"),
    ("operational_impedance_controller", "Operational Impedance"),
    ("position_controller", "Joint Position"),
    ("torque_controller", "Joint Effort"),
]


class ControllerSwitcherNode(Node):

    def __init__(self):
        super().__init__("controller_switcher")

        self._active_pub = self.create_publisher(String, "/active_controller", 10)

        self._switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._list_cli = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        self._active_controller = ""
        self._switching = False
        self._menu_ids: dict[int, str] = {}

        self._server = InteractiveMarkerServer(self, "controller_switcher")
        self._menu = MenuHandler()
        for ctrl_name, label in SWITCHABLE:
            eid = self._menu.insert(label, callback=self._on_menu_select)
            self._menu_ids[eid] = ctrl_name

        self._build_marker()

        self._poll_timer = self.create_timer(1.0, self._poll_controllers)

    # ── InteractiveMarker ────────────────────────────────────────────────

    def _build_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = "world"
        im.name = "controller_selector"
        im.description = "Right-click: switch controller"
        im.scale = 0.08
        im.pose.position.x = 0.0
        im.pose.position.y = -0.15
        im.pose.position.z = 0.45

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.035
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.9, 0.7, 0.1, 0.9

        ctrl = InteractiveMarkerControl()
        ctrl.always_visible = True
        ctrl.interaction_mode = InteractiveMarkerControl.MENU
        ctrl.markers.append(cube)
        im.controls.append(ctrl)

        self._server.insert(im)
        self._menu.apply(self._server, im.name)
        self._server.applyChanges()
        self.get_logger().info(
            "Controller switcher marker ready — right-click it in RViz"
        )

    # ── Menu callback ────────────────────────────────────────────────────

    def _on_menu_select(self, feedback):
        target = self._menu_ids.get(feedback.menu_entry_id)
        if not target or target == self._active_controller or self._switching:
            return
        self.get_logger().info(f"Requesting switch → {target}")
        self._request_switch(target)

    # ── Switch service call ──────────────────────────────────────────────

    def _request_switch(self, target: str):
        if not self._switch_cli.service_is_ready():
            self.get_logger().warn("switch_controller service not available yet")
            return

        self._switching = True
        req = SwitchController.Request()
        req.activate_controllers = [target]
        if self._active_controller:
            req.deactivate_controllers = [self._active_controller]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        future = self._switch_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_switch_result(f, target))

    def _on_switch_result(self, future, target: str):
        self._switching = False
        try:
            resp = future.result()
            if resp.ok:
                self.get_logger().info(f"Switched to {target}")
                self._active_controller = target
                self._publish_active()
                self._refresh_checks()
            else:
                self.get_logger().error(f"Switch to {target} rejected")
        except Exception as exc:
            self.get_logger().error(f"Switch service error: {exc}")

    # ── Poll controller states ───────────────────────────────────────────

    def _poll_controllers(self):
        if not self._list_cli.service_is_ready():
            return
        future = self._list_cli.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_result)

    def _on_list_result(self, future):
        try:
            resp = future.result()
        except Exception:
            return
        switchable_names = {n for n, _ in SWITCHABLE}
        for ctrl in resp.controller:
            if ctrl.name in switchable_names and ctrl.state == "active":
                if ctrl.name != self._active_controller:
                    self._active_controller = ctrl.name
                    self._publish_active()
                    self._refresh_checks()
                return

    # ── Helpers ──────────────────────────────────────────────────────────

    def _publish_active(self):
        msg = String(data=self._active_controller)
        self._active_pub.publish(msg)

    def _refresh_checks(self):
        for eid, name in self._menu_ids.items():
            state = (
                MenuHandler.CHECKED
                if name == self._active_controller
                else MenuHandler.UNCHECKED
            )
            self._menu.setCheckState(eid, state)
        self._menu.reApply(self._server)
        self._server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
