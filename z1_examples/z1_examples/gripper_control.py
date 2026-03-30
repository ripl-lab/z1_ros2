#!/usr/bin/env python3

"""
Gripper control GUI with position and effort (torque) sliders.

Provides a tkinter interface to switch between position control
(via gripper_controller / GripperActionController) and effort control
(via gripper_effort_controller / JointGroupEffortController) for the
Z1 gripper.

Position mode:
    The slider sends GripperCommand action goals.  The existing
    ``gripper_controller`` with ``allow_stalling: true`` will succeed
    even when the gripper stalls against an object.

Effort mode:
    The slider publishes a torque command to
    ``/gripper_effort_controller/commands``.  Positive values close,
    negative values open.  The command is published continuously at
    ~20 Hz so the controller always has a fresh reference.

Usage:
    ros2 run z1_examples gripper_control.py
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController

GRIPPER_JOINT = "jointGripper"
POS_CTRL = "gripper_controller"
EFF_CTRL = "gripper_effort_controller"

POS_MIN = -1.5708
POS_MAX = 0.0
EFF_MIN = -20.0
EFF_MAX = 20.0

BG = "#2b2b2b"
FG = "#e0e0e0"
ACCENT = "#4a9eff"
ACCENT_EFF = "#ff6b4a"
WIDGET_BG = "#3c3c3c"
DISABLED_FG = "#666666"


class GripperControlNode(Node):

    def __init__(self):
        super().__init__("gripper_control")
        self._pos = 0.0
        self._vel = 0.0
        self._eff = 0.0
        self._lock = threading.Lock()

        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10
        )
        self._action = ActionClient(
            self, GripperCommand, f"/{POS_CTRL}/gripper_cmd"
        )
        self._eff_pub = self.create_publisher(
            Float64MultiArray, f"/{EFF_CTRL}/commands", 10
        )
        self._switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

    def _js_cb(self, msg: JointState):
        try:
            i = msg.name.index(GRIPPER_JOINT)
        except ValueError:
            return
        with self._lock:
            self._pos = msg.position[i]
            self._vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self._eff = msg.effort[i] if i < len(msg.effort) else 0.0

    def state(self):
        with self._lock:
            return self._pos, self._vel, self._eff

    def send_pos(self, position: float, max_effort: float = 30.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        if self._action.server_is_ready():
            self._action.send_goal_async(goal)
        else:
            self.get_logger().warn("Gripper action server not ready")

    def send_eff(self, effort: float):
        self._eff_pub.publish(Float64MultiArray(data=[effort]))

    def switch(self, activate: str, deactivate: str, callback=None):
        if not self._switch_cli.service_is_ready():
            self.get_logger().warn("switch_controller service not available")
            if callback:
                callback(False)
            return
        req = SwitchController.Request()
        req.activate_controllers = [activate]
        req.deactivate_controllers = [deactivate]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        future = self._switch_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_switch(f, callback))

    def _on_switch(self, future, callback):
        try:
            ok = future.result().ok
        except Exception:
            ok = False
        if callback:
            callback(ok)


class GripperGUI:

    def __init__(self, node: GripperControlNode):
        self.node = node
        self.mode = "position"
        self._switching = False
        self._pos_debounce_id = None

        self.root = tk.Tk()
        self.root.title("Z1 Gripper Control")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self._configure_styles()
        self._build()
        self._poll_state()
        self._poll_effort()

    # ── Styles ────────────────────────────────────────────────────────────

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("Dark.TFrame", background=BG)
        style.configure(
            "Dark.TLabel", background=BG, foreground=FG,
            font=("sans-serif", 10),
        )
        style.configure(
            "Title.TLabel", background=BG, foreground=FG,
            font=("sans-serif", 14, "bold"),
        )
        style.configure(
            "Value.TLabel", background=BG, foreground=ACCENT,
            font=("monospace", 11, "bold"),
        )
        style.configure(
            "Mode.TLabel", background=BG, foreground=ACCENT,
            font=("sans-serif", 11, "bold"),
        )
        style.configure(
            "ModeEff.TLabel", background=BG, foreground=ACCENT_EFF,
            font=("sans-serif", 11, "bold"),
        )
        style.configure(
            "Switch.TButton",
            font=("sans-serif", 10, "bold"),
            padding=(12, 6),
        )

    # ── Layout ────────────────────────────────────────────────────────────

    def _build(self):
        pad = dict(padx=14, pady=4)
        main = ttk.Frame(self.root, style="Dark.TFrame")
        main.pack(fill="both", expand=True, padx=8, pady=8)

        # Title
        ttk.Label(
            main, text="Z1 Gripper Control", style="Title.TLabel",
        ).pack(**pad, pady=(8, 2))

        ttk.Separator(main).pack(fill="x", **pad)

        # ── Mode indicator + switch button ────────────────────────────────
        mode_frame = ttk.Frame(main, style="Dark.TFrame")
        mode_frame.pack(fill="x", **pad, pady=(6, 2))

        ttk.Label(
            mode_frame, text="Mode:", style="Dark.TLabel",
        ).pack(side="left")

        self._mode_label = ttk.Label(
            mode_frame, text="POSITION", style="Mode.TLabel",
        )
        self._mode_label.pack(side="left", padx=(6, 16))

        self._switch_btn = ttk.Button(
            mode_frame, text="Switch to Effort",
            style="Switch.TButton", command=self._on_mode_switch,
        )
        self._switch_btn.pack(side="right")

        ttk.Separator(main).pack(fill="x", **pad, pady=(6, 2))

        # ── Live state ────────────────────────────────────────────────────
        state_frame = ttk.Frame(main, style="Dark.TFrame")
        state_frame.pack(fill="x", **pad)

        ttk.Label(
            state_frame, text="Live State", style="Dark.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w")

        for i, (label, unit) in enumerate([
            ("Position", "rad"),
            ("Velocity", "rad/s"),
            ("Effort", "Nm"),
        ]):
            ttk.Label(
                state_frame, text=f"  {label}:", style="Dark.TLabel",
            ).grid(row=i + 1, column=0, sticky="w")

        self._pos_val = ttk.Label(
            state_frame, text="0.0000 rad", style="Value.TLabel",
        )
        self._vel_val = ttk.Label(
            state_frame, text="0.0000 rad/s", style="Value.TLabel",
        )
        self._eff_val = ttk.Label(
            state_frame, text="0.0000 Nm", style="Value.TLabel",
        )
        self._pos_val.grid(row=1, column=1, sticky="w", padx=(8, 0))
        self._vel_val.grid(row=2, column=1, sticky="w", padx=(8, 0))
        self._eff_val.grid(row=3, column=1, sticky="w", padx=(8, 0))

        ttk.Separator(main).pack(fill="x", **pad, pady=(6, 2))

        # ── Position slider ───────────────────────────────────────────────
        pos_frame = ttk.Frame(main, style="Dark.TFrame")
        pos_frame.pack(fill="x", **pad)

        ttk.Label(
            pos_frame, text="Position Command", style="Dark.TLabel",
        ).pack(anchor="w")

        self._pos_cmd_label = ttk.Label(
            pos_frame, text="0.0000 rad", style="Value.TLabel",
        )
        self._pos_cmd_label.pack(anchor="e")

        self._pos_slider = tk.Scale(
            pos_frame,
            from_=POS_MIN, to=POS_MAX,
            resolution=0.001,
            orient="horizontal",
            length=420,
            bg=WIDGET_BG, fg=FG,
            troughcolor="#555555",
            highlightbackground=BG,
            activebackground=ACCENT,
            font=("monospace", 9),
            command=self._on_pos_slider,
        )
        self._pos_slider.set(0.0)
        self._pos_slider.pack(fill="x")

        range_frame = ttk.Frame(pos_frame, style="Dark.TFrame")
        range_frame.pack(fill="x")
        ttk.Label(
            range_frame, text=f"{POS_MIN:.2f} (open)", style="Dark.TLabel",
        ).pack(side="left")
        ttk.Label(
            range_frame, text=f"{POS_MAX:.2f} (closed)", style="Dark.TLabel",
        ).pack(side="right")

        ttk.Separator(main).pack(fill="x", **pad, pady=(6, 2))

        # ── Effort slider ─────────────────────────────────────────────────
        eff_frame = ttk.Frame(main, style="Dark.TFrame")
        eff_frame.pack(fill="x", **pad)

        ttk.Label(
            eff_frame, text="Effort Command", style="Dark.TLabel",
        ).pack(anchor="w")

        self._eff_cmd_label = ttk.Label(
            eff_frame, text="0.0000 Nm", style="Value.TLabel",
        )
        self._eff_cmd_label.pack(anchor="e")

        self._eff_slider = tk.Scale(
            eff_frame,
            from_=EFF_MIN, to=EFF_MAX,
            resolution=0.1,
            orient="horizontal",
            length=420,
            bg=WIDGET_BG, fg=DISABLED_FG,
            troughcolor="#444444",
            highlightbackground=BG,
            activebackground=ACCENT_EFF,
            font=("monospace", 9),
            state="disabled",
            command=self._on_eff_slider,
        )
        self._eff_slider.set(0.0)
        self._eff_slider.pack(fill="x")

        range_frame2 = ttk.Frame(eff_frame, style="Dark.TFrame")
        range_frame2.pack(fill="x")
        ttk.Label(
            range_frame2, text=f"{EFF_MIN:.1f} Nm", style="Dark.TLabel",
        ).pack(side="left")
        ttk.Label(
            range_frame2, text=f"{EFF_MAX:.1f} Nm", style="Dark.TLabel",
        ).pack(side="right")

        # ── Bottom padding ────────────────────────────────────────────────
        ttk.Frame(main, style="Dark.TFrame", height=8).pack()

    # ── Slider callbacks ──────────────────────────────────────────────────

    def _on_pos_slider(self, val):
        self._pos_cmd_label.configure(text=f"{float(val):.4f} rad")
        if self.mode != "position":
            return
        if self._pos_debounce_id is not None:
            self.root.after_cancel(self._pos_debounce_id)
        self._pos_debounce_id = self.root.after(
            150, lambda: self._send_pos_goal(float(val))
        )

    def _send_pos_goal(self, val):
        self._pos_debounce_id = None
        self.node.send_pos(val)

    def _on_eff_slider(self, val):
        self._eff_cmd_label.configure(text=f"{float(val):.4f} Nm")

    # ── Mode switching ────────────────────────────────────────────────────

    def _on_mode_switch(self):
        if self._switching:
            return
        self._switching = True
        self._switch_btn.configure(state="disabled")

        if self.mode == "position":
            self.node.switch(
                EFF_CTRL, POS_CTRL,
                callback=lambda ok: self.root.after(
                    0, self._finish_switch, "effort", ok
                ),
            )
        else:
            self.node.switch(
                POS_CTRL, EFF_CTRL,
                callback=lambda ok: self.root.after(
                    0, self._finish_switch, "position", ok
                ),
            )

    def _finish_switch(self, new_mode: str, ok: bool):
        self._switching = False
        self._switch_btn.configure(state="normal")
        if not ok:
            self.node.get_logger().error("Controller switch failed")
            return

        self.mode = new_mode
        if self.mode == "position":
            self._mode_label.configure(text="POSITION", style="Mode.TLabel")
            self._switch_btn.configure(text="Switch to Effort")
            self._pos_slider.configure(state="normal", fg=FG)
            self._eff_slider.configure(state="disabled", fg=DISABLED_FG)
            # Send position goal at the current gripper position so
            # the position controller doesn't jump to the slider value
            pos, _, _ = self.node.state()
            self._pos_slider.set(pos)
            self.node.send_pos(pos)
        else:
            self._mode_label.configure(text="EFFORT", style="ModeEff.TLabel")
            self._switch_btn.configure(text="Switch to Position")
            self._pos_slider.configure(state="disabled", fg=DISABLED_FG)
            self._eff_slider.configure(state="normal", fg=FG)
            self._eff_slider.set(0.0)
            self.node.send_eff(0.0)

    # ── Periodic updates ──────────────────────────────────────────────────

    def _poll_state(self):
        pos, vel, eff = self.node.state()
        self._pos_val.configure(text=f"{pos:+.4f} rad")
        self._vel_val.configure(text=f"{vel:+.4f} rad/s")
        self._eff_val.configure(text=f"{eff:+.4f} Nm")
        self.root.after(50, self._poll_state)

    def _poll_effort(self):
        if self.mode == "effort":
            self.node.send_eff(self._eff_slider.get())
        self.root.after(50, self._poll_effort)

    # ── Run / teardown ────────────────────────────────────────────────────

    def run(self):
        try:
            self.root.mainloop()
        finally:
            if self.mode == "effort":
                self.node.send_eff(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = GripperGUI(node)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
