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
Unified launch: bringup + MoveIt + controller switcher + impedance marker.

All switchable controllers are loaded (only the starting one is active).
A right-click interactive-marker menu in RViz lets you switch at runtime.

Usage:
    ros2 launch z1_bringup z1_unified.launch.py                     # Ignition sim
    ros2 launch z1_bringup z1_unified.launch.py sim_ignition:=false  # Real robot
    ros2 launch z1_bringup z1_unified.launch.py starting_controller:=cartesian_impedance_controller
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


ALL_SWITCHABLE = [
    "joint_trajectory_controller",
    "cartesian_impedance_controller",
    "operational_impedance_controller",
]


def launch_setup(context, *args, **kwargs):
    sim_ignition = LaunchConfiguration("sim_ignition").perform(context)
    sim_isaac = LaunchConfiguration("sim_isaac").perform(context)
    rviz_flag = LaunchConfiguration("rviz").perform(context)
    starting_controller = LaunchConfiguration("starting_controller").perform(context)
    with_gripper = LaunchConfiguration("with_gripper").perform(context)
    controller_config = LaunchConfiguration("controller_config").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    xacro_file = LaunchConfiguration("xacro_file").perform(context)

    if sim_isaac == "true":
        sim_ignition = "false"

    use_sim_time = sim_ignition == "true" or sim_isaac == "true"

    entities = []

    # ── 1. Bringup (hardware / sim  +  joint_state_broadcaster  +  starting controller) ──
    bringup_file = os.path.join(
        get_package_share_path("z1_bringup"), "launch", "z1.launch.py"
    )
    entities.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_file),
            launch_arguments={
                "sim_ignition": sim_ignition,
                "sim_isaac": sim_isaac,
                "rviz": "false",
                "starting_controller": starting_controller,
                "with_gripper": with_gripper,
                "controller_config": controller_config,
                "robot_name": robot_name,
                "xacro_file": xacro_file,
            }.items(),
        )
    )

    # ── 2. MoveIt move_group ─────────────────────────────────────────────
    moveit_config = MoveItConfigsBuilder(
        "z1_description", package_name="z1_moveit"
    ).to_moveit_configs()
    moveit_config.trajectory_execution["use_sim_time"] = use_sim_time
    entities.append(generate_move_group_launch(moveit_config))

    # ── 3. Load non-starting controllers as *inactive* ───────────────────
    for ctrl in ALL_SWITCHABLE:
        if ctrl == starting_controller:
            continue
        entities.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[ctrl, "--inactive", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    # ── 4. Controller-switcher (right-click menu in RViz) ────────────────
    entities.append(
        Node(
            package="z1_examples",
            executable="controller_switcher.py",
            name="controller_switcher",
            output="screen",
        )
    )

    # ── 5. Impedance marker (auto-enables when an impedance ctrl is active)
    entities.append(
        Node(
            package="z1_examples",
            executable="impedance_marker.py",
            name="impedance_marker",
            output="screen",
            parameters=[{"standalone": False}],
        )
    )

    # ── 6. RViz (MoveIt panel + interactive markers) ─────────────────────
    #   The MotionPlanning plugin needs kinematics, pipelines and limits
    #   as node parameters (same as generate_moveit_rviz_launch passes).
    rviz_config = os.path.join(
        get_package_share_path("z1_bringup"), "rviz", "z1_unified.rviz"
    )
    entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(rviz_flag),
        )
    )

    return entities


def generate_launch_description():
    xacro_file_default = os.path.join(
        get_package_share_path("z1_description"), "urdf", "z1.urdf.xacro"
    )
    controller_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "config", "z1_controllers.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "starting_controller",
            default_value="joint_trajectory_controller",
            description="Controller to activate at start",
        ),
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Launch in Ignition Gazebo?",
        ),
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Use Isaac Sim backend?",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "with_gripper",
            default_value="true",
            description="Use the gripper?",
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value=controller_config_default,
            description="Path to the controllers.yaml file",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="z1",
            description="Name of the robot",
        ),
        DeclareLaunchArgument(
            "xacro_file",
            default_value=xacro_file_default,
            description="Path to xacro file of the Z1 manipulator",
        ),
        OpaqueFunction(function=launch_setup),
    ])
