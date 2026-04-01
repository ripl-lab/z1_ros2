import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def launch_setup(context, *args, **kwargs):
    sim_ignition = LaunchConfiguration("sim_ignition").perform(context)
    sim_isaac = LaunchConfiguration("sim_isaac").perform(context)
    step_mode = LaunchConfiguration("step").perform(context) == "true"

    if sim_isaac == "true":
        sim_ignition = "false"

    is_real = sim_ignition == "false" and sim_isaac == "false"
    use_sim_time = not is_real

    nodes = []

    # ── Bringup (controllers, robot_state_publisher, sim) ────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                str(get_package_share_path("z1_bringup")),
                "launch",
                "z1.launch.py",
            )
        ),
        launch_arguments={
            "sim_ignition": sim_ignition,
            "sim_isaac": sim_isaac,
            "rviz": "false",
            "starting_controller": "joint_trajectory_controller",
        }.items(),
    )
    nodes.append(bringup)

    # ── MoveGroup (planning only, no MTC capability needed) ──────────
    moveit_config = MoveItConfigsBuilder(
        "z1_description", package_name="z1_moveit"
    ).to_moveit_configs()
    moveit_config.trajectory_execution["use_sim_time"] = use_sim_time

    move_group = generate_move_group_launch(moveit_config)
    nodes.append(move_group)

    # ── RViz ─────────────────────────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                str(get_package_share_path("z1_moveit")),
                "launch",
                "moveit_rviz.launch.py",
            )
        ),
    )
    nodes.append(rviz)

    # ── Task Sequencer ───────────────────────────────────────────────
    sequencer = TimerAction(
        period=10.0,
        actions=[
            Node(
                name="z1_task_sequencer",
                package="z1_task",
                executable="task_sequencer.py",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "step_mode": step_mode,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
        ],
    )
    nodes.append(sequencer)

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_ignition",
                default_value="true",
                description="Use Ignition Gazebo simulation",
            ),
            DeclareLaunchArgument(
                "sim_isaac",
                default_value="false",
                description="Use Isaac Sim as physics backend",
            ),
            DeclareLaunchArgument(
                "step",
                default_value="false",
                description="Step-by-step mode (press Enter between steps)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
