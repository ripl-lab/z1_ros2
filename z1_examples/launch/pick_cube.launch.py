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
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    sim_ignition = LaunchConfiguration("sim_ignition").perform(context)
    sim_isaac = LaunchConfiguration("sim_isaac").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    camera_frame = LaunchConfiguration("camera_frame").perform(context)

    if sim_isaac == "true":
        sim_ignition = "false"

    # ── MoveIt config (needed by MoveItPy inside pick_cube.py) ───────────
    moveit_config = MoveItConfigsBuilder(
        "z1_description", package_name="z1_moveit"
    ).to_moveit_configs()
    moveit_config.trajectory_execution["use_sim_time"] = (
        sim_ignition == "true" or sim_isaac == "true"
    )

    # ── MoveIt stack (bringup + move_group + rviz) ───────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("z1_moveit"), "/launch/z1_moveit.launch.py"
        ]),
        launch_arguments={
            "sim_ignition": sim_ignition,
            "sim_isaac": sim_isaac,
            "rviz": "true",
        }.items(),
    )

    # ── AprilTag detector ────────────────────────────────────────────────
    apriltag_config = os.path.join(
        get_package_share_path("z1_examples"), "config", "apriltag.yaml"
    )
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        remappings=[
            ("image_rect", image_topic),
            ("camera_info", camera_info_topic),
        ],
        parameters=[apriltag_config],
        output="screen",
    )

    # ── AprilTag localizer (world → camera via landmark tag 69) ──────────
    localizer_node = Node(
        package="z1_examples",
        executable="apriltag_localizer.py",
        parameters=[{
            "world_frame": "world",
            "landmark_frame": "apriltag_69_landmark",
            "observed_tag_frame": "apriltag_69",
            "camera_frame": camera_frame,
        }],
        output="screen",
    )

    # ── Pick cube (delayed; name must match MoveItPy node_name) ──────────
    moveit_config_dict = moveit_config.to_dict()
    if "planning_pipelines" in moveit_config_dict:
        moveit_config_dict["planning_pipelines.pipeline_names"] = moveit_config_dict["planning_pipelines"]

    moveit_config_dict.update({
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0,
        },
    })

    pick_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                name="pick_cube_moveit",
                package="z1_examples",
                executable="pick_cube.py",
                output="screen",
                parameters=[moveit_config_dict],
            ),
        ],
    )

    return [moveit_launch, apriltag_node, localizer_node, pick_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Ignition simulation (true) or real hardware (false)",
        ),
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Use Isaac Sim as the physics backend",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/rgb",
            description="Raw image topic for AprilTag detection",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera_info",
            description="Camera info topic for AprilTag detection",
        ),
        DeclareLaunchArgument(
            "camera_frame",
            default_value="Camera_OmniVision_OV9782_Color",
            description="Camera TF frame name (used by apriltag_localizer)",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
