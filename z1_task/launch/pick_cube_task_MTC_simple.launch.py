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
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    camera_frame = LaunchConfiguration("camera_frame").perform(context)
    landmark_frame = LaunchConfiguration("landmark_frame").perform(context)
    observed_tag_frame = LaunchConfiguration("observed_tag_frame").perform(context)
    landmark_x = LaunchConfiguration("landmark_x").perform(context)
    landmark_y = LaunchConfiguration("landmark_y").perform(context)
    landmark_z = LaunchConfiguration("landmark_z").perform(context)

    tag_frame = LaunchConfiguration("tag_frame")
    grasp_offset = LaunchConfiguration("grasp_offset")
    approach_offset_x = LaunchConfiguration("approach_offset_x")
    approach_offset_y = LaunchConfiguration("approach_offset_y")
    approach_offset_z = LaunchConfiguration("approach_offset_z")
    goal_joint_tolerance = LaunchConfiguration("goal_joint_tolerance")

    if sim_isaac == "true":
        sim_ignition = "false"

    is_real = sim_ignition == "false" and sim_isaac == "false"
    use_sim_time = not is_real

    nodes = []

    # ── Bringup (controllers, robot_state_publisher, sim) ────────────────
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

    # ── RealSense camera (real hardware only) ────────────────────────────
    if is_real:
        realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    str(get_package_share_path("realsense2_camera")),
                    "launch",
                    "rs_launch.py",
                )
            ),
        )
        nodes.append(realsense)

    # ── MoveIt + MTC ExecuteTaskSolution capability ──────────────────────
    moveit_config = MoveItConfigsBuilder(
        "z1_description", package_name="z1_moveit"
    ).to_moveit_configs()
    moveit_config.trajectory_execution["use_sim_time"] = use_sim_time
    moveit_config.move_group_capabilities["capabilities"] = (
        "move_group/ExecuteTaskSolutionCapability"
    )

    move_group = generate_move_group_launch(moveit_config)
    nodes.append(move_group)

    # ── RViz ─────────────────────────────────────────────────────────────
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

    # ── AprilTag detector ────────────────────────────────────────────────
    apriltag_config_file = "apriltag_real.yaml" if is_real else "apriltag.yaml"
    apriltag_config = os.path.join(
        str(get_package_share_path("z1_examples")), "config", apriltag_config_file
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
    nodes.append(apriltag_node)

    # ── Static TF for landmark (real hardware only) ──────────────────────
    if is_real:
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                landmark_x, landmark_y, landmark_z,
                "0.0", "0.0", "0.0",
                "world", landmark_frame,
            ],
            output="screen",
        )
        nodes.append(static_tf)

    # ── AprilTag localizer (world → camera via landmark tag) ─────────────
    localizer_node = Node(
        package="z1_examples",
        executable="apriltag_localizer.py",
        parameters=[
            {
                "world_frame": "world",
                "landmark_frame": landmark_frame,
                "observed_tag_frame": observed_tag_frame,
                "camera_frame": camera_frame,
            }
        ],
        output="screen",
    )
    nodes.append(localizer_node)

    # ── MTC simple pick-and-place node ───────────────────────────────────
    moveit_config_dict = moveit_config.to_dict()
    if "planning_pipelines" in moveit_config_dict:
        moveit_config_dict["planning_pipelines.pipeline_names"] = moveit_config_dict[
            "planning_pipelines"
        ]

    moveit_config_dict.update(
        {
            "planning_scene_monitor_options": {
                "name": "planning_scene_monitor",
                "robot_description": "robot_description",
                "joint_state_topic": "/joint_states",
                "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
                "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
                "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
                "wait_for_initial_state_timeout": 10.0,
            },
        }
    )

    mtc_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                name="z1_pick_cube",
                package="z1_task",
                executable="pick_cube_task_MTC_simple",
                output="screen",
                parameters=[
                    moveit_config_dict,
                    {
                        "tag_frame": tag_frame,
                        "grasp_offset": grasp_offset,
                        "approach_offset_x": approach_offset_x,
                        "approach_offset_y": approach_offset_y,
                        "approach_offset_z": approach_offset_z,
                        "goal_joint_tolerance": goal_joint_tolerance,
                    },
                ],
            ),
        ],
    )
    nodes.append(mtc_node)

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
                description="Camera TF frame name",
            ),
            DeclareLaunchArgument(
                "landmark_frame",
                default_value="apriltag_0_landmark",
                description="Landmark tag TF frame for localizer",
            ),
            DeclareLaunchArgument(
                "observed_tag_frame",
                default_value="apriltag_0",
                description="Observed tag TF frame from apriltag_ros",
            ),
            DeclareLaunchArgument(
                "landmark_x",
                default_value="0.0",
                description="Static TF: landmark X in world (real only)",
            ),
            DeclareLaunchArgument(
                "landmark_y",
                default_value="0.18",
                description="Static TF: landmark Y in world (real only)",
            ),
            DeclareLaunchArgument(
                "landmark_z",
                default_value="0.0",
                description="Static TF: landmark Z in world (real only)",
            ),
            DeclareLaunchArgument(
                "tag_frame",
                default_value="apriltag_0",
                description="TF frame of the AprilTag on the pick target",
            ),
            DeclareLaunchArgument(
                "grasp_offset",
                default_value="0.135",
                description="Z offset for grasping",
            ),
            DeclareLaunchArgument(
                "approach_offset_x",
                default_value="0.0",
                description="Approach offset along tag X axis (meters)",
            ),
            DeclareLaunchArgument(
                "approach_offset_y",
                default_value="0.0",
                description="Approach offset along tag Y axis (meters)",
            ),
            DeclareLaunchArgument(
                "approach_offset_z",
                default_value="0.4",
                description="Approach offset along tag Z axis (meters)",
            ),
            DeclareLaunchArgument(
                "goal_joint_tolerance",
                default_value="0.1",
                description="OMPL goal joint tolerance in radians (higher = more planner flexibility)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
