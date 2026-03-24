# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
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

from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)



def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()

    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")
    with_gripper = LaunchConfiguration("with_gripper")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    controller_config = LaunchConfiguration("controller_config")
    starting_controller = LaunchConfiguration("starting_controller")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_isaac = LaunchConfiguration("sim_isaac")

    use_sim_time = (sim_ignition.perform(context) == "true")

    # Conditions that tell whether the robot is simulated or not
    is_simulation = IfCondition(sim_ignition)
    is_real = UnlessCondition(sim_ignition)
    is_isaac = IfCondition(sim_isaac)

    #   ____
    #  / ___|___  _ __ ___  _ __ ___   ___  _ __  ___
    # | |   / _ \| '_ ` _ \| '_ ` _ \ / _ \| '_ \/ __|
    # | |__| (_) | | | | | | | | | | | (_) | | | \__ \
    #  \____\___/|_| |_| |_|_| |_| |_|\___/|_| |_|___/
    #
    robot_description_content = xacro.process(
        xacro_file.perform(context),
        mappings={
            "name": robot_name.perform(context),
            "prefix": "",
            "with_gripper": with_gripper.perform(context),
            "controllers": controller_config.perform(context),
            "sim_ignition": sim_ignition.perform(context),
            "sim_isaac": sim_isaac.perform(context),
        }
    )
    robot_description = {"robot_description": robot_description_content}

    # In sim, gz_ros2_control publishes joint_states in the model namespace (e.g. /z1/joint_states).
    # Remap so robot_state_publisher receives them and can publish TF for all links.
    # joint_states_remaps = []
    # if use_sim_time:
    #     joint_states_remaps = [
    #         ("joint_states", "/" + robot_name.perform(context) + "/joint_states"),
    #     ]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": use_sim_time,
        }],
        # remappings=joint_states_remaps,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
    )

    starting_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[starting_controller.perform(context), "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
        condition=IfCondition(with_gripper),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config.perform(context)],
        condition=IfCondition(rviz),
    )

    rviz_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start += [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        starting_controller_spawner,
        gripper_controller_spawner,
        rviz_delayed,
    ]

    #  ____            _   ____       _           _
    # |  _ \ ___  __ _| | |  _ \ ___ | |__   ___ | |_
    # | |_) / _ \/ _` | | | |_) / _ \| '_ \ / _ \| __|
    # |  _ <  __/ (_| | | |  _ < (_) | |_) | (_) | |_
    # |_| \_\___|\__,_|_| |_| \_\___/|_.__/ \___/ \__|
    #
    # ros2_control_node runs for both real hardware and Isaac Sim
    # (whenever sim_ignition=false); the hardware interface plugin selected
    # in the URDF determines what happens inside.
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, controller_config, {
                "use_sim_time": use_sim_time
            }
        ],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ],
        condition=is_real,  # is_real == UnlessCondition(sim_ignition)
    )

    nodes_to_start.append(controller_manager_node)

    # The Unitree SDK background process is only needed for the physical robot,
    # not when Isaac Sim is the backend.
    _is_real_hw = (
        sim_ignition.perform(context) != "true"
        and sim_isaac.perform(context) != "true"
    )
    if _is_real_hw:
        z1_controller_script_path = os.path.join(
            get_package_share_path("z1_hardware_interface"),
            "scripts",
            "z1_controller_process.py"
        )
        print(z1_controller_script_path)
        nodes_to_start.append(
            ExecuteProcess(
                cmd=["python3", z1_controller_script_path],
                output="screen",
            )
        )

    #  ___            _ _   _
    # |_ _|__ _ _ __ (_) |_(_) ___  _ __
    #  | |/ _` | '_ \| | __| |/ _ \| '_ \
    #  | | (_| | | | | | |_| | (_) | | | |
    # |___\__, |_| |_|_|\__|_|\___/|_| |_|
    #     |___/
    # Bridge Gazebo /clock to ROS 2 so use_sim_time works and TF/joint_states are processed
    clock_bridge_launch = os.path.join(
        get_package_share_path("ros_gz_bridge"), "launch", "clock_bridge.launch"
    )
    clock_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(clock_bridge_launch),
        launch_arguments=[("bridge_name", "z1_clock_bridge")],
        condition=IfCondition(sim_ignition),
    )

    ignition_simulator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ], ),
        launch_arguments={
            "gz_args": " -r -v 1 empty.sdf",
        }.items(),
        condition=IfCondition(sim_ignition),
    )

    ignition_spawn_z1_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "/robot_description",
        ],
        condition=IfCondition(sim_ignition),
    )

    nodes_to_start += [
        clock_bridge,
        ignition_simulator_node,
        ignition_spawn_z1_node,
    ]
    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    rviz_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "rviz", "z1.rviz"
    )
    xacro_file_default = os.path.join(
        get_package_share_path("z1_description"), "urdf", "z1.urdf.xacro"
    )
    controller_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "config", "z1_controllers.yaml"
    )

    # --- Setup environment variables so Gazebo finds z1_description meshes
    # IGN_GAZEBO_* = Ignition / older; GZ_SIM_* = Gazebo Harmonic / newer
    z1_share = os.path.join(get_package_prefix("z1_description"), "share")
    for res_var in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        if res_var in os.environ:
            os.environ[res_var] = os.environ[res_var].rstrip(":") + ":" + z1_share
        else:
            os.environ[res_var] = z1_share

    LIB_ENV_VAR = "IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
    if LIB_ENV_VAR in os.environ:
        os.environ[LIB_ENV_VAR] += ":/opt/ros/humble/lib"
    else:
        os.environ[LIB_ENV_VAR] = "/opt/ros/humble/lib"

    # --- Launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value=xacro_file_default,
            description="Path to xacro file of the Z1 manipulator"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name", default_value="z1", description="Name of the robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value=controller_config_default,
            description=
            "Path to the controllers.yaml file that can be loaded by the robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper", default_value="true", description="Use the gripper?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "starting_controller",
            default_value="torque_controller",
            description="Name of the controller to be started"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Launch simulation in Ignition Gazebo?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Use Isaac Sim as the physics backend (requires sim_ignition:=false)"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_default,
            description="Path to RViz configuration file"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
