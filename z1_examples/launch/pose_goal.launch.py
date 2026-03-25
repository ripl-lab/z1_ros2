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



def launch_setup(context, *args, **kwargs):
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_isaac = LaunchConfiguration("sim_isaac")

    # Start the full MoveIt stack (move_group + z1_bringup + joint_trajectory_controller).
    # joint_trajectory_controller is the default starting_controller in z1_moveit.launch.py.
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

    # Delay pose_goal.py so that move_group and the controller are fully up
    pose_goal_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="z1_examples",
                executable="pose_goal.py",
                output="screen",
            ),
        ],
    )

    return [moveit_launch, pose_goal_node]



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
            description="Use Isaac Sim as the physics backend (requires sim_ignition:=false)",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
