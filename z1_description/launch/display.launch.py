import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    z1_description_share = get_package_share_directory('z1_description')
    z1_bringup_share = get_package_share_directory('z1_bringup')

    end_effector_arg = DeclareLaunchArgument(
        'end_effector',
        default_value='clarius',
        description='Type of end effector to use. Options: "z1_gripper", "clarius"'
    )

    urdf_file = os.path.join(z1_description_share, 'urdf', 'z1.urdf.xacro')

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' end_effector:=',
        LaunchConfiguration('end_effector')
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_config_file = os.path.join(z1_bringup_share, 'rviz', 'z1.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        end_effector_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])