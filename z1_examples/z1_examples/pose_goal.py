"""
Send a 6D pose goal to the Z1 end-effector via MoveIt.

The planning group "z1_arm" runs KDL IK on the chain link00→link06.
MoveIt solves the IK, plans a joint trajectory, and executes it through
the joint_trajectory_controller → FollowJointTrajectory action server.

Prerequisites:
    ros2 launch z1_moveit z1_moveit.launch.py [sim_ignition:=false]

or use the bundled launch file:
    ros2 launch z1_examples pose_goal.launch.py [sim_ignition:=false]
"""

import rclpy
import rclpy.logging
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


PLANNING_GROUP = "z1_arm"
EE_LINK        = "link06"
REFERENCE_FRAME = "world"


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("pose_goal")

    # MoveItPy creates its own internal node and reads robot_description /
    # MoveIt parameters from the ROS2 parameter server (published by move_group).
    moveit = MoveItPy(node_name="pose_goal_node")
    arm    = moveit.get_planning_component(PLANNING_GROUP)

    logger.info(f"MoveItPy ready — planning group: '{PLANNING_GROUP}', EE: '{EE_LINK}'")

    # ------------------------------------------------------------------
    # Define the target 6D pose.
    # Adjust position (x, y, z) and orientation (quaternion x, y, z, w)
    # to any reachable pose within the arm's ~0.7 m workspace.
    # ------------------------------------------------------------------
    target = PoseStamped()
    target.header.frame_id = REFERENCE_FRAME

    # Position [m] — in front of and above the base
    target.pose.position.x = 0.35
    target.pose.position.y = 0.0
    target.pose.position.z = 0.25

    # Orientation (quaternion) — pointing straight along the world x-axis
    # (identity rotation relative to the world frame)
    target.pose.orientation.x = 0.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 1.0

    # ------------------------------------------------------------------
    # Plan and execute
    # ------------------------------------------------------------------
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=target, pose_link=EE_LINK)

    logger.info(
        f"Planning to pose: "
        f"pos=({target.pose.position.x:.3f}, "
        f"{target.pose.position.y:.3f}, "
        f"{target.pose.position.z:.3f}) "
        f"quat=({target.pose.orientation.x:.3f}, "
        f"{target.pose.orientation.y:.3f}, "
        f"{target.pose.orientation.z:.3f}, "
        f"{target.pose.orientation.w:.3f})"
    )

    plan_result = arm.plan()

    if plan_result:
        logger.info("Planning succeeded — executing trajectory")
        moveit.execute(plan_result.trajectory, controllers=[])
        logger.info("Execution complete")
    else:
        logger.error(
            "Planning failed. Check that the pose is reachable and that "
            "move_group is running with the joint_trajectory_controller active."
        )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
