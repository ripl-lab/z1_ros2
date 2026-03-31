// Non-MTC pick-and-place for Z1 arm using MoveGroupInterface.
// Direct plan-and-execute with Cartesian paths + TOTG for linear moves.
// Faster and simpler than the MTC version (pick_cube_task_MTC.cpp).
//
// Techniques adopted from Isaac bin-picking reference:
//   - MoveGroupInterface for planning (no MTC overhead)
//   - computeCartesianPath for precise linear motions
//   - TimeOptimalTrajectoryGeneration for smooth vel/accel control
//   - startStateMonitor + setStartStateToCurrentState before every plan
//
// Isaac Sim's JTC does not report goal completion back to move_group,
// so arm.execute() hangs indefinitely.  We use MoveGroupInterface for
// planning only, then send trajectories directly to the JTC action
// server with time-based waits.
//
// Sequence:
//   1. Open gripper
//   2. Move to approach position (free-space)
//   3. Descend to grasp pose (Cartesian + TOTG)
//   4. Close gripper
//   5. Lift straight up (Cartesian + TOTG)
//   6. Move above place location (free-space)
//   7. Lower straight down (Cartesian + TOTG)
//   8. Open gripper (release)
//   9. Retreat upward (Cartesian + TOTG)
//  10. Return home (named target)
//
// Prerequisites:
//   - move_group running
//   - joint_trajectory_controller + gripper_controller active
//   - apriltag_ros publishing TF for the target tag
//
// Usage:
//   ros2 launch z1_task pick_cube_task.launch.py

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ── Robot config (must match z1_description SRDF) ───────────────────────
const std::string ARM_GROUP = "z1_arm";
const std::string EE_FRAME = "link06";
const std::string WORLD_FRAME = "world";

// Gripper SRDF joint values
constexpr double GRIPPER_OPEN_POS = -1.0;
constexpr double GRIPPER_CLOSE_POS = 0.0;

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using JTC = control_msgs::action::FollowJointTrajectory;
using GripAction = control_msgs::action::GripperCommand;

// ── Helpers ─────────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped make_pose(const Eigen::Vector3d &pos,
                                          const Eigen::Quaterniond &orientation,
                                          const std::string &frame) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame;
  ps.pose.position.x = pos.x();
  ps.pose.position.y = pos.y();
  ps.pose.position.z = pos.z();
  ps.pose.orientation = tf2::toMsg(orientation);
  return ps;
}

// Send a joint trajectory directly to the JTC action server and wait
// based on the trajectory duration.  Bypasses move_group's execution
// monitoring which hangs in Isaac Sim (JTC never reports completion).
bool execute_on_jtc(rclcpp_action::Client<JTC>::SharedPtr jtc,
                    const trajectory_msgs::msg::JointTrajectory &traj,
                    double settle_time, rclcpp::Logger log) {
  if (traj.points.empty()) {
    RCLCPP_WARN(log, "Empty trajectory, skipping");
    return true;
  }

  JTC::Goal goal;
  goal.trajectory = traj;

  auto future = jtc->async_send_goal(goal);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
      !future.get()) {
    RCLCPP_ERROR(log, "JTC goal rejected or timed out");
    return false;
  }

  auto &last = traj.points.back().time_from_start;
  double duration = last.sec + last.nanosec * 1e-9;
  double wait = std::max(duration + 1.0, settle_time);

  RCLCPP_INFO(log, "JTC goal accepted (%zu pts, %.1f s), waiting %.1f s",
              traj.points.size(), duration, wait);
  std::this_thread::sleep_for(std::chrono::duration<double>(wait));
  return true;
}

// Cartesian move with TOTG time-parameterization (Isaac-style).
// Plans a straight-line path from current EE pose to target, applies
// TimeOptimalTrajectoryGeneration, then sends directly to JTC.
bool move_cartesian(MoveGroup &arm, const geometry_msgs::msg::Pose &target,
                    double vel_scale, double accel_scale, double eef_step,
                    rclcpp_action::Client<JTC>::SharedPtr jtc,
                    double settle, rclcpp::Logger log) {
  arm.setStartStateToCurrentState();
  auto start = arm.getCurrentPose(EE_FRAME).pose;

  std::vector<geometry_msgs::msg::Pose> waypoints = {start, target};

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction =
      arm.computeCartesianPath(waypoints, eef_step, trajectory);

  if (fraction < 0.95) {
    RCLCPP_ERROR(log, "Cartesian path only %.0f%% complete", fraction * 100.0);
    return false;
  }

  robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
  rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, vel_scale, accel_scale);
  rt.getRobotTrajectoryMsg(trajectory);

  return execute_on_jtc(jtc, trajectory.joint_trajectory, settle, log);
}

// Free-space plan via OMPL, then send directly to JTC.
bool move_to_pose(MoveGroup &arm, const geometry_msgs::msg::PoseStamped &target,
                  rclcpp_action::Client<JTC>::SharedPtr jtc, double settle,
                  rclcpp::Logger log) {
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target);

  MoveGroup::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(log, "Free-space planning failed");
    return false;
  }
  return execute_on_jtc(jtc, plan.trajectory.joint_trajectory, settle, log);
}

// Named target plan, then send directly to JTC.
bool move_to_named(MoveGroup &arm, const std::string &name,
                   rclcpp_action::Client<JTC>::SharedPtr jtc, double settle,
                   rclcpp::Logger log) {
  arm.setStartStateToCurrentState();
  if (!arm.setNamedTarget(name)) {
    RCLCPP_ERROR(log, "Unknown named target: %s", name.c_str());
    return false;
  }

  MoveGroup::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(log, "Planning to '%s' failed", name.c_str());
    return false;
  }
  return execute_on_jtc(jtc, plan.trajectory.joint_trajectory, settle, log);
}

bool send_gripper(rclcpp_action::Client<GripAction>::SharedPtr client,
                  double position, double effort, rclcpp::Logger log) {
  GripAction::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = effort;

  auto future = client->async_send_goal(goal);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
      !future.get()) {
    RCLCPP_ERROR(log, "Gripper goal rejected or timed out");
    return false;
  }

  RCLCPP_INFO(log, "Gripper goal accepted (pos=%.2f)", position);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  return true;
}

// ── Main ────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("z1_pick_cube", options);
  auto log = node->get_logger();

  // ── Parameters ──────────────────────────────────────────────────────
  auto tag_frame =
      node->declare_parameter<std::string>("tag_frame", "apriltag_21");
  auto approach_offset_x = node->get_parameter("approach_offset_x").as_double();
  auto approach_offset_y = node->get_parameter("approach_offset_y").as_double();
  auto approach_offset_z = node->get_parameter("approach_offset_z").as_double();
  auto grasp_offset = node->get_parameter("grasp_offset").as_double();
  auto lift_height = node->declare_parameter<double>("lift_height", 0.30);
  auto place_offset_x = node->declare_parameter<double>("place_offset_x", 0.0);
  auto place_offset_y =
      node->declare_parameter<double>("place_offset_y", 0.15);
  auto place_lower =
      node->declare_parameter<double>("place_lower_distance", 0.10);
  auto vel_scale = node->declare_parameter<double>("velocity_scaling", 0.5);
  auto accel_scale =
      node->declare_parameter<double>("acceleration_scaling", 0.5);
  auto cartesian_step =
      node->declare_parameter<double>("cartesian_step_size", 0.002);
  auto gripper_effort =
      node->declare_parameter<double>("gripper_max_effort", 20.0);
  auto planning_time = node->declare_parameter<double>("planning_time", 5.0);
  auto settle = node->declare_parameter<double>("move_settle_time", 5.0);

  // Spin in background so TF and MoveIt callbacks are processed
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ── 1. Locate the AprilTag via TF ───────────────────────────────────
  tf2_ros::Buffer tf_buf(node->get_clock());
  tf_buf.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listen(tf_buf, node, false);

  RCLCPP_INFO(log, "Waiting for TF: %s -> %s ...", WORLD_FRAME.c_str(),
              tag_frame.c_str());

  geometry_msgs::msg::TransformStamped tag_tf;
  bool found = false;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    try {
      if (tf_buf.canTransform(WORLD_FRAME, tag_frame, tf2::TimePointZero,
                              tf2::durationFromSec(1.0))) {
        tag_tf =
            tf_buf.lookupTransform(WORLD_FRAME, tag_frame, tf2::TimePointZero);
        found = true;
        break;
      }
    } catch (const tf2::TransformException &) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if (!found) {
    RCLCPP_ERROR(log, "TF for '%s' not found within 20 s", tag_frame.c_str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // ── 2. Compute target poses from the tag TF ────────────────────────
  Eigen::Isometry3d tag_pose = tf2::transformToEigen(tag_tf);
  Eigen::Vector3d tag_pos = tag_pose.translation();
  Eigen::Vector3d tag_x = tag_pose.rotation().col(0);
  Eigen::Vector3d tag_y = tag_pose.rotation().col(1);
  Eigen::Vector3d tag_z = tag_pose.rotation().col(2);

  // EE orientation: rotate tag frame 90° around Y so the gripper
  // (which extends along EE X) points toward the tag surface (-tag Z).
  Eigen::Quaterniond tag_q(tag_pose.rotation());
  Eigen::Quaterniond ry90(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond ee_q = (tag_q * ry90).normalized();

  Eigen::Vector3d approach_pos = tag_pos + approach_offset_x * tag_x +
                                 approach_offset_y * tag_y +
                                 approach_offset_z * tag_z;
  Eigen::Vector3d grasp_pos = tag_pos + grasp_offset * tag_z;

  Eigen::Vector3d place_above_pos(tag_pos.x() + place_offset_x,
                                  tag_pos.y() + place_offset_y,
                                  approach_pos.z());

  RCLCPP_INFO(log, "Tag         (%.3f, %.3f, %.3f)", tag_pos.x(), tag_pos.y(),
              tag_pos.z());
  RCLCPP_INFO(log, "Approach    (%.3f, %.3f, %.3f)", approach_pos.x(),
              approach_pos.y(), approach_pos.z());
  RCLCPP_INFO(log, "Grasp       (%.3f, %.3f, %.3f)", grasp_pos.x(),
              grasp_pos.y(), grasp_pos.z());
  RCLCPP_INFO(log, "Place-above (%.3f, %.3f, %.3f)", place_above_pos.x(),
              place_above_pos.y(), place_above_pos.z());
  RCLCPP_INFO(log, "Lift height: %.3f m", lift_height);

  auto approach_msg = make_pose(approach_pos, ee_q, WORLD_FRAME);
  auto place_msg = make_pose(place_above_pos, ee_q, WORLD_FRAME);

  // Grasp target for the Cartesian descent
  geometry_msgs::msg::Pose grasp_target;
  grasp_target.position.x = grasp_pos.x();
  grasp_target.position.y = grasp_pos.y();
  grasp_target.position.z = grasp_pos.z();
  grasp_target.orientation = tf2::toMsg(ee_q);

  // ── 3. Init MoveGroupInterface (planning only) ────────────────────
  MoveGroup arm(node, ARM_GROUP);
  arm.startStateMonitor();
  arm.setMaxVelocityScalingFactor(vel_scale);
  arm.setMaxAccelerationScalingFactor(accel_scale);
  arm.setPlanningTime(planning_time);
  arm.setNumPlanningAttempts(5);

  RCLCPP_INFO(log, "Waiting for robot state ...");
  auto state_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < state_deadline) {
    if (arm.getCurrentState(0.5))
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ── 4. Init action clients ────────────────────────────────────────
  auto jtc = rclcpp_action::create_client<JTC>(
      node, "/joint_trajectory_controller/follow_joint_trajectory");
  auto grip = rclcpp_action::create_client<GripAction>(
      node, "/gripper_controller/gripper_cmd");

  if (!jtc->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(log, "JTC action server not available");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  if (!grip->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(log, "Gripper action server not available");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  // ── 5. Execute pick-and-place ─────────────────────────────────────
  double slow_vel = vel_scale * 0.6;
  double slow_accel = accel_scale * 0.6;

  auto ok = [&]() -> bool {
    // ── PICK ──────────────────────────────────────────────────────
    RCLCPP_INFO(log, "[1/10] Opening gripper");
    if (!send_gripper(grip, GRIPPER_OPEN_POS, gripper_effort, log))
      return false;

    RCLCPP_INFO(log, "[2/10] Moving to approach position");
    if (!move_to_pose(arm, approach_msg, jtc, settle, log))
      return false;

    RCLCPP_INFO(log, "[3/10] Descending to grasp");
    if (!move_cartesian(arm, grasp_target, slow_vel, slow_accel, cartesian_step,
                        jtc, settle, log))
      return false;

    RCLCPP_INFO(log, "[4/10] Closing gripper");
    if (!send_gripper(grip, GRIPPER_CLOSE_POS, gripper_effort, log))
      return false;

    RCLCPP_INFO(log, "[5/10] Lifting");
    {
      auto cur = arm.getCurrentPose(EE_FRAME).pose;
      cur.position.z += lift_height;
      if (!move_cartesian(arm, cur, vel_scale, accel_scale, cartesian_step, jtc,
                          settle, log))
        return false;
    }

    // ── PLACE ─────────────────────────────────────────────────────
    RCLCPP_INFO(log, "[6/10] Moving to place position");
    if (!move_to_pose(arm, place_msg, jtc, settle, log))
      return false;

    RCLCPP_INFO(log, "[7/10] Lowering");
    {
      auto cur = arm.getCurrentPose(EE_FRAME).pose;
      cur.position.z -= place_lower;
      if (!move_cartesian(arm, cur, slow_vel, slow_accel, cartesian_step, jtc,
                          settle, log))
        return false;
    }

    RCLCPP_INFO(log, "[8/10] Releasing");
    if (!send_gripper(grip, GRIPPER_OPEN_POS, gripper_effort, log))
      return false;

    RCLCPP_INFO(log, "[9/10] Retreating");
    {
      auto cur = arm.getCurrentPose(EE_FRAME).pose;
      cur.position.z += place_lower;
      if (!move_cartesian(arm, cur, vel_scale, accel_scale, cartesian_step, jtc,
                          settle, log))
        return false;
    }

    RCLCPP_INFO(log, "[10/10] Returning home");
    move_to_named(arm, "home", jtc, settle, log);

    return true;
  }();

  if (ok)
    RCLCPP_INFO(log, "Pick and place complete!");
  else
    RCLCPP_ERROR(log, "Pick and place failed");

  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  spin_thread.join();
  return ok ? 0 : 1;
}
