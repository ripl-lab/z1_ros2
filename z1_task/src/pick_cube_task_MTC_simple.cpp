// MTC pick-and-place for Z1 arm.
// MVP: flat stage pipeline, no collision objects, easy to modify gripper
// control.
//
// Sequence:
//   1. Open gripper
//   2. Move to approach position (offset from tag in tag frame)
//   3. Descend to grasp pose
//   4. Close gripper
//   5. Lift straight up
//   6. Move above place location
//   7. Lower straight down
//   8. Open gripper (release)
//   9. Retreat upward
//  10. Return home
//
// Prerequisites:
//   - move_group running with ExecuteTaskSolution capability loaded
//   - joint_trajectory_controller + gripper_controller active
//   - apriltag_ros publishing TF for the target tag
//
// Usage:
//   ros2 launch z1_task pick_cube_task.launch.py

#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace mtc = moveit::task_constructor;

// ── Robot config (must match z1_description SRDF) ───────────────────────
const std::string ARM_GROUP = "z1_arm";
const std::string GRIPPER_GROUP = "gripper";
const std::string EE_FRAME = "link06";
const std::string EE_NAME = "gripper_ee";
const std::string WORLD_FRAME = "world";

// Offset from link06 to gripperTip (0.051 gripperStator + 0.099 gripperTip)
const Eigen::Isometry3d TIP_OFFSET = [] {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.15, 0.0, 0.0);
  return t;
}();

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

void add_cartesian_move(
    mtc::Task &task, const std::string &name,
    const std::shared_ptr<mtc::solvers::CartesianPath> &planner,
    const Eigen::Vector3d &direction, double min_dist, double max_dist) {
  auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
  stage->setGroup(ARM_GROUP);
  stage->setIKFrame(TIP_OFFSET, EE_FRAME);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = WORLD_FRAME;
  vec.vector.x = direction.x();
  vec.vector.y = direction.y();
  vec.vector.z = direction.z();
  stage->setDirection(vec);
  stage->setMinMaxDistance(min_dist, max_dist);
  task.add(std::move(stage));
}

// Gripper stage via position controller (gripper_controller / GripperCommand).
// ── TO SWAP TO EFFORT/TORQUE CONTROL ──
// Replace the body of this function with a custom stage that:
//   1. Activates gripper_effort_controller (deactivate gripper_controller)
//      via /controller_manager/switch_controller
//   2. Publishes Float64MultiArray to /gripper_effort_controller/commands
//      (positive = close, negative = open)
//   3. Waits for stall / timeout
//   4. Re-activates gripper_controller
// See z1_examples/gripper_control.py for the switching + publish pattern.
void add_gripper_move(
    mtc::Task &task, const std::string &name,
    const std::shared_ptr<mtc::solvers::JointInterpolationPlanner> &planner,
    const std::string &named_state) {
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(GRIPPER_GROUP);
  stage->setGoal(named_state);
  task.add(std::move(stage));
}

void add_arm_pose_move(
    mtc::Task &task, const std::string &name,
    const std::shared_ptr<mtc::solvers::PipelinePlanner> &planner,
    const geometry_msgs::msg::PoseStamped &target) {
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(ARM_GROUP);
  stage->setIKFrame(TIP_OFFSET, EE_FRAME);
  stage->setGoal(target);
  task.add(std::move(stage));
}

// ── Main ────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("z1_pick_cube", options);
  auto log = node->get_logger();

  // ── Parameters ──────────────────────────────────────────────────────
  auto tag_frame = node->get_parameter("tag_frame").as_string();
  auto approach_offset_x = node->get_parameter("approach_offset_x").as_double();
  auto approach_offset_y = node->get_parameter("approach_offset_y").as_double();
  auto approach_offset_z = node->get_parameter("approach_offset_z").as_double();
  auto grasp_offset = node->get_parameter("grasp_offset").as_double();
  auto lift_height = node->declare_parameter<double>("lift_height", 0.30);
  auto place_offset_x = node->declare_parameter<double>("place_offset_x", 0.0);
  auto place_offset_y = node->declare_parameter<double>("place_offset_y", 0.15);
  auto place_lower =
      node->declare_parameter<double>("place_lower_distance", 0.10);
  auto goal_joint_tolerance =
      node->get_parameter("goal_joint_tolerance").as_double();

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

  // EE orientation: rotate tag frame 90° around local Y so that
  // gripper X (extension axis) aligns with -tag Z (into the tag surface).
  // Result: EE_X = -tag_Z, EE_Y = tag_Y, EE_Z = tag_X.
  Eigen::Quaterniond tag_q(tag_pose.rotation());
  Eigen::Quaterniond ry90(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond ee_q = (tag_q * ry90).normalized();

  Eigen::Vector3d approach_pos = tag_pos + approach_offset_x * tag_x +
                                 approach_offset_y * tag_y +
                                 approach_offset_z * tag_z;
  Eigen::Vector3d grasp_pos = tag_pos + grasp_offset * tag_z;

  Eigen::Vector3d descent_vec = grasp_pos - approach_pos;
  double descend_dist = descent_vec.norm();
  Eigen::Vector3d descent_dir = descent_vec.normalized();

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
  RCLCPP_INFO(log, "EE orient    quat(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
              ee_q.w(), ee_q.x(), ee_q.y(), ee_q.z());
  RCLCPP_INFO(log, "Descend dist: %.3f m   Lift: %.3f m   Joint tol: %.3f rad",
              descend_dist, lift_height, goal_joint_tolerance);

  auto approach_msg = make_pose(approach_pos, ee_q, WORLD_FRAME);
  auto place_msg = make_pose(place_above_pos, ee_q, WORLD_FRAME);

  // ── 3. Build MTC task ───────────────────────────────────────────────
  mtc::Task task;
  task.stages()->setName("pick_and_place");
  task.loadRobotModel(node);

  task.setProperty("group", ARM_GROUP);
  task.setProperty("eef", EE_NAME);
  task.setProperty("ik_frame", EE_FRAME);

  auto max_solutions = node->declare_parameter<int>("max_solutions", 10);

  // Planners
  auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  ompl->setProperty("num_planning_attempts", 10u);
  ompl->setProperty("goal_joint_tolerance", goal_joint_tolerance);
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setStepSize(0.005);
  cartesian->setMaxVelocityScalingFactor(0.5);
  cartesian->setMaxAccelerationScalingFactor(0.5);
  auto joint_interp =
      std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // ── PICK ────────────────────────────────────────────────────────────
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  add_gripper_move(task, "open gripper", joint_interp, "open");

  add_arm_pose_move(task, "approach", ompl, approach_msg);

  add_cartesian_move(task, "descend", cartesian, descent_dir,
                     descend_dist * 0.8, descend_dist);

  add_gripper_move(task, "close gripper", joint_interp, "closed");

  add_cartesian_move(task, "lift", cartesian, Eigen::Vector3d::UnitZ(),
                     lift_height * 0.5, lift_height);

  // ── PLACE ───────────────────────────────────────────────────────────
  add_arm_pose_move(task, "move to place", ompl, place_msg);

  add_cartesian_move(task, "lower", cartesian, -Eigen::Vector3d::UnitZ(),
                     place_lower * 0.5, place_lower);

  add_gripper_move(task, "release", joint_interp, "open");

  add_cartesian_move(task, "retreat", cartesian, Eigen::Vector3d::UnitZ(), 0.05,
                     place_lower);

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("home", ompl);
    stage->setGroup(ARM_GROUP);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  // ── 4. Plan ──────────────────────────────────────────────────────────
  try {
    task.init();
  } catch (const mtc::InitStageException &e) {
    RCLCPP_ERROR_STREAM(log, "Task init failed: " << e);
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  RCLCPP_INFO(log, "Planning (up to %ld solutions) ...", max_solutions);
  if (!task.plan(max_solutions)) {
    RCLCPP_ERROR(log, "Planning failed – no solutions found");
    RCLCPP_ERROR(log, "Target poses that failed:");
    RCLCPP_ERROR(log,
                 "  Approach  pos(%.3f, %.3f, %.3f) quat(w=%.3f, x=%.3f, "
                 "y=%.3f, z=%.3f)",
                 approach_pos.x(), approach_pos.y(), approach_pos.z(), ee_q.w(),
                 ee_q.x(), ee_q.y(), ee_q.z());
    RCLCPP_ERROR(log,
                 "  Place     pos(%.3f, %.3f, %.3f) quat(w=%.3f, x=%.3f, "
                 "y=%.3f, z=%.3f)",
                 place_above_pos.x(), place_above_pos.y(), place_above_pos.z(),
                 ee_q.w(), ee_q.x(), ee_q.y(), ee_q.z());
    RCLCPP_ERROR(log, "  Goal joint tolerance: %.3f rad", goal_joint_tolerance);
    double dist_from_base = approach_pos.norm();
    RCLCPP_ERROR(log, "  Approach distance from base: %.3f m", dist_from_base);
    std::ostringstream oss;
    task.printState(oss);
    RCLCPP_ERROR_STREAM(log, "Stage-by-stage status:\n" << oss.str());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  RCLCPP_INFO(log, "Found %zu solutions", task.solutions().size());

  // ── 5. Execute via direct controller calls ──────────────────────────
  // Isaac Sim's JTC does not report goal completion back to move_group,
  // so task.execute() hangs indefinitely. Instead we send each
  // sub-trajectory directly to the controllers with time-based waits.
  using JTC = control_msgs::action::FollowJointTrajectory;
  using Grip = control_msgs::action::GripperCommand;

  auto jtc_client = rclcpp_action::create_client<JTC>(
      node, "/joint_trajectory_controller/follow_joint_trajectory");
  auto grip_client = rclcpp_action::create_client<Grip>(
      node, "/gripper_controller/gripper_cmd");

  auto settle = node->declare_parameter<double>("move_settle_time", 5.0);
  auto grip_effort =
      node->declare_parameter<double>("gripper_max_effort", 20.0);

  if (!jtc_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(log, "JTC action server not available");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  if (!grip_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(log, "Gripper action server not available");
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  bool executed = false;
  int sol_idx = 0;

  for (const auto &solution_ptr : task.solutions()) {
    sol_idx++;
    auto &solution = *solution_ptr;
    task.introspection().publishSolution(solution);

    moveit_task_constructor_msgs::msg::Solution sol_msg;
    solution.toMsg(sol_msg);

    RCLCPP_INFO(log, "Trying solution %d/%zu (%zu sub-trajectories) ...",
                sol_idx, task.solutions().size(),
                sol_msg.sub_trajectory.size());

    rclcpp_action::ClientGoalHandle<JTC>::SharedPtr active_jtc_handle;
    int step = 0;
    bool ok = true;

    for (const auto &sub : sol_msg.sub_trajectory) {
      if (!rclcpp::ok()) {
        ok = false;
        break;
      }

      auto &jt = sub.trajectory.joint_trajectory;
      if (jt.joint_names.empty() || jt.points.empty())
        continue;
      step++;

      bool is_gripper =
          (jt.joint_names.size() == 1 && jt.joint_names[0] == "jointGripper");

      if (is_gripper) {
        double target = jt.points.back().positions[0];
        RCLCPP_INFO(log, "[%d] Gripper -> %.3f rad", step, target);

        Grip::Goal goal;
        goal.command.position = target;
        goal.command.max_effort = grip_effort;

        auto future = grip_client->async_send_goal(goal);
        if (future.wait_for(std::chrono::seconds(3)) !=
                std::future_status::ready ||
            !future.get()) {
          RCLCPP_ERROR(log, "[%d] Gripper goal rejected", step);
          ok = false;
          break;
        }
        RCLCPP_INFO(log, "[%d] Gripper goal accepted", step);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      } else {
        if (active_jtc_handle) {
          jtc_client->async_cancel_goal(active_jtc_handle);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        double duration = 0.0;
        if (!jt.points.empty()) {
          auto &t = jt.points.back().time_from_start;
          duration = t.sec + t.nanosec * 1e-9;
        }
        RCLCPP_INFO(log, "[%d] Arm trajectory (%zu pts, %.1f s)", step,
                    jt.points.size(), duration);

        JTC::Goal goal;
        goal.trajectory = jt;

        auto future = jtc_client->async_send_goal(goal);
        if (future.wait_for(std::chrono::seconds(5)) !=
                std::future_status::ready ||
            !future.get()) {
          RCLCPP_ERROR(log, "[%d] JTC goal rejected", step);
          ok = false;
          break;
        }
        active_jtc_handle = future.get();
        RCLCPP_INFO(log, "[%d] JTC goal accepted, waiting %.1f s", step,
                    std::max(duration + 1.0, settle));
        std::this_thread::sleep_for(
            std::chrono::duration<double>(std::max(duration + 1.0, settle)));
      }
    }

    if (ok) {
      RCLCPP_INFO(log, "Pick and place complete (solution %d)!", sol_idx);
      executed = true;
      break;
    }
    RCLCPP_WARN(log, "Solution %d failed at step %d, trying next ...", sol_idx,
                step);
  }

  if (!executed) {
    RCLCPP_ERROR(log, "All %d solutions failed during execution", sol_idx);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  spin_thread.join();
  return executed ? 0 : 1;
}