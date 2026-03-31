// MTC multi-object pick-and-place for Z1 arm.
// Detects bin locations via AprilTags at startup, then picks each object
// and places it in the assigned bin.
//
// Task plan is configured via BINS and TASKS constants at the top.
// Status messages are published to ~/status for monitoring.
//
// Per-task sequence:
//   1. Open gripper
//   2. Move to approach position (offset from object tag in tag frame)
//   3. Descend to grasp pose
//   4. Close gripper
//   5. Lift straight up
//   6. Move above target bin
//   7. Lower into bin
//   8. Open gripper (release)
//   9. Retreat upward
//  10. Return home
//
// Prerequisites:
//   - move_group running with ExecuteTaskSolution capability loaded
//   - joint_trajectory_controller + gripper_controller active
//   - apriltag_ros publishing TF for detected tags
//
// Usage:
//   ros2 launch z1_task pick_cube_task.launch.py

#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
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

// ══════════════════════════════════════════════════════════════════════════
// TASK PLAN — edit these to add/remove bins, objects, and assignments.
// Comment out lines to skip specific bins or pick-place tasks.
// Future: this config will be received from an external task planner.
// ══════════════════════════════════════════════════════════════════════════

struct BinDef {
  std::string tag_frame; // TF frame published by apriltag_ros
  std::string label;     // human-readable name, referenced by TaskDef
};

struct TaskDef {
  std::string object_tag; // TF frame of the object's AprilTag
  std::string bin_label;  // target bin (must match a BinDef::label)
};

static const std::vector<BinDef> BINS = {
    {"apriltag_100", "bin_100"},
    {"apriltag_200", "bin_200"},
    // {"apriltag_300", "bin_300"},  // add more bins here
};

static const std::vector<TaskDef> TASKS = {
    {"apriltag_21", "bin_100"}, // pick tag-21 object -> drop in bin 100
    {"apriltag_22", "bin_200"}, // pick tag-22 object -> drop in bin 200
    // {"apriltag_23", "bin_100"},  // add more tasks here
};

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
  stage->setIKFrame(Eigen::Isometry3d::Identity(), EE_FRAME);

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
  stage->setGoal(target);
  stage->setIKFrame(Eigen::Isometry3d::Identity(), EE_FRAME);
  task.add(std::move(stage));
}

void add_arm_point_move(
    mtc::Task &task, const std::string &name,
    const std::shared_ptr<mtc::solvers::PipelinePlanner> &planner,
    const geometry_msgs::msg::PointStamped &target) {
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(ARM_GROUP);
  stage->setGoal(target);
  stage->setIKFrame(Eigen::Isometry3d::Identity(), EE_FRAME);
  task.add(std::move(stage));
}

void publish_status(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub,
    const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  pub->publish(msg);
}

// ── Main ────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("z1_pick_cube", options);
  auto log = node->get_logger();

  auto status_pub =
      node->create_publisher<std_msgs::msg::String>("~/status", 10);

  // ── Parameters ──────────────────────────────────────────────────────
  auto approach_offset_x = node->get_parameter("approach_offset_x").as_double();
  auto approach_offset_y = node->get_parameter("approach_offset_y").as_double();
  auto approach_offset_z = node->get_parameter("approach_offset_z").as_double();
  auto grasp_offset = node->get_parameter("grasp_offset").as_double();
  auto lift_height = node->declare_parameter<double>("lift_height", 0.30);
  auto place_lower =
      node->declare_parameter<double>("place_lower_distance", 0.03);

  // Spin in background so TF and MoveIt callbacks are processed
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ── TF setup ──────────────────────────────────────────────────────────
  tf2_ros::Buffer tf_buf(node->get_clock());
  tf_buf.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listen(tf_buf, node, false);

  // ── Validate task plan ────────────────────────────────────────────────
  std::map<std::string, std::string> bin_tag_for_label;
  for (const auto &b : BINS)
    bin_tag_for_label[b.label] = b.tag_frame;

  for (const auto &t : TASKS) {
    if (bin_tag_for_label.find(t.bin_label) == bin_tag_for_label.end()) {
      RCLCPP_ERROR(log, "Task references unknown bin '%s'",
                   t.bin_label.c_str());
      publish_status(status_pub,
                     "ERROR: task references unknown bin " + t.bin_label);
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }
  }

  // ── 1. Wait for all bin AprilTags and save their positions ────────────
  RCLCPP_INFO(log, "=== Waiting for bin AprilTags ===");

  std::map<std::string, Eigen::Vector3d> bin_positions; // label -> world pos
  std::set<std::string> pending_bins;
  for (const auto &b : BINS)
    pending_bins.insert(b.tag_frame);

  while (!pending_bins.empty() && rclcpp::ok()) {
    for (auto it = pending_bins.begin(); it != pending_bins.end();) {
      try {
        if (tf_buf.canTransform(WORLD_FRAME, *it, tf2::TimePointZero,
                                tf2::durationFromSec(0.1))) {
          auto tf =
              tf_buf.lookupTransform(WORLD_FRAME, *it, tf2::TimePointZero);
          auto pos = tf2::transformToEigen(tf).translation();

          std::string label;
          for (const auto &b : BINS) {
            if (b.tag_frame == *it) {
              label = b.label;
              break;
            }
          }
          bin_positions[label] = pos;

          RCLCPP_INFO(log, "FOUND %s [%s] at (%.3f, %.3f, %.3f)",
                      label.c_str(), it->c_str(), pos.x(), pos.y(), pos.z());
          publish_status(status_pub, "FOUND " + label);

          it = pending_bins.erase(it);
          continue;
        }
      } catch (const tf2::TransformException &) {
      }
      ++it;
    }

    if (!pending_bins.empty()) {
      std::string waiting;
      for (const auto &p : pending_bins)
        waiting += (waiting.empty() ? "" : ", ") + p;
      auto msg =
          "WAITING for bins: [" + waiting + "] -- move camera to see them";
      RCLCPP_INFO_THROTTLE(log, *node->get_clock(), 3000, "%s", msg.c_str());
      publish_status(status_pub, msg);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }

  RCLCPP_INFO(log, "=== All %zu bins located ===", BINS.size());
  publish_status(status_pub, "All bins located. Starting tasks.");

  // ── Setup action clients ──────────────────────────────────────────────
  // Isaac Sim's JTC does not report goal completion back to move_group,
  // so task.execute() hangs indefinitely. Instead we send each
  // sub-trajectory directly to the controllers with time-based waits.
  using JTC = control_msgs::action::FollowJointTrajectory;
  using Grip = control_msgs::action::GripperCommand;

  auto jtc_client = rclcpp_action::create_client<JTC>(
      node, "/joint_trajectory_controller/follow_joint_trajectory");
  auto grip_client = rclcpp_action::create_client<Grip>(
      node, "/gripper_controller/gripper_cmd");

  auto settle = node->declare_parameter<double>("move_settle_time", 15.0);
  auto grip_effort =
      node->declare_parameter<double>("gripper_max_effort", 10.0);

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

  // ── 2. Execute pick-place tasks with retry ──────────────────────────
  // Bin hover height is kept low to maximize reach at distant bins.
  // Object approach uses approach_offset_z (typically 0.25m), but bins
  // only need enough clearance to clear the rim.
  auto bin_hover_height =
      node->declare_parameter<double>("bin_hover_height", 0.15);

  std::vector<bool> task_done(TASKS.size(), false);
  int round = 0;

  while (rclcpp::ok()) {
    round++;
    bool any_pending = false;
    bool any_succeeded_this_round = false;

    for (size_t ti = 0; ti < TASKS.size(); ti++) {
      if (task_done[ti])
        continue;
      if (!rclcpp::ok())
        break;
      any_pending = true;

      const auto &tdef = TASKS[ti];
      std::string task_name =
          "pick_" + tdef.object_tag + "_to_" + tdef.bin_label;

      RCLCPP_INFO(log, "");
      RCLCPP_INFO(log, "====== [Round %d] Task %zu/%zu: %s -> %s ======",
                  round, ti + 1, TASKS.size(), tdef.object_tag.c_str(),
                  tdef.bin_label.c_str());
      publish_status(status_pub,
                     "[Round " + std::to_string(round) + "] " + task_name);

      // ── Re-read bin position (user may have moved it closer) ────────
      try {
        auto bin_tf = tf_buf.lookupTransform(
            WORLD_FRAME, bin_tag_for_label.at(tdef.bin_label),
            tf2::TimePointZero);
        bin_positions[tdef.bin_label] =
            tf2::transformToEigen(bin_tf).translation();
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN(log, "Could not re-read TF for %s, using last known pos",
                    tdef.bin_label.c_str());
      }

      // ── Wait for object tag (with timeout so we can try other tasks) ─
      geometry_msgs::msg::TransformStamped obj_tf;
      bool obj_found = false;
      auto obj_deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(15);

      while (std::chrono::steady_clock::now() < obj_deadline &&
             rclcpp::ok()) {
        try {
          if (tf_buf.canTransform(WORLD_FRAME, tdef.object_tag,
                                  tf2::TimePointZero,
                                  tf2::durationFromSec(0.5))) {
            obj_tf = tf_buf.lookupTransform(WORLD_FRAME, tdef.object_tag,
                                            tf2::TimePointZero);
            obj_found = true;
            break;
          }
        } catch (const tf2::TransformException &) {
        }

        auto msg = "WAITING for object: " + tdef.object_tag +
                   " -- move camera to see it";
        RCLCPP_INFO_THROTTLE(log, *node->get_clock(), 3000, "%s",
                             msg.c_str());
        publish_status(status_pub, msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      if (!obj_found) {
        auto msg = "SKIP " + task_name + ": object " + tdef.object_tag +
                   " not visible -- move camera to see it, will retry";
        RCLCPP_WARN(log, "%s", msg.c_str());
        publish_status(status_pub, msg);
        continue;
      }

      RCLCPP_INFO(log, "Object %s detected", tdef.object_tag.c_str());

      // ── Compute pick geometry from object tag ────────────────────────
      Eigen::Isometry3d tag_pose = tf2::transformToEigen(obj_tf);
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

      // ── Compute place geometry from bin position ──────────────────────
      const auto &bin_pos = bin_positions.at(tdef.bin_label);
      Eigen::Vector3d place_above_pos(bin_pos.x(), bin_pos.y(),
                                      bin_pos.z() + bin_hover_height);

      double bin_xy_dist =
          std::sqrt(bin_pos.x() * bin_pos.x() + bin_pos.y() * bin_pos.y());

      RCLCPP_INFO(log, "Object      (%.3f, %.3f, %.3f)", tag_pos.x(),
                  tag_pos.y(), tag_pos.z());
      RCLCPP_INFO(log, "Approach    (%.3f, %.3f, %.3f)  [position-only]",
                  approach_pos.x(), approach_pos.y(), approach_pos.z());
      RCLCPP_INFO(log, "Grasp       (%.3f, %.3f, %.3f)  [full pose]",
                  grasp_pos.x(), grasp_pos.y(), grasp_pos.z());
      RCLCPP_INFO(log, "Bin %s      (%.3f, %.3f, %.3f) [XY dist: %.3f m]",
                  tdef.bin_label.c_str(), bin_pos.x(), bin_pos.y(),
                  bin_pos.z(), bin_xy_dist);
      RCLCPP_INFO(log, "Place-above (%.3f, %.3f, %.3f)  [position-only]",
                  place_above_pos.x(), place_above_pos.y(),
                  place_above_pos.z());

      if (bin_xy_dist > 0.50)
        RCLCPP_WARN(
            log,
            "Bin %s is %.3f m from base -- may be near or beyond reach!",
            tdef.bin_label.c_str(), bin_xy_dist);

      // Approach: full 6-DOF pose — orientation matters to avoid knocking object down.
      auto approach_msg = make_pose(approach_pos, ee_q, WORLD_FRAME);

      // Grasp: full 6-DOF pose — orientation matters here for the grip.
      auto grasp_msg = make_pose(grasp_pos, ee_q, WORLD_FRAME);

      // Bin: position-only — just get above the bin, any orientation.
      geometry_msgs::msg::PointStamped place_point;
      place_point.header.frame_id = WORLD_FRAME;
      place_point.point.x = place_above_pos.x();
      place_point.point.y = place_above_pos.y();
      place_point.point.z = place_above_pos.z();

      // We also need a full pose for the bin approach to prevent the arm from
      // picking a wild orientation (like pointing straight down into the table)
      // while moving to the position-only target.
      // But instead of ee_q (which points straight down), we can use a neutral
      // downward-facing orientation to make it easier for OMPL to find a path
      Eigen::Quaterniond neutral_down_q(
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
      auto place_msg = make_pose(place_above_pos, neutral_down_q, WORLD_FRAME);

      // ── Build MTC task ─────────────────────────────────────────────
      // Planners are created per-task because MTC requires the planner's
      // robot model to be the same instance as the task's robot model.
      mtc::Task task;
      task.stages()->setName(task_name);
      task.loadRobotModel(node);

      auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node);
      ompl->setTimeout(10.0);
      auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian->setStepSize(0.005);
      cartesian->setMaxVelocityScalingFactor(0.5);
      cartesian->setMaxAccelerationScalingFactor(0.5);
      auto joint_interp =
          std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      task.setProperty("group", ARM_GROUP);
      task.setProperty("eef", EE_NAME);
      task.setProperty("ik_frame", EE_FRAME);

      // ── PICK ────────────────────────────────────────────────────────
      task.add(std::make_unique<mtc::stages::CurrentState>("current"));

      add_gripper_move(task, "open gripper", joint_interp, "open");

      add_arm_pose_move(task, "approach object", ompl, approach_msg);

      add_arm_pose_move(task, "move to grasp", ompl, grasp_msg);

      add_gripper_move(task, "close gripper", joint_interp, "closed");

      add_cartesian_move(task, "lift", cartesian, Eigen::Vector3d::UnitZ(),
                         lift_height * 0.3, lift_height);

      // ── PLACE ───────────────────────────────────────────────────────
      {
        auto stage =
            std::make_unique<mtc::stages::MoveTo>("move above bin", ompl);
        stage->setGroup(ARM_GROUP);
        stage->setGoal(place_point);
        stage->setIKFrame(Eigen::Isometry3d::Identity(), EE_FRAME);
        task.add(std::move(stage));
      }

      add_cartesian_move(task, "lower into bin", cartesian,
                         -Eigen::Vector3d::UnitZ(), 0.01, place_lower);

      add_gripper_move(task, "release", joint_interp, "open");

      add_cartesian_move(task, "retreat", cartesian, Eigen::Vector3d::UnitZ(),
                         0.01, place_lower);

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("home", ompl);
        stage->setGroup(ARM_GROUP);
        stage->setGoal("home");
        task.add(std::move(stage));
      }

      // ── Plan ────────────────────────────────────────────────────────
      try {
        task.init();
      } catch (const mtc::InitStageException &e) {
        RCLCPP_ERROR_STREAM(log, "Task init failed: " << e);
        auto msg = "SKIP " + task_name + ": init error, will retry";
        publish_status(status_pub, msg);
        continue;
      }

      RCLCPP_INFO(log, "Planning %s (up to 10 solutions) ...",
                  task_name.c_str());
      publish_status(status_pub, "Planning: " + task_name);

      if (!task.plan(10)) {
        auto msg = "PLAN FAILED: " + task_name + " [bin XY dist: " +
                   std::to_string(bin_xy_dist).substr(0, 5) +
                   "m] -- move " + tdef.bin_label +
                   " closer to robot, will retry";
        RCLCPP_WARN(log, "%s", msg.c_str());
        publish_status(status_pub, msg);
        continue;
      }

      auto &solution = *task.solutions().front();
      task.introspection().publishSolution(solution);

      // ── Execute via direct controller calls ──────────────────────────
      // Isaac Sim's JTC does not report goal completion back to move_group,
      // so task.execute() hangs indefinitely. Instead we send each
      // sub-trajectory directly to the controllers with time-based waits.
      moveit_task_constructor_msgs::msg::Solution sol_msg;
      solution.toMsg(sol_msg);

      RCLCPP_INFO(log, "Executing %s (%zu sub-trajectories) ...",
                  task_name.c_str(), sol_msg.sub_trajectory.size());
      publish_status(status_pub, "Executing: " + task_name);

      rclcpp_action::ClientGoalHandle<JTC>::SharedPtr active_jtc_handle;
      int step = 0;
      bool task_ok = true;

      for (const auto &sub : sol_msg.sub_trajectory) {
        if (!rclcpp::ok()) {
          task_ok = false;
          break;
        }

        auto &jt = sub.trajectory.joint_trajectory;
        if (jt.joint_names.empty() || jt.points.empty())
          continue;
        step++;

        bool is_gripper = (jt.joint_names.size() == 1 &&
                           jt.joint_names[0] == "jointGripper");

        auto step_msg = "[" + std::to_string(step) + "/" +
                        std::to_string(sol_msg.sub_trajectory.size()) +
                        "] " + task_name;

        if (is_gripper) {
          double target = jt.points.back().positions[0];
          RCLCPP_INFO(log, "[%d] Gripper -> %.3f rad", step, target);
          publish_status(status_pub, step_msg + " gripper->" +
                                        std::to_string(target).substr(0, 5));

          Grip::Goal goal;
          goal.command.position = target;
          goal.command.max_effort = grip_effort;

          auto future = grip_client->async_send_goal(goal);
          if (future.wait_for(std::chrono::seconds(3)) !=
                  std::future_status::ready ||
              !future.get()) {
            RCLCPP_ERROR(log, "[%d] Gripper goal rejected", step);
            task_ok = false;
            break;
          }
          RCLCPP_INFO(log, "[%d] Gripper goal accepted", step);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else {
          // Cancel any lingering JTC goal before sending a new one
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
          publish_status(status_pub,
                         step_msg + " arm " + std::to_string(jt.points.size()) +
                             "pts " + std::to_string(duration).substr(0, 4) + "s");

          JTC::Goal goal;
          goal.trajectory = jt;

          auto future = jtc_client->async_send_goal(goal);
          if (future.wait_for(std::chrono::seconds(5)) !=
                  std::future_status::ready ||
              !future.get()) {
            RCLCPP_ERROR(log, "[%d] JTC goal rejected", step);
            task_ok = false;
            break;
          }
          active_jtc_handle = future.get();
          RCLCPP_INFO(log, "[%d] JTC goal accepted, waiting %.1f s", step,
                      std::max(duration + 1.0, settle));
          std::this_thread::sleep_for(
              std::chrono::duration<double>(std::max(duration + 1.0, settle)));
        }
      }

      if (task_ok) {
        RCLCPP_INFO(log, "DONE: %s", task_name.c_str());
        publish_status(status_pub, "DONE: " + task_name);
        task_done[ti] = true;
        any_succeeded_this_round = true;
      } else {
        auto msg =
            "EXEC FAILED: " + task_name + " at step " +
            std::to_string(step) +
            " -- will retry (robot may need to be homed manually)";
        RCLCPP_ERROR(log, "%s", msg.c_str());
        publish_status(status_pub, msg);
      }
    } // end for each task

    if (!any_pending)
      break; // all tasks done

    if (!rclcpp::ok())
      break;

    // Summarize remaining tasks
    std::string remaining;
    for (size_t ti = 0; ti < TASKS.size(); ti++) {
      if (!task_done[ti])
        remaining +=
            (remaining.empty() ? "" : ", ") + TASKS[ti].object_tag +
            "->" + TASKS[ti].bin_label;
    }

    if (!any_succeeded_this_round) {
      auto msg = "No progress this round. Remaining: [" + remaining +
                 "]. Move bins/objects closer to robot. "
                 "Retrying in 10s...";
      RCLCPP_WARN(log, "%s", msg.c_str());
      publish_status(status_pub, msg);
      std::this_thread::sleep_for(std::chrono::seconds(10));
    } else {
      auto msg = "Retrying remaining: [" + remaining + "]";
      RCLCPP_INFO(log, "%s", msg.c_str());
      publish_status(status_pub, msg);
    }
  } // end while retry loop

  // ── Shutdown ──────────────────────────────────────────────────────────
  bool all_done = true;
  for (bool d : task_done)
    if (!d)
      all_done = false;

  if (all_done) {
    RCLCPP_INFO(log, "=== All %zu tasks complete! ===", TASKS.size());
    publish_status(status_pub, "ALL TASKS COMPLETE");
  } else {
    RCLCPP_WARN(log, "Shutting down with incomplete tasks");
    publish_status(status_pub, "SHUTDOWN with incomplete tasks");
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  // Stop the executor before rclcpp::shutdown() to avoid double-free in
  // MTC's internal node cleanup racing with the still-spinning executor.
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return all_done ? 0 : 1;
}
