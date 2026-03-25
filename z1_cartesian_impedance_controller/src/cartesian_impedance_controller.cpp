#include "z1_cartesian_impedance_controller/cartesian_impedance_controller.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <unitree_arm_sdk/model/ArmModel.h>

#include "rclcpp/logging.hpp"
#include <pluginlib/class_list_macros.hpp>

using z1::CartesianImpedanceController;

// ─────────────────────────────────────────────────────────────────────────────
//  ros2_control interface configuration
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_init — declare parameters
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints",
                                           std::vector<std::string>{});

    auto_declare<double>("stiffness.translation_x", 500.0);
    auto_declare<double>("stiffness.translation_y", 500.0);
    auto_declare<double>("stiffness.translation_z", 500.0);
    auto_declare<double>("stiffness.rotation_x", 50.0);
    auto_declare<double>("stiffness.rotation_y", 50.0);
    auto_declare<double>("stiffness.rotation_z", 50.0);

    auto_declare<double>("damping_factor.translation_x", 1.0);
    auto_declare<double>("damping_factor.translation_y", 1.0);
    auto_declare<double>("damping_factor.translation_z", 1.0);
    auto_declare<double>("damping_factor.rotation_x", 1.0);
    auto_declare<double>("damping_factor.rotation_y", 1.0);
    auto_declare<double>("damping_factor.rotation_z", 1.0);

    auto_declare<double>("delta_tau_max", 5.0);
    auto_declare<double>("filtering.stiffness", 0.99);
    auto_declare<double>("filtering.pose", 0.99);
    auto_declare<double>("filtering.wrench", 0.99);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_configure — read parameters, create model & subscriptions
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
  auto logger = get_node()->get_logger();

  // --- Joint names ---
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.size() != NUM_JOINTS) {
    RCLCPP_ERROR(logger, "Expected %zu joints, got %zu", NUM_JOINTS,
                 joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- Stiffness ---
  cartesian_stiffness_target_.setZero();
  cartesian_stiffness_target_(0, 0) =
      get_node()->get_parameter("stiffness.translation_x").as_double();
  cartesian_stiffness_target_(1, 1) =
      get_node()->get_parameter("stiffness.translation_y").as_double();
  cartesian_stiffness_target_(2, 2) =
      get_node()->get_parameter("stiffness.translation_z").as_double();
  cartesian_stiffness_target_(3, 3) =
      get_node()->get_parameter("stiffness.rotation_x").as_double();
  cartesian_stiffness_target_(4, 4) =
      get_node()->get_parameter("stiffness.rotation_y").as_double();
  cartesian_stiffness_target_(5, 5) =
      get_node()->get_parameter("stiffness.rotation_z").as_double();
  cartesian_stiffness_ = cartesian_stiffness_target_;

  // --- Damping factors ---
  damping_factors_(0) =
      get_node()->get_parameter("damping_factor.translation_x").as_double();
  damping_factors_(1) =
      get_node()->get_parameter("damping_factor.translation_y").as_double();
  damping_factors_(2) =
      get_node()->get_parameter("damping_factor.translation_z").as_double();
  damping_factors_(3) =
      get_node()->get_parameter("damping_factor.rotation_x").as_double();
  damping_factors_(4) =
      get_node()->get_parameter("damping_factor.rotation_y").as_double();
  damping_factors_(5) =
      get_node()->get_parameter("damping_factor.rotation_z").as_double();
  apply_damping();
  cartesian_damping_ = cartesian_damping_target_;

  // --- Torque and filtering ---
  delta_tau_max_ = get_node()->get_parameter("delta_tau_max").as_double();
  filter_stiffness_ =
      get_node()->get_parameter("filtering.stiffness").as_double();
  filter_pose_ = get_node()->get_parameter("filtering.pose").as_double();
  filter_wrench_ = get_node()->get_parameter("filtering.wrench").as_double();

  // --- SDK dynamics model ---
  model_ = std::make_unique<UNITREE_ARM::Z1Model>();
  RCLCPP_INFO(logger,
              "Z1 SDK dynamics model initialised (6-DOF, no nullspace)");

  // --- Subscribers ---
  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/reference_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        position_d_target_ << msg->pose.position.x, msg->pose.position.y,
            msg->pose.position.z;
        orientation_d_target_.coeffs() << msg->pose.orientation.x,
            msg->pose.orientation.y, msg->pose.orientation.z,
            msg->pose.orientation.w;
        orientation_d_target_.normalize();
      });
  wrench_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "~/commanded_wrench", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            cartesian_wrench_target_ << msg->wrench.force.x,
                msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
                msg->wrench.torque.y, msg->wrench.torque.z;
          });

  // --- Parameter callback for runtime stiffness/damping changes ---
  auto param_cb = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params)
          -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
          if (p.get_name() == "stiffness.translation_x")
            cartesian_stiffness_target_(0, 0) = p.as_double();
          else if (p.get_name() == "stiffness.translation_y")
            cartesian_stiffness_target_(1, 1) = p.as_double();
          else if (p.get_name() == "stiffness.translation_z")
            cartesian_stiffness_target_(2, 2) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_x")
            cartesian_stiffness_target_(3, 3) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_y")
            cartesian_stiffness_target_(4, 4) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_z")
            cartesian_stiffness_target_(5, 5) = p.as_double();

          else if (p.get_name() == "damping_factor.translation_x")
            damping_factors_(0) = p.as_double();
          else if (p.get_name() == "damping_factor.translation_y")
            damping_factors_(1) = p.as_double();
          else if (p.get_name() == "damping_factor.translation_z")
            damping_factors_(2) = p.as_double();
          else if (p.get_name() == "damping_factor.rotation_x")
            damping_factors_(3) = p.as_double();
          else if (p.get_name() == "damping_factor.rotation_y")
            damping_factors_(4) = p.as_double();
          else if (p.get_name() == "damping_factor.rotation_z")
            damping_factors_(5) = p.as_double();
        }
        apply_damping();
        return result;
      });

  // -- Debug publishers (decimated to ~100 Hz) --
  const auto debug_qos = rclcpp::SensorDataQoS();
  debug_reference_gap_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/reference_gap", debug_qos);
  debug_tracking_error_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/tracking_error", debug_qos);
  debug_torque_desired_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/torque_desired", debug_qos);
  debug_torque_commanded_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/torque_commanded", debug_qos);
  debug_ee_velocity_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/ee_velocity", debug_qos);

  RCLCPP_INFO(logger, "Cartesian impedance controller configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_activate — set reference to current pose
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
  read_joint_states();
  update_kinematics();

  // Lock the reference to the current pose so the robot doesn't jump
  position_d_ = position_;
  position_d_target_ = position_;
  orientation_d_ = orientation_;
  orientation_d_target_ = orientation_;

  cartesian_wrench_.setZero();
  cartesian_wrench_target_.setZero();
  tau_c_.setZero();

  RCLCPP_INFO(
      get_node()->get_logger(),
      "Cartesian impedance controller activated — holding current pose");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_deactivate
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Cartesian impedance controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  update() — the main control loop, called at update_rate Hz
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
) {
  read_joint_states();
  update_kinematics();
  apply_filtering();

  Vec6 tau_d = compute_torques();
  saturate_torque_rate(tau_d, tau_c_, delta_tau_max_);

  if (++debug_counter_ % DEBUG_DECIMATION == 0) {
    publish_debug(tau_d);
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    [[maybe_unused]] bool ok =
        command_interfaces_[i].set_value(tau_c_(static_cast<long>(i)));
  }
  return controller_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  State reading from ros2_control interfaces
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::read_joint_states() {
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    const auto q_opt = state_interfaces_[2 * i].get_optional();
    const auto dq_opt = state_interfaces_[2 * i + 1].get_optional();
    if (q_opt)
      q_(static_cast<long>(i)) = *q_opt;
    if (dq_opt)
      dq_(static_cast<long>(i)) = *dq_opt;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Kinematics via Unitree SDK
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::update_kinematics() {
  // -- Jacobian (SDK returns spatial Jacobian in [angular; linear] order) --
  Eigen::Matrix<double, 6, 6> J_sdk = model_->CalcJacobian(q_);

  // Permute to [linear; angular] convention
  jacobian_.topRows<3>() = J_sdk.bottomRows<3>();
  jacobian_.bottomRows<3>() = J_sdk.topRows<3>();

  // -- Forward kinematics --
  Eigen::Matrix4d T = model_->forwardKinematics(q_, 6);
  position_ = T.block<3, 1>(0, 3);
  orientation_ = Eigen::Quaterniond(T.block<3, 3>(0, 0)).normalized();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filtering (exponential smoothing on targets → actives)
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::apply_filtering() {
  const double step_k = filter_step(update_frequency_, filter_stiffness_);
  const double step_p = filter_step(update_frequency_, filter_pose_);
  const double step_w = filter_step(update_frequency_, filter_wrench_);

  // Stiffness + damping
  cartesian_stiffness_ = filtered_update(cartesian_stiffness_target_,
                                         cartesian_stiffness_, step_k);
  cartesian_damping_ =
      filtered_update(cartesian_damping_target_, cartesian_damping_, step_k);

  // Pose
  position_d_ = filtered_update(position_d_target_, position_d_, step_p);
  orientation_d_ = orientation_d_.slerp(step_p, orientation_d_target_);

  // Wrench
  cartesian_wrench_ =
      filtered_update(cartesian_wrench_target_, cartesian_wrench_, step_w);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core control law
// ─────────────────────────────────────────────────────────────────────────────

CartesianImpedanceController::Vec6
CartesianImpedanceController::compute_torques() {
  // 6D pose error
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = position_ - position_d_;
  error.tail<3>() = orientation_error(orientation_d_, orientation_);

  // Cartesian PD: tau_task = J^T (-K e - D J dq)
  Vec6 tau_task =
      jacobian_.transpose() *
      (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian_ * dq_));

  // Wrench feedforward: tau_wrench = J^T F_cmd
  Vec6 tau_wrench = jacobian_.transpose() * cartesian_wrench_;

  // Gravity compensation via SDK inverse dynamics
  Vec6 gravity =
      model_->inverseDynamics(q_, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());

  return tau_task + tau_wrench + gravity;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Orientation error (quaternion short-path → angle-axis)
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d CartesianImpedanceController::orientation_error(
    const Eigen::Quaterniond &desired, Eigen::Quaterniond current) {
  // Ensure shortest path
  if (desired.coeffs().dot(current.coeffs()) < 0.0) {
    current.coeffs() = -current.coeffs();
  }
  const Eigen::Quaterniond q_err(current * desired.inverse());
  Eigen::AngleAxisd aa(q_err);
  return aa.axis() * aa.angle();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Torque rate saturation
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::saturate_torque_rate(const Vec6 &desired,
                                                        Vec6 &previous,
                                                        double delta_max) {
  for (long i = 0; i < static_cast<long>(NUM_JOINTS); ++i) {
    const double diff = desired(i) - previous(i);
    previous(i) += std::clamp(diff, -delta_max, delta_max);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Damping: d_i = alpha_i * 2 * sqrt(k_i)
// ─────────────────────────────────────────────────────────────────────────────

double CartesianImpedanceController::damping_rule(double stiffness) {
  return 2.0 * std::sqrt(stiffness);
}

void CartesianImpedanceController::apply_damping() {
  for (int i = 0; i < 6; ++i) {
    cartesian_damping_target_(i, i) =
        damping_factors_(i) * damping_rule(cartesian_stiffness_target_(i, i));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filter helpers (same formula as Mayr et al.)
// ─────────────────────────────────────────────────────────────────────────────

double CartesianImpedanceController::filter_step(double update_frequency,
                                                 double filter_percentage) {
  const double safe_pct = std::min(filter_percentage, 0.999999);
  const double kappa = -1.0 / std::log(1.0 - safe_pct);
  return 1.0 / (kappa * update_frequency + 1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Debug publishing (runs at update_rate / DEBUG_DECIMATION)
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::publish_debug(const Vec6 &tau_d) {
  auto make_msg = [](const double *data, size_t n) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(data, data + n);
    return msg;
  };

  // How far the filtered reference lags behind the commanded target.
  // Large values here → the pose filter is the speed bottleneck.
  {
    Eigen::Vector3d pos_gap = position_d_target_ - position_d_;
    Eigen::Vector3d rot_gap =
        orientation_error(orientation_d_target_, orientation_d_);
    Eigen::Matrix<double, 6, 1> gap;
    gap << pos_gap, rot_gap;
    debug_reference_gap_pub_->publish(make_msg(gap.data(), 6));
  }

  // How well the arm tracks the (filtered) reference.
  // Large values here → stiffness too low or hardware torque limits hit.
  {
    Eigen::Matrix<double, 6, 1> err;
    err.head<3>() = position_ - position_d_;
    err.tail<3>() = orientation_error(orientation_d_, orientation_);
    debug_tracking_error_pub_->publish(make_msg(err.data(), 6));
  }

  // Torque before vs after rate limiting.
  // If these diverge, delta_tau_max is clipping.
  debug_torque_desired_pub_->publish(make_msg(tau_d.data(), NUM_JOINTS));
  debug_torque_commanded_pub_->publish(make_msg(tau_c_.data(), NUM_JOINTS));

  // End-effector Cartesian velocity [linear(3); angular(3)].
  {
    Eigen::Matrix<double, 6, 1> ee_vel = jacobian_ * dq_;
    debug_ee_velocity_pub_->publish(make_msg(ee_vel.data(), 6));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Plugin export
// ─────────────────────────────────────────────────────────────────────────────

PLUGINLIB_EXPORT_CLASS(z1::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
