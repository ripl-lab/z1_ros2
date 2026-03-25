#include "z1_operational_impedance_controller/operational_impedance_controller.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

#include <unitree_arm_sdk/model/ArmModel.h>

#include "rclcpp/logging.hpp"
#include <pluginlib/class_list_macros.hpp>

using z1::OperationalImpedanceController;

// ─────────────────────────────────────────────────────────────────────────────
//  ros2_control interface configuration
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
OperationalImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
OperationalImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_init — declare parameters
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn OperationalImpedanceController::on_init() {
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

    auto_declare<bool>("use_inertia_shaping", false);
    auto_declare<double>("desired_inertia.translation_x", 1.0);
    auto_declare<double>("desired_inertia.translation_y", 1.0);
    auto_declare<double>("desired_inertia.translation_z", 1.0);
    auto_declare<double>("desired_inertia.rotation_x", 0.1);
    auto_declare<double>("desired_inertia.rotation_y", 0.1);
    auto_declare<double>("desired_inertia.rotation_z", 0.1);

    auto_declare<double>("lambda_damping", 0.005);
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

controller_interface::CallbackReturn
OperationalImpedanceController::on_configure(
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

  // --- Stiffness K_m ---
  stiffness_target_.setZero();
  stiffness_target_(0, 0) =
      get_node()->get_parameter("stiffness.translation_x").as_double();
  stiffness_target_(1, 1) =
      get_node()->get_parameter("stiffness.translation_y").as_double();
  stiffness_target_(2, 2) =
      get_node()->get_parameter("stiffness.translation_z").as_double();
  stiffness_target_(3, 3) =
      get_node()->get_parameter("stiffness.rotation_x").as_double();
  stiffness_target_(4, 4) =
      get_node()->get_parameter("stiffness.rotation_y").as_double();
  stiffness_target_(5, 5) =
      get_node()->get_parameter("stiffness.rotation_z").as_double();
  stiffness_ = stiffness_target_;

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
  damping_ = damping_target_;

  // --- Inertia shaping ---
  use_inertia_shaping_ =
      get_node()->get_parameter("use_inertia_shaping").as_bool();
  desired_inertia_.setZero();
  desired_inertia_(0, 0) =
      get_node()->get_parameter("desired_inertia.translation_x").as_double();
  desired_inertia_(1, 1) =
      get_node()->get_parameter("desired_inertia.translation_y").as_double();
  desired_inertia_(2, 2) =
      get_node()->get_parameter("desired_inertia.translation_z").as_double();
  desired_inertia_(3, 3) =
      get_node()->get_parameter("desired_inertia.rotation_x").as_double();
  desired_inertia_(4, 4) =
      get_node()->get_parameter("desired_inertia.rotation_y").as_double();
  desired_inertia_(5, 5) =
      get_node()->get_parameter("desired_inertia.rotation_z").as_double();
  desired_inertia_inv_.setZero();
  for (int i = 0; i < 6; ++i) {
    desired_inertia_inv_(i, i) = 1.0 / desired_inertia_(i, i);
  }

  // --- Torque and filtering ---
  lambda_damping_ = get_node()->get_parameter("lambda_damping").as_double();
  delta_tau_max_ = get_node()->get_parameter("delta_tau_max").as_double();
  filter_stiffness_ =
      get_node()->get_parameter("filtering.stiffness").as_double();
  filter_pose_ = get_node()->get_parameter("filtering.pose").as_double();
  filter_wrench_ = get_node()->get_parameter("filtering.wrench").as_double();

  // --- SDK dynamics model ---
  model_ = std::make_unique<UNITREE_ARM::Z1Model>();
  RCLCPP_INFO(logger, "Z1 SDK dynamics model initialised (6-DOF)");

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

  // Desired end-effector velocity [linear(3); angular(3)]
  twist_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          "~/reference_twist", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            x_dot_d_ << msg->twist.linear.x, msg->twist.linear.y,
                msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
                msg->twist.angular.z;
          });

  // External/reaction wrench F_r [force(3); torque(3)]
  wrench_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "~/external_wrench", rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            F_ext_target_ << msg->wrench.force.x, msg->wrench.force.y,
                msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
                msg->wrench.torque.z;
          });

  // --- Parameter callback for runtime stiffness/damping changes ---
  auto param_cb = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params)
          -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
          if (p.get_name() == "stiffness.translation_x")
            stiffness_target_(0, 0) = p.as_double();
          else if (p.get_name() == "stiffness.translation_y")
            stiffness_target_(1, 1) = p.as_double();
          else if (p.get_name() == "stiffness.translation_z")
            stiffness_target_(2, 2) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_x")
            stiffness_target_(3, 3) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_y")
            stiffness_target_(4, 4) = p.as_double();
          else if (p.get_name() == "stiffness.rotation_z")
            stiffness_target_(5, 5) = p.as_double();

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
  debug_lambda_diag_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/lambda_diagonal", debug_qos);
  debug_tau_ext_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/tau_ext", debug_qos);
  debug_F_ext_hat_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/debug/F_ext_hat", debug_qos);

  RCLCPP_INFO(logger,
              "Operational-space impedance controller configured "
              "(inertia_shaping=%s)",
              use_inertia_shaping_ ? "ON" : "OFF");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_activate — set reference to current pose
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn
OperationalImpedanceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
  read_joint_states();
  update_kinematics();

  // Lock the reference to the current pose so the robot doesn't jump
  position_d_ = position_;
  position_d_target_ = position_;
  orientation_d_ = orientation_;
  orientation_d_target_ = orientation_;

  x_dot_d_.setZero();
  F_ext_.setZero();
  F_ext_target_.setZero();
  tau_c_.setZero();

  RCLCPP_INFO(get_node()->get_logger(),
              "Operational-space impedance controller activated — holding "
              "current pose");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle: on_deactivate
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn
OperationalImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/
) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Operational-space impedance controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  update() — the main control loop, called at update_rate Hz
// ─────────────────────────────────────────────────────────────────────────────

controller_interface::return_type OperationalImpedanceController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
) {
  read_joint_states();
  update_kinematics();
  update_dynamics();
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

void OperationalImpedanceController::read_joint_states() {
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    const auto q_opt = state_interfaces_[3 * i].get_optional();
    const auto dq_opt = state_interfaces_[3 * i + 1].get_optional();
    const auto tau_opt = state_interfaces_[3 * i + 2].get_optional();
    if (q_opt)
      q_(static_cast<long>(i)) = *q_opt;
    if (dq_opt)
      dq_(static_cast<long>(i)) = *dq_opt;
    if (tau_opt)
      tau_measured_(static_cast<long>(i)) = *tau_opt;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Kinematics via Unitree SDK
// ─────────────────────────────────────────────────────────────────────────────

void OperationalImpedanceController::update_kinematics() {
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
//  Joint-space dynamics via RNEA
// ─────────────────────────────────────────────────────────────────────────────

void OperationalImpedanceController::update_dynamics() {
  compute_mass_matrix();
  compute_operational_space_dynamics();
}

void OperationalImpedanceController::compute_mass_matrix() {
  // g(q) = ID(q, 0, 0, 0)
  gravity_vec_ =
      model_->inverseDynamics(q_, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());

  // c(q, dq) = ID(q, dq, 0, 0) − g(q)
  coriolis_ = model_->inverseDynamics(q_, dq_, Vec6::Zero(), Vec6::Zero()) -
              gravity_vec_;

  // M(q) column by column: M(:,i) = ID(q, 0, e_i, 0) − g(q)
  for (int i = 0; i < 6; ++i) {
    Vec6 unit = Vec6::Zero();
    unit(i) = 1.0;
    mass_matrix_.col(i) =
        model_->inverseDynamics(q_, Vec6::Zero(), unit, Vec6::Zero()) -
        gravity_vec_;
  }

  // M is symmetric positive definite — use Cholesky for inversion
  Eigen::LLT<Mat6> llt(mass_matrix_);
  mass_matrix_inv_ = llt.solve(Mat6::Identity());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Operational-space dynamics: Λ, μ, p
// ─────────────────────────────────────────────────────────────────────────────

void OperationalImpedanceController::compute_operational_space_dynamics() {
  // Λ = (J M⁻¹ Jᵀ)⁻¹ with small damping for near-singularity robustness
  Mat6 JMinvJt = jacobian_ * mass_matrix_inv_ * jacobian_.transpose();
  JMinvJt.diagonal().array() += lambda_damping_;
  lambda_ = JMinvJt.inverse();

  // J̇ q̇ via numerical finite difference of J
  Eigen::Matrix<double, 6, 1> J_dot_dq = compute_J_dot_dq();

  // μ = Λ (J M⁻¹ c − J̇ q̇)
  mu_ = lambda_ * (jacobian_ * mass_matrix_inv_ * coriolis_ - J_dot_dq);

  // p = Λ J M⁻¹ g  (equivalent to J⁻ᵀ g for square non-singular J)
  p_ = lambda_ * jacobian_ * mass_matrix_inv_ * gravity_vec_;
}

Eigen::Matrix<double, 6, 1> OperationalImpedanceController::compute_J_dot_dq() {
  constexpr double eps = 1e-7;
  Vec6 q_perturbed = q_ + dq_ * eps;

  // Jacobian at the perturbed configuration
  Eigen::Matrix<double, 6, 6> J_sdk_p = model_->CalcJacobian(q_perturbed);
  Mat6 J_perturbed;
  J_perturbed.topRows<3>() = J_sdk_p.bottomRows<3>();
  J_perturbed.bottomRows<3>() = J_sdk_p.topRows<3>();

  // J̇ q̇ ≈ [(J(q+ε q̇) − J(q)) / ε] q̇
  return ((J_perturbed - jacobian_) / eps) * dq_;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filtering (exponential smoothing on targets → actives)
// ─────────────────────────────────────────────────────────────────────────────

void OperationalImpedanceController::apply_filtering() {
  const double step_k = filter_step(update_frequency_, filter_stiffness_);
  const double step_p = filter_step(update_frequency_, filter_pose_);
  const double step_w = filter_step(update_frequency_, filter_wrench_);

  // Stiffness + damping
  stiffness_ = filtered_update(stiffness_target_, stiffness_, step_k);
  damping_ = filtered_update(damping_target_, damping_, step_k);

  // Pose
  position_d_ = filtered_update(position_d_target_, position_d_, step_p);
  orientation_d_ = orientation_d_.slerp(step_p, orientation_d_target_);

  // External wrench
  F_ext_ = filtered_update(F_ext_target_, F_ext_, step_w);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core control law — operational-space impedance
//
//  Two-stage formulation (Khatib 1987):
//    a_cmd = ẍ_d + M_m⁻¹ [ D_m (ẋ_d − ẋ) + K_m (x_d − x) + F_r ]
//    τ     = Jᵀ  [ Λ a_cmd + μ + p − F_r ]
//
//  Natural-inertia case (M_m = Λ): F_r cancels in the torque equation.
//    τ = Jᵀ [ D_m (ẋ_d − ẋ) + K_m (x_d − x) + μ + p ]
// ─────────────────────────────────────────────────────────────────────────────

OperationalImpedanceController::Vec6
OperationalImpedanceController::compute_torques() {
  // End-effector velocity ẋ = J q̇
  Eigen::Matrix<double, 6, 1> x_dot = jacobian_ * dq_;

  // 6D pose error: e = x − x_d
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = position_ - position_d_;
  error.tail<3>() = orientation_error(orientation_d_, orientation_);

  // Velocity error: ẋ − ẋ_d
  Eigen::Matrix<double, 6, 1> vel_error = x_dot - x_dot_d_;

  Eigen::Matrix<double, 6, 1> F_task;

  if (!use_inertia_shaping_) {
    // ── Natural inertia: M_m = Λ(q) ──
    //
    // The F_r terms cancel analytically, giving:
    //   τ = Jᵀ [ −D_m (ẋ − ẋ_d) − K_m (x − x_d) + μ + p ]
    //
    // Closed-loop impedance: Λ ẍ + D_m ẋ_err + K_m x_err = F_ext
    F_task = -damping_ * vel_error - stiffness_ * error + mu_ + p_;
  } else {
    // ── Custom inertia M_m ──
    //
    // a_cmd = M_m⁻¹ [ D_m (ẋ_d − ẋ) + K_m (x_d − x) + F_r ]
    // τ = Jᵀ [ Λ a_cmd + μ + p + (Λ M_m⁻¹ − I) F_r ]
    Eigen::Matrix<double, 6, 1> impedance =
        -damping_ * vel_error - stiffness_ * error + F_ext_;
    Eigen::Matrix<double, 6, 1> a_cmd = desired_inertia_inv_ * impedance;
    F_task = lambda_ * a_cmd + mu_ + p_ +
             (lambda_ * desired_inertia_inv_ - Mat6::Identity()) * F_ext_;
  }

  return jacobian_.transpose() * F_task;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Orientation error (quaternion short-path → angle-axis)
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d OperationalImpedanceController::orientation_error(
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

void OperationalImpedanceController::saturate_torque_rate(const Vec6 &desired,
                                                          Vec6 &previous,
                                                          double delta_max) {
  for (long i = 0; i < static_cast<long>(NUM_JOINTS); ++i) {
    const double diff = desired(i) - previous(i);
    previous(i) += std::clamp(diff, -delta_max, delta_max);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Damping: D_i = alpha_i * 2 * sqrt(K_i)
// ─────────────────────────────────────────────────────────────────────────────

double OperationalImpedanceController::damping_rule(double stiffness) {
  return 2.0 * std::sqrt(stiffness);
}

void OperationalImpedanceController::apply_damping() {
  for (int i = 0; i < 6; ++i) {
    damping_target_(i, i) =
        damping_factors_(i) * damping_rule(stiffness_target_(i, i));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filter helpers (same formula as Mayr et al.)
// ─────────────────────────────────────────────────────────────────────────────

double OperationalImpedanceController::filter_step(double update_frequency,
                                                   double filter_percentage) {
  if (filter_percentage >= 1.0) return 1.0;
  const double kappa = -1.0 / std::log(1.0 - filter_percentage);
  return 1.0 / (kappa * update_frequency + 1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Debug publishing (runs at update_rate / DEBUG_DECIMATION)
// ─────────────────────────────────────────────────────────────────────────────

void OperationalImpedanceController::publish_debug(const Vec6 &tau_d) {
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

  // Diagonal of Λ — shows effective task-space inertia per axis.
  {
    Eigen::Matrix<double, 6, 1> lambda_diag = lambda_.diagonal();
    debug_lambda_diag_pub_->publish(make_msg(lambda_diag.data(), 6));
  }

  // Estimated external wrench (quasi-static):
  //   τ_ext = τ_measured − c(q,q̇) − g(q)
  //   F̂_ext = J⁻ᵀ τ_ext  (= Λ J M⁻¹ τ_ext for the dynamically consistent version)
  {
    Vec6 tau_ext = tau_measured_ - coriolis_ - gravity_vec_;
    debug_tau_ext_pub_->publish(make_msg(tau_ext.data(), NUM_JOINTS));

    Eigen::Matrix<double, 6, 1> F_ext_hat =
        lambda_ * jacobian_ * mass_matrix_inv_ * tau_ext;
    debug_F_ext_hat_pub_->publish(make_msg(F_ext_hat.data(), 6));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Plugin export
// ─────────────────────────────────────────────────────────────────────────────

PLUGINLIB_EXPORT_CLASS(z1::OperationalImpedanceController,
                       controller_interface::ControllerInterface)
