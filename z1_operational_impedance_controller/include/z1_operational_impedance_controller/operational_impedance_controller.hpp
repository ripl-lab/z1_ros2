/*
 * Copyright 2025 IDRA, University of Trento
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef Z1_OPERATIONAL_IMPEDANCE_CONTROLLER_HPP__
#define Z1_OPERATIONAL_IMPEDANCE_CONTROLLER_HPP__

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace UNITREE_ARM {
class Z1Model;
}

namespace z1 {

/// Operational-space impedance controller for the Unitree Z1 (6-DOF).
///
/// Implements the two-stage impedance control law from Khatib's
/// operational-space framework with full inverse dynamics cancellation:
///
///   1. Desired impedance:
///      M_m (ẍ − ẍ_d) + D_m (ẋ − ẋ_d) + K_m (x − x_d) = F_r
///
///   2. Commanded task-space acceleration:
///      a_cmd = ẍ_d + M_m⁻¹ [ D_m (ẋ_d − ẋ) + K_m (x_d − x) + F_r ]
///
///   3. Torque via task-space inverse dynamics:
///      τ = Jᵀ [ Λ a_cmd + μ + p − F_r ]
///
/// where:
///   Λ  = (J M⁻¹ Jᵀ)⁻¹          operational-space inertia
///   μ  = Λ (J M⁻¹ c − J̇ q̇)     task-space Coriolis/centrifugal
///   p  = Λ J M⁻¹ g              task-space gravity
///   M  = joint-space mass matrix (extracted from SDK RNEA)
///   c  = C(q,q̇)q̇               Coriolis vector
///   g  = gravity vector
///
/// When use_inertia_shaping is false (default), M_m = Λ(q) and the
/// external force terms cancel, giving the natural-inertia form:
///   τ = Jᵀ [ D_m (ẋ_d − ẋ) + K_m (x_d − x) + μ + p ]
class OperationalImpedanceController
    : public controller_interface::ControllerInterface {
public:
  static constexpr size_t NUM_JOINTS = 6;
  using Vec6 = Eigen::Vector<double, NUM_JOINTS>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  OperationalImpedanceController() = default;
  ~OperationalImpedanceController() override = default;

  // -- ros2_control lifecycle --
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // -- SDK model --
  std::unique_ptr<UNITREE_ARM::Z1Model> model_;

  // -- Joint state --
  Vec6 q_ = Vec6::Zero();
  Vec6 dq_ = Vec6::Zero();

  // -- Kinematics --
  Mat6 jacobian_ = Mat6::Zero();
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();

  // -- Dynamics (recomputed each cycle) --
  Mat6 mass_matrix_ = Mat6::Identity();
  Mat6 mass_matrix_inv_ = Mat6::Identity();
  Vec6 coriolis_ = Vec6::Zero();
  Vec6 gravity_vec_ = Vec6::Zero();
  Mat6 lambda_ = Mat6::Identity();
  Vec6 mu_ = Vec6::Zero();
  Vec6 p_ = Vec6::Zero();

  // -- Reference pose (filtered and target) --
  Eigen::Vector3d position_d_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_d_target_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_d_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond orientation_d_target_ = Eigen::Quaterniond::Identity();

  // -- Reference velocity (optional, from subscription) --
  Eigen::Matrix<double, 6, 1> x_dot_d_ =
      Eigen::Matrix<double, 6, 1>::Zero();

  // -- Impedance gains (filtered and target) --
  Mat6 stiffness_ = Mat6::Identity();
  Mat6 stiffness_target_ = Mat6::Identity();
  Mat6 damping_ = Mat6::Identity();
  Mat6 damping_target_ = Mat6::Identity();

  // alpha_i in D_i = alpha_i * 2*sqrt(K_i)
  Eigen::Matrix<double, 6, 1> damping_factors_ =
      Eigen::Matrix<double, 6, 1>::Ones();

  // -- Inertia shaping --
  bool use_inertia_shaping_ = false;
  Mat6 desired_inertia_ = Mat6::Identity();
  Mat6 desired_inertia_inv_ = Mat6::Identity();

  // -- External/reaction force F_r (filtered and target) --
  Eigen::Matrix<double, 6, 1> F_ext_ =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> F_ext_target_ =
      Eigen::Matrix<double, 6, 1>::Zero();

  // -- Commanded torques (kept for rate limiting) --
  Vec6 tau_c_ = Vec6::Zero();

  // -- Parameters --
  std::vector<std::string> joint_names_;
  double delta_tau_max_ = 1.0;
  double update_frequency_ = 1000.0;
  double filter_stiffness_ = 0.1;
  double filter_pose_ = 0.1;
  double filter_wrench_ = 0.1;
  double lambda_damping_ = 0.005;

  // -- ROS subscribers --
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;

  // -- Debug publishers (decimated to update_rate / DEBUG_DECIMATION) --
  using DebugPub =
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;
  DebugPub debug_reference_gap_pub_;
  DebugPub debug_tracking_error_pub_;
  DebugPub debug_torque_desired_pub_;
  DebugPub debug_torque_commanded_pub_;
  DebugPub debug_ee_velocity_pub_;
  DebugPub debug_lambda_diag_pub_;
  size_t debug_counter_ = 0;
  static constexpr size_t DEBUG_DECIMATION = 10;

  // -- Internal helpers --
  void read_joint_states();
  void update_kinematics();
  void update_dynamics();
  void apply_filtering();
  Vec6 compute_torques();

  /// Extract 6x6 joint-space mass matrix from RNEA (column by column).
  void compute_mass_matrix();

  /// Compute Λ, μ, p from the joint-space dynamics and Jacobian.
  void compute_operational_space_dynamics();

  /// Numerical J̇ q̇ via finite difference of the Jacobian.
  Eigen::Matrix<double, 6, 1> compute_J_dot_dq();

  static Eigen::Vector3d orientation_error(const Eigen::Quaterniond &desired,
                                           Eigen::Quaterniond current);

  static void saturate_torque_rate(const Vec6 &desired, Vec6 &previous,
                                   double delta_max);

  /// d = 2 * sqrt(k)
  static double damping_rule(double stiffness);
  void apply_damping();

  /// First-order filter step from percentage and update rate.
  static double filter_step(double update_frequency, double filter_percentage);

  void publish_debug(const Vec6 &tau_d);

  /// Exponential-smoothing update.
  template <typename T>
  static T filtered_update(const T &target, const T &current, double alpha) {
    return (1.0 - alpha) * current + alpha * target;
  }
};

} // namespace z1

#endif // Z1_OPERATIONAL_IMPEDANCE_CONTROLLER_HPP__
