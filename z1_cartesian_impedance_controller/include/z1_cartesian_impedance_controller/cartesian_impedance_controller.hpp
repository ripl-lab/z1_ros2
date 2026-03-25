#ifndef Z1_CARTESIAN_IMPEDANCE_CONTROLLER_HPP__
#define Z1_CARTESIAN_IMPEDANCE_CONTROLLER_HPP__

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace UNITREE_ARM {
class Z1Model;
}

namespace z1 {

/// Cartesian impedance controller for the Unitree Z1 (6-DOF).
///
/// Implements a Jacobian-transpose Cartesian spring-damper with gravity
/// compensation using the Unitree SDK dynamics model.  Ported from the
/// Mayr et al. Cartesian Impedance Controller (ROS 1) and adapted for
/// the Z1's 6-DOF kinematics (no nullspace).
///
/// Control law:
///   tau = J^T (-K e - D J dq) + J^T F_cmd + g(q)
///
/// where K/D are 6x6 diagonal Cartesian stiffness/damping, e is the 6D
/// pose error, F_cmd is an optional commanded wrench, and g(q) is the
/// joint-space gravity vector from the SDK's Newton-Euler inverse dynamics.
class CartesianImpedanceController
    : public controller_interface::ControllerInterface {
public:
  static constexpr size_t NUM_JOINTS = 6;
  using Vec6 = Eigen::Vector<double, NUM_JOINTS>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  CartesianImpedanceController() = default;
  ~CartesianImpedanceController() override = default;

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

  // -- Reference pose (filtered and target) --
  Eigen::Vector3d position_d_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_d_target_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_d_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond orientation_d_target_ = Eigen::Quaterniond::Identity();

  // -- Impedance gains (filtered and target) --
  Mat6 cartesian_stiffness_ = Mat6::Identity();
  Mat6 cartesian_stiffness_target_ = Mat6::Identity();
  Mat6 cartesian_damping_ = Mat6::Identity();
  Mat6 cartesian_damping_target_ = Mat6::Identity();

  // alpha_i in d_i = alpha_i * 2*sqrt(k_i)
  Eigen::Matrix<double, 6, 1> damping_factors_ =
      Eigen::Matrix<double, 6, 1>::Ones();

  // -- Wrench feedforward (filtered and target) --
  Eigen::Matrix<double, 6, 1> cartesian_wrench_ =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> cartesian_wrench_target_ =
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

  // -- ROS subscribers --
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;

  // -- Debug publishers (decimated to update_rate / DEBUG_DECIMATION) --
  using DebugPub = rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr;
  DebugPub debug_reference_gap_pub_;
  DebugPub debug_tracking_error_pub_;
  DebugPub debug_torque_desired_pub_;
  DebugPub debug_torque_commanded_pub_;
  DebugPub debug_ee_velocity_pub_;
  size_t debug_counter_ = 0;
  static constexpr size_t DEBUG_DECIMATION = 10;

  // -- Internal helpers --
  void read_joint_states();
  void update_kinematics();
  void apply_filtering();
  Vec6 compute_torques();

  static Eigen::Vector3d orientation_error(const Eigen::Quaterniond &desired,
                                           Eigen::Quaterniond current);

  static void saturate_torque_rate(const Vec6 &desired, Vec6 &previous,
                                   double delta_max);

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

#endif // Z1_CARTESIAN_IMPEDANCE_CONTROLLER_HPP__
