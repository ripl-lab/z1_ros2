#ifndef Z1_ISAAC_HW_INTERFACE_HPP
#define Z1_ISAAC_HW_INTERFACE_HPP

#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace unitree::z1 {

/**
 * ros2_control hardware interface that bridges the Z1 arm to Isaac Sim.
 *
 * read()  ← subscribes to /joint_states   (Isaac Sim publishes)
 * write() → publishes to  /joint_commands  (Isaac Sim subscribes via
 *                                           Articulation Controller)
 *
 * The subscriber runs on a dedicated spin thread so it never blocks
 * the ros2_control real-time loop.
 */
class IsaacHardwareInterface : public hardware_interface::SystemInterface {
public:
    using Vec6 = Eigen::Vector<double, 6>;

    RCLCPP_SHARED_PTR_DEFINITIONS(IsaacHardwareInterface)

    IsaacHardwareInterface()           = default;
    ~IsaacHardwareInterface() override = default;

    IsaacHardwareInterface(const IsaacHardwareInterface&)             = delete;
    IsaacHardwareInterface(IsaacHardwareInterface&&)                  = delete;
    IsaacHardwareInterface& operator=(const IsaacHardwareInterface&)  = delete;
    IsaacHardwareInterface& operator=(IsaacHardwareInterface&&)       = delete;

    // --- Lifecycle -----------------------------------------------------------

    hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& prev_state
    ) override;

    // --- ros2_control interface ----------------------------------------------

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces
    ) override;

    // -------------------------------------------------------------------------

    [[nodiscard]] bool with_gripper() const;

private:
    rclcpp::Logger _logger = rclcpp::get_logger("z1_isaac_hardware_interface");

    // ROS2 communication
    rclcpp::Node::SharedPtr                                            _node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     _joint_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        _joint_cmd_pub;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr              _executor;
    std::thread                                                        _spin_thread;

    // Latest joint state received from Isaac Sim (mutex-protected)
    mutable std::mutex            _state_mutex;
    sensor_msgs::msg::JointState  _latest_joint_state;
    bool                          _state_received{false};

    // State buffers (exported as StateInterfaces)
    struct {
        Vec6   q   = Vec6::Zero();
        Vec6   qd  = Vec6::Zero();
        Vec6   tau = Vec6::Zero();
    } _arm_state;

    struct {
        double q   = 0.0;
        double qd  = 0.0;
        double tau = 0.0;
    } _gripper_state;

    // Command buffers (written by active controller via CommandInterfaces).
    // NaN = "not commanded" — write() skips arrays where all values are NaN,
    // so Isaac Sim's ArticulationController only applies the active command type.
    static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

    struct {
        Vec6   q   = Vec6::Constant(NaN);
        Vec6   qd  = Vec6::Constant(NaN);
        Vec6   tau = Vec6::Constant(NaN);
    } _arm_cmd;

    struct {
        double q   = NaN;
        double qd  = NaN;
        double tau = NaN;
    } _gripper_cmd;

    void stop_spin_thread();
    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);

    // Returns the index of joint_name in info_.joints, or -1 if not found
    long get_joint_idx(const std::string& joint_name) const;
};

}  // namespace unitree::z1

#endif  // Z1_ISAAC_HW_INTERFACE_HPP
