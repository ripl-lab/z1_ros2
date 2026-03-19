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
#include "z1_hardware_interface/z1_isaac_hardware_interface.hpp"

#include <algorithm>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include <pluginlib/class_list_macros.hpp>

using unitree::z1::IsaacHardwareInterface;

//  _     _  __                      _
// | |   (_)/ _| ___  ___ _   _  ___| | ___
// | |   | | |_ / _ \/ __| | | |/ __| |/ _ \
// | |___| |  _|  __/ (__| |_| | (__| |  __/
// |_____|_|_|  \___|\___|\__, |\___|_|\___|
//                        |___/

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_configure(const rclcpp_lifecycle::State& prev_state) {
    if (hardware_interface::SystemInterface::on_configure(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(_logger, "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(_logger, "Configuring Isaac Sim hardware interface");
    if (with_gripper()) RCLCPP_INFO(_logger, "Gripper enabled");
    else                RCLCPP_INFO(_logger, "Gripper disabled");

    _node = std::make_shared<rclcpp::Node>("z1_isaac_hw_node");

    _joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            joint_state_callback(std::move(msg));
        }
    );

    _joint_cmd_pub = _node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_commands",
        rclcpp::SystemDefaultsQoS()
    );

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(_node);
    _spin_thread = std::thread([this]() { _executor->spin(); });

    RCLCPP_INFO(
        _logger,
        "Subscribed to /joint_states — publishing commands to /joint_commands"
    );
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& prev_state) {
    if (hardware_interface::SystemInterface::on_cleanup(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(_logger, "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    stop_spin_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& prev_state) {
    hardware_interface::SystemInterface::on_shutdown(prev_state);
    stop_spin_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_activate(const rclcpp_lifecycle::State& prev_state) {
    if (hardware_interface::SystemInterface::on_activate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(_logger, "parent on_activate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(_logger, "Isaac Sim hardware interface activated — waiting for /joint_states");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state) {
    if (hardware_interface::SystemInterface::on_deactivate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(_logger, "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
IsaacHardwareInterface::on_error(const rclcpp_lifecycle::State& prev_state) {
    hardware_interface::SystemInterface::on_error(prev_state);
    stop_spin_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

std::vector<hardware_interface::StateInterface>
IsaacHardwareInterface::export_state_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(with_gripper() ? 21 : 18);  // 7(or 6) joints * 3 states

    for (long i = 0; i < 6; ++i) {
        const std::string& jnt = info_.joints[i].name;
        state_interfaces.emplace_back(jnt, HW_IF_POSITION, &_arm_state.q(i));
        state_interfaces.emplace_back(jnt, HW_IF_VELOCITY, &_arm_state.qd(i));
        state_interfaces.emplace_back(jnt, HW_IF_EFFORT,   &_arm_state.tau(i));
    }
    if (with_gripper()) {
        const std::string& jnt = info_.joints[6].name;
        state_interfaces.emplace_back(jnt, HW_IF_POSITION, &_gripper_state.q);
        state_interfaces.emplace_back(jnt, HW_IF_VELOCITY, &_gripper_state.qd);
        state_interfaces.emplace_back(jnt, HW_IF_EFFORT,   &_gripper_state.tau);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IsaacHardwareInterface::export_command_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::CommandInterface> cmd_interfaces;
    cmd_interfaces.reserve(with_gripper() ? 21 : 18);

    for (long i = 0; i < 6; ++i) {
        const std::string& jnt = info_.joints[i].name;
        cmd_interfaces.emplace_back(jnt, HW_IF_POSITION, &_arm_cmd.q(i));
        cmd_interfaces.emplace_back(jnt, HW_IF_VELOCITY, &_arm_cmd.qd(i));
        cmd_interfaces.emplace_back(jnt, HW_IF_EFFORT,   &_arm_cmd.tau(i));
    }
    if (with_gripper()) {
        const std::string& jnt = info_.joints[6].name;
        cmd_interfaces.emplace_back(jnt, HW_IF_POSITION, &_gripper_cmd.q);
        cmd_interfaces.emplace_back(jnt, HW_IF_VELOCITY, &_gripper_cmd.qd);
        cmd_interfaces.emplace_back(jnt, HW_IF_EFFORT,   &_gripper_cmd.tau);
    }
    return cmd_interfaces;
}

//   ____            _             _   _
//  / ___|___  _ __ | |_ _ __ ___ | | | |    ___   ___  _ __
// | |   / _ \| '_ \| __| '__/ _ \| | | |   / _ \ / _ \| '_ \
// | |__| (_) | | | | |_| | | (_) | | | |__| (_) | (_) | |_) |
//  \____\___/|_| |_|\__|_|  \___/|_| |_____\___/ \___/| .__/
//                                                      |_|

hardware_interface::return_type
IsaacHardwareInterface::read(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */
) {
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (!_state_received) return hardware_interface::return_type::OK;

    const auto& msg = _latest_joint_state;
    const std::size_t n = std::min(msg.name.size(), msg.position.size());

    for (std::size_t k = 0; k < n; ++k) {
        const long idx = get_joint_idx(msg.name[k]);
        if (idx < 0) continue;

        const bool has_vel    = k < msg.velocity.size();
        const bool has_effort = k < msg.effort.size();

        if (idx < 6) {
            _arm_state.q(idx)   = msg.position[k];
            if (has_vel)    _arm_state.qd(idx)  = msg.velocity[k];
            if (has_effort) _arm_state.tau(idx) = msg.effort[k];
        } else if (with_gripper()) {
            _gripper_state.q   = msg.position[k];
            if (has_vel)    _gripper_state.qd  = msg.velocity[k];
            if (has_effort) _gripper_state.tau = msg.effort[k];
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
IsaacHardwareInterface::write(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */
) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = _node->get_clock()->now();

    const int n_joints = with_gripper() ? 7 : 6;
    msg.name.reserve(n_joints);
    msg.position.reserve(n_joints);
    msg.velocity.reserve(n_joints);
    msg.effort.reserve(n_joints);

    for (int i = 0; i < 6; ++i) {
        msg.name.push_back(info_.joints[i].name);
        msg.position.push_back(_arm_cmd.q(i));
        msg.velocity.push_back(_arm_cmd.qd(i));
        msg.effort.push_back(_arm_cmd.tau(i));
    }
    if (with_gripper()) {
        msg.name.push_back(info_.joints[6].name);
        msg.position.push_back(_gripper_cmd.q);
        msg.velocity.push_back(_gripper_cmd.qd);
        msg.effort.push_back(_gripper_cmd.tau);
    }

    _joint_cmd_pub->publish(msg);
    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

void
IsaacHardwareInterface::stop_spin_thread() {
    if (_executor) _executor->cancel();
    if (_spin_thread.joinable()) _spin_thread.join();
}

void
IsaacHardwareInterface::joint_state_callback(
        sensor_msgs::msg::JointState::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(_state_mutex);
    _latest_joint_state = std::move(*msg);
    _state_received     = true;
}

long
IsaacHardwareInterface::get_joint_idx(const std::string& joint_name) const {
    for (long i = 0; i < static_cast<long>(info_.joints.size()); ++i) {
        if (info_.joints[i].name == joint_name) return i;
    }
    return -1;
}

bool
IsaacHardwareInterface::with_gripper() const {
    auto it = info_.hardware_parameters.find("gripper");
    if (it == info_.hardware_parameters.end()) return true;
    std::string val = it->second;
    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    return val == "true";
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
PLUGINLIB_EXPORT_CLASS(
        unitree::z1::IsaacHardwareInterface,
        hardware_interface::SystemInterface
);
