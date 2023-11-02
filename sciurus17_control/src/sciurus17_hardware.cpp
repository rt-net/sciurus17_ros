// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "sciurus17_control/sciurus17_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace sciurus17_control
{
static rclcpp::Logger LOGGER = rclcpp::get_logger("Sciurus17Hardware");
std::vector<std::string> GROUP_NAMES = {"right_arm", "left_arm", "torso"};
constexpr auto START_P_GAIN = 800;
constexpr auto START_I_GAIN = 0;
constexpr auto START_D_GAIN = 0;
constexpr auto STOP_P_GAIN = 5;
constexpr auto STOP_I_GAIN = 0;
constexpr auto STOP_D_GAIN = 0;

Sciurus17Hardware::~Sciurus17Hardware()
{
  if (hardware_) {
    // Set low PID gains for safe shutdown.
    for (const auto & group_name : GROUP_NAMES) {
      if (!hardware_->write_position_pid_gain_to_group(
          group_name, STOP_P_GAIN, STOP_I_GAIN, STOP_D_GAIN))
      {
        RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
      }
    }
    hardware_->disconnect();
  }
}

CallbackReturn Sciurus17Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    hw_position_commands_[joint.name] = std::numeric_limits<double>::quiet_NaN();
    hw_position_states_[joint.name] = std::numeric_limits<double>::quiet_NaN();
    hw_velocity_states_[joint.name] = std::numeric_limits<double>::quiet_NaN();
    hw_effort_states_[joint.name] = std::numeric_limits<double>::quiet_NaN();
  }

  // Load joint parameters
  for (auto joint : info_.joints) {
    if (joint.parameters["current_to_effort"] != "") {
      current_to_effort_[joint.name] = std::stod(joint.parameters["current_to_effort"]);
    } else {
      RCLCPP_ERROR(
        LOGGER, "Joint '%s' does not have 'current_to_effort' parameter.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %ld command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s'has %ld state interfaces. At least 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    for (auto state_interface : joint.state_interfaces) {
      if (!(state_interface.name == hardware_interface::HW_IF_POSITION ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY ||
        state_interface.name == hardware_interface::HW_IF_EFFORT))
      {
        RCLCPP_FATAL(
          LOGGER,
          "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return CallbackReturn::ERROR;
      }
    }
  }

  const auto port_name = info_.hardware_parameters["port_name"];
  const auto baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  const auto config_file_path = info_.hardware_parameters["manipulator_config_file_path"];
  hardware_ = std::make_shared<rt_manipulators_cpp::Hardware>(port_name);

  if (!hardware_->connect(baudrate)) {
    RCLCPP_ERROR(LOGGER, "Failed to connect a robot.");
    return CallbackReturn::ERROR;
  }

  if (!hardware_->load_config_file(config_file_path)) {
    RCLCPP_ERROR(LOGGER, "Failed to read a config file.");
    return CallbackReturn::ERROR;
  }

  timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Sciurus17Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & joint : info_.joints) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_position_states_[joint.name]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[joint.name]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[joint.name]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Sciurus17Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto joint : info_.joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[joint.name]));
  }

  return command_interfaces;
}

CallbackReturn Sciurus17Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set current timestamp to disable the communication timeout.
  prev_comm_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;

  // Set present joint positions to hw_position_commands for safe start-up.
  read(prev_comm_timestamp_, rclcpp::Duration::from_seconds(0));
  for (auto joint : info_.joints) {
    const auto present_position = hw_position_states_[joint.name];
    const auto limit_min = present_position;
    const auto limit_max = present_position;
    for (auto interface : joint.command_interfaces) {
      if (interface.name == "position") {
        limit_min = std::stod(interface.min);
        limit_max = std::stod(interface.max);
      }
    }
    hw_position_commands_[joint.name] = std::clamp(present_position, limit_min, limit_max);
  }
  write(prev_comm_timestamp_, rclcpp::Duration::from_seconds(0));

  for (const auto & group_name : GROUP_NAMES) {
    if (!hardware_->write_position_pid_gain_to_group(
        group_name, START_P_GAIN, START_I_GAIN, START_D_GAIN))
    {
      RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
      return CallbackReturn::ERROR;
    }

    if (!hardware_->torque_on(group_name)) {
      RCLCPP_ERROR(LOGGER, "Failed to set torque on.");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Sciurus17Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set low PID gains for safe stopping.
  for (const auto & group_name : GROUP_NAMES) {
    if (!hardware_->write_position_pid_gain_to_group(
        group_name, STOP_P_GAIN, STOP_I_GAIN, STOP_D_GAIN))
    {
      RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

return_type Sciurus17Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(LOGGER, "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  for (const auto & group_name : GROUP_NAMES) {
    if (!hardware_->sync_read(group_name)) {
      RCLCPP_ERROR(LOGGER, "Failed to sync read from servo motors.");
      // sync_readに失敗しても通信は継続させる。
      // 不確かなデータをセットしないようにOKを返す。
      return return_type::OK;
    }
  }

  for (const auto & joint : info_.joints) {
    hardware_->get_position(joint.name, hw_position_states_[joint.name]);
  }

  for (const auto & joint : info_.joints) {
    hardware_->get_velocity(joint.name, hw_velocity_states_[joint.name]);
  }

  double current;
  for (const auto & joint : info_.joints) {
    hardware_->get_current(joint.name, current);
    hw_effort_states_[joint.name] = current * current_to_effort_[joint.name];
  }

  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

return_type Sciurus17Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(LOGGER, "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  for (const auto & joint : info_.joints) {
    hardware_->set_position(joint.name, hw_position_commands_[joint.name]);
  }

  for (const auto & group_name : GROUP_NAMES) {
    if (!hardware_->sync_write(group_name)) {
      RCLCPP_ERROR(LOGGER, "Failed to sync write to servo motors.");
    }
  }
  // Motor電源がOFFでもsync_writeはエラーを返さないので、ここではtimestampを更新しない
  // prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

bool Sciurus17Hardware::communication_timeout()
{
  if (steady_clock_.now().seconds() - prev_comm_timestamp_.seconds() >= timeout_seconds_) {
    return true;
  } else {
    return false;
  }
}

}  // namespace sciurus17_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  sciurus17_control::Sciurus17Hardware,
  hardware_interface::SystemInterface)
