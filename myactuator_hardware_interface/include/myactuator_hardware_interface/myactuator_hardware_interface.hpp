// Copyright 2021 Factor Robotics
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

#pragma once

#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "myactuator_hardware_interface/myactuator_control.hpp"
#include "myactuator_hardware_interface/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"

#include <myactuator_rmd/driver.hpp>

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8


using namespace myactuator;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace myactuator_hardware_interface
{
class MyActuatorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyActuatorHardwareInterface)

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  MyActuatorCAN * myactuator;

  std::vector<std::vector<int64_t>> can_numbers_;
  std::vector<int> axes_;
  std::vector<float> torque_constants_;
  std::vector<bool> enable_watchdogs_;

  std::vector<double> hw_vbus_voltages_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  std::vector<double> hw_axis_errors_;
  std::vector<double> hw_motor_errors_;
  std::vector<double> hw_encoder_errors_;
  std::vector<double> hw_controller_errors_;
  std::vector<double> hw_fet_temperatures_;
  std::vector<double> hw_motor_temperatures_;

  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };

  std::vector<integration_level_t> control_level_;
};
}  // namespace myactuator_hardware_interface
