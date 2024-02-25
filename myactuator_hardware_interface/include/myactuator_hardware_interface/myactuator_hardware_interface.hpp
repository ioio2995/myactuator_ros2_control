
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

// myactuator_rmd
#include <myactuator_rmd/myactuator_rmd.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"

//ros2_control hardware_interface
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "myactuator_hardware_interface/visibility_control.hpp"

#define HANDLE_TS_EXCEPTIONS(code_block)                                        \
  try                                                                           \
  {                                                                             \
    code_block;                                                                 \
  }                                                                             \
  catch (const myactuator_rmd::ProtocolException &ProtocolException)            \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver ProtocolException");   \
    return CallbackReturn::ERROR;                                               \
  }                                                                             \
  catch (const myactuator_rmd::ValueRangeException &ValueRangeException)        \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver ValueRangeException"); \
    return CallbackReturn::ERROR;                                               \
  }                                                                             \
  catch (const myactuator_rmd::Exception &DriverException)                      \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver Exception");           \
    return CallbackReturn::ERROR;                                               \
  } 

#define HANDLE_RW_EXCEPTIONS(code_block)                                        \
  try                                                                           \
  {                                                                             \
    code_block;                                                                 \
  }                                                                             \
  catch (const myactuator_rmd::ProtocolException &ProtocolException)            \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver ProtocolException");   \
    return hardware_interface::return_type::ERROR;                              \
  }                                                                             \
  catch (const myactuator_rmd::ValueRangeException &ValueRangeException)        \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver ValueRangeException"); \
    return hardware_interface::return_type::ERROR;                              \
  }                                                                             \
  catch (const myactuator_rmd::Exception &DriverException)                      \
  {                                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(APP_MODULE), "Driver Exception");           \
    return hardware_interface::return_type::ERROR;                              \
  }

namespace myactuator_hardware_interface
{
  class MyActuatorHardwareInterface : public hardware_interface::SystemInterface 
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyActuatorHardwareInterface)

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> &, const std::vector<std::string> &) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  private:
    std::vector<myactuator_rmd::Actuator> rdm_;
    std::vector<std::string> ifname_;
    std::vector<std::uint32_t> can_id_;

    std::vector<double> hw_commands_position_;
    std::vector<double> hw_commands_velocitie_;
    std::vector<double> hw_commands_effort_;
    std::vector<double> hw_position_;
    std::vector<double> hw_velocitie_;
    std::vector<double> hw_effort_;

    std::vector<std::uint32_t> position_acceleration_;
    std::vector<std::uint32_t> position_deceleration_;
    std::vector<std::uint32_t> velocity_acceleration_;
    std::vector<std::uint32_t> velocity_deceleration_;
    std::vector<myactuator_rmd::Gains> gains_;

    std::vector<float> torque_constant_;

    std::vector<std::chrono::milliseconds> timeout_;

    std::vector<double> hw_motor_temperature_;
    std::vector<std::chrono::milliseconds> hw_uptime_;

    std::vector<double> hw_voltage_;
    std::vector<double> hw_current_;

    std::vector<double> hw_current_phase_a_;
    std::vector<double> hw_current_phase_b_;
    std::vector<double> hw_current_phase_c_;

    std::vector<double> hw_speed_;
    std::vector<double> hw_angle_;

    std::vector<double> hw_brake_;

    std::vector<double> hw_motor_errors_;

    enum class integration_level_t : int32_t
    {
      UNDEFINED = 0,
      EFFORT = 1,
      VELOCITY = 2,
      POSITION = 3
    };

    std::vector<integration_level_t> control_level_;
  };
} // namespace myactuator_hardware_interface
