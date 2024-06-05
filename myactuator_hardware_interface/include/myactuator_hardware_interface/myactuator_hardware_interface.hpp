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
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

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
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

    MYACTUATOR_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  private:
    std::string ifname_;
    std::vector<myactuator_rmd::CanDriver> driver_;
    
    std::vector<std::uint32_t> can_id_;
    std::vector<myactuator_rmd::ActuatorInterface> rdm_;

    std::uint32_t motors_max_speed_ ;
    std::chrono::milliseconds motors_timeout_;

    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_commands_efforts_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_efforts_;

    std::vector<double> hw_motor_temperature_;
    std::vector<double> hw_voltage_;
    std::vector<double> hw_current_phase_a_;
    std::vector<double> hw_current_phase_b_;
    std::vector<double> hw_current_phase_c_;
    std::vector<double> hw_motor_errors_;
    
    std::vector<std::uint32_t> motor_position_acceleration_;
    std::vector<std::uint32_t> motor_position_deceleration_;
    std::vector<std::uint32_t> motor_velocity_acceleration_;
    std::vector<std::uint32_t> motor_velocity_deceleration_;
    std::vector<myactuator_rmd::Gains> motor_gains_;

    std::vector<double> motor_torque_constant_;
    std::vector<double> motor_reducer_ratio_;
    std::vector<double> motor_speed_constant_;
    std::vector<double> motor_rotor_inertia_ ;

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
