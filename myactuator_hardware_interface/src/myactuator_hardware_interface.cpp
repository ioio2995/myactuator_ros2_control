#include "myactuator_hardware_interface/myactuator_hardware_interface.hpp"

#define APP_MODULE "MyActuator_HardwareInterface"

#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace myactuator_hardware_interface
{
  CallbackReturn MyActuatorHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);

    hw_motor_temperature_.resize(info_.joints.size(), std::numeric_limits<std::uint16_t>::quiet_NaN());
    hw_voltage_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_current_phase_a_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_current_phase_b_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_current_phase_c_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_brake_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    ifname_ = info_.hardware_parameters["ifname"];
    motors_timeout_ = std::chrono::milliseconds(int(std::stoi(info_.hardware_parameters["timeout"])));
    motors_max_speed_ = std::stoi(info_.hardware_parameters["max_speed"]);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      can_id_.emplace_back(std::stoi(joint.parameters.at("can_id")));
      motor_torque_constant_.emplace_back(std::stof(joint.parameters.at("torque_constant")));
      motor_reducer_ratio_.emplace_back(std::stof(joint.parameters.at("reducer_ratio")));
      motor_rotor_inertia_.emplace_back(std::stoi(joint.parameters.at("rotor_inertia")));
      motor_speed_constant_.emplace_back(std::stoi(joint.parameters.at("speed_constant")));
      motor_position_acceleration_.emplace_back(std::stoi(joint.parameters.at("position_acceleration")));
      motor_position_deceleration_.emplace_back(std::stoi(joint.parameters.at("position_deceleration")));
      motor_velocity_acceleration_.emplace_back(std::stoi(joint.parameters.at("velocity_acceleration")));
      motor_velocity_deceleration_.emplace_back(std::stoi(joint.parameters.at("velocity_deceleration")));
      motor_gains_.emplace_back(
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_current")),
              std::stoi(joint.parameters.at("ki_current"))),
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_speed")),
              std::stoi(joint.parameters.at("ki_speed"))),
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_position")),
              std::stoi(joint.parameters.at("ki_position"))));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger(APP_MODULE), "Configuring... please wait a moment...");
    HANDLE_TS_EXCEPTIONS(driver_.emplace_back(ifname_));
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      HANDLE_TS_EXCEPTIONS(rdm_.emplace_back(driver_.back(), can_id_[i]));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setTimeout(std::chrono::milliseconds(0)));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setCurrentPositionAsEncoderZero());
      HANDLE_TS_EXCEPTIONS(rdm_[i].reset());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      HANDLE_TS_EXCEPTIONS(rdm_[i].setControllerGains(motor_gains_[i], true));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
          motor_position_acceleration_[i],
          myactuator_rmd::AccelerationType::POSITION_PLANNING_ACCELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
          motor_position_deceleration_[i],
          myactuator_rmd::AccelerationType::POSITION_PLANNING_DECELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
          motor_velocity_acceleration_[i],
          myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
          motor_velocity_deceleration_[i],
          myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setTimeout(motors_timeout_));
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger(APP_MODULE), "Activating... please wait a moment...");

    // Set some default values
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (std::isnan(hw_states_positions_[i])) hw_states_positions_[i] = 0;
      if (std::isnan(hw_states_velocities_[i])) hw_states_velocities_[i] = 0;
      if (std::isnan(hw_states_efforts_[i])) hw_states_efforts_[i] = 0;
      if (std::isnan(hw_commands_positions_[i])) hw_commands_positions_[i] = 0;
      if (std::isnan(hw_commands_velocities_[i])) hw_commands_velocities_[i] = 0;
      if (std::isnan(hw_commands_efforts_[i])) hw_commands_efforts_[i] = 0;
      control_level_[i] = integration_level_t::UNDEFINED;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger(APP_MODULE), "Deactivating... please wait a moment...");
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      hw_commands_efforts_[i] = 0;
      hw_commands_velocities_[i] = 0;
      control_level_[i] = integration_level_t::UNDEFINED;
      HANDLE_TS_EXCEPTIONS(rdm_[i].shutdownMotor());
      HANDLE_TS_EXCEPTIONS(rdm_[i].setTimeout(std::chrono::milliseconds(0)));
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_temperature", &hw_motor_temperature_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_voltage", &hw_voltage_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_phase_a_current", &hw_current_phase_a_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_phase_b_current", &hw_current_phase_b_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_phase_c_current", &hw_current_phase_c_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "motor_brake", &hw_brake_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MyActuatorHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger(APP_MODULE), "Mode Switch.. please wait a moment...");
    std::vector<integration_level_t> new_modes;
    std::vector<bool> new_switch;
    new_modes.resize(info_.joints.size(), integration_level_t::UNDEFINED);
    new_switch.resize(info_.joints.size(), false);


    // Set command modes for starting interfaces
    for (std::string key : start_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          new_modes[i] = integration_level_t::POSITION ;
          new_switch[i] = true ;
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          new_modes[i] = integration_level_t::VELOCITY;
          new_switch[i] = true ;
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
        {
          new_modes[i] = integration_level_t::EFFORT;
          new_switch[i] = true ;
        }
      }
    }

    // Stop motion on all relevant joints that are stopping
    for (std::string key : stop_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        new_modes[i] = integration_level_t::UNDEFINED;
        new_switch[i] = true ;
      }
    }

    // Set the new command modes
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (new_switch[i] != false)
      {
        if (new_modes[i] != integration_level_t::UNDEFINED)
        {
          if (control_level_[i] != integration_level_t::UNDEFINED)
          {
            // Something else is using the joint! Abort!
            std::cerr << "Error: Joint " << i << ": Control level already defined. Aborting..." << std::endl;
            return hardware_interface::return_type::ERROR;
          }
        }
        control_level_[i] = new_modes[i];
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      myactuator_rmd::MotorStatus1 MotorStatus_1;
      myactuator_rmd::MotorStatus2 MotorStatus_2;
      myactuator_rmd::MotorStatus3 MotorStatus_3;
      int32_t MultiTurnAngle;

      HANDLE_RW_EXCEPTIONS(MultiTurnAngle = rdm_[i].getMultiTurnAngle());
      HANDLE_RW_EXCEPTIONS(MotorStatus_1 = rdm_[i].getMotorStatus1());
      HANDLE_RW_EXCEPTIONS(MotorStatus_2 = rdm_[i].getMotorStatus2());
      HANDLE_RW_EXCEPTIONS(MotorStatus_3 = rdm_[i].getMotorStatus3());

      hw_states_positions_[i] = static_cast<double>((MultiTurnAngle * M_PI) / 180);
      hw_states_efforts_[i] = static_cast<double>(MotorStatus_2.current * motor_torque_constant_[i]);
      hw_states_velocities_[i] = static_cast<double>((MotorStatus_2.shaft_speed * M_PI) / 180);
      hw_motor_errors_[i] = static_cast<int>(MotorStatus_1.error_code);
      hw_motor_temperature_[i] = MotorStatus_1.temperature;
      hw_voltage_[i] = static_cast<double>(MotorStatus_1.voltage);
      hw_current_phase_a_[i] = static_cast<double>(MotorStatus_3.current_phase_a);
      hw_current_phase_b_[i] = static_cast<double>(MotorStatus_3.current_phase_b);
      hw_current_phase_c_[i] = static_cast<double>(MotorStatus_3.current_phase_c);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      float input_torque, input_vel, input_pos;

      switch (control_level_[i])
      {
      case integration_level_t::POSITION:
        input_pos = hw_commands_positions_[i] * 180 / M_PI;
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendPositionAbsoluteSetpoint(input_pos, motors_max_speed_));
        break;
      case integration_level_t::VELOCITY:
        input_vel = hw_commands_velocities_[i] * 180 / M_PI;
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendVelocitySetpoint(input_vel));
        break;
      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendTorqueSetpoint(input_torque, motor_torque_constant_[i]));
        break;
      case integration_level_t::UNDEFINED:
        HANDLE_RW_EXCEPTIONS(rdm_[i].shutdownMotor());
        break;
      }
    }
    return hardware_interface::return_type::OK;
  }
} // namespace myactuator_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    myactuator_hardware_interface::MyActuatorHardwareInterface, hardware_interface::SystemInterface)