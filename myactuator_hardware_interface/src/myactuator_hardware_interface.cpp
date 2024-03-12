
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
    torque_constant_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());
    reducer_ratio_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());
    speed_constant_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());
    rotor_inertia_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());

    hw_motor_temperature_.resize(info_.joints.size(), std::numeric_limits<std::uint16_t>::quiet_NaN());
    hw_uptime_.resize(info_.joints.size(), std::chrono::milliseconds(0));

    hw_voltage_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_current_phase_a_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_current_phase_b_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_current_phase_c_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_velocitie_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocitie_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_brake_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      can_id_.emplace_back(std::stoi(joint.parameters.at("can_id")));
      timeout_.emplace_back(std::stoi(joint.parameters.at("timeout")));
      torque_constant_.emplace_back(std::stoi(joint.parameters.at("torque_constant")));
      reducer_ratio_.emplace_back(std::stoi(joint.parameters.at("reducer_ratio")));
      speed_constant_.emplace_back(std::stoi(joint.parameters.at("speed_constant")));
      rotor_inertia_.emplace_back(std::stoi(joint.parameters.at("rotor_inertia")));
      position_acceleration_.emplace_back(std::stoi(joint.parameters.at("position_acceleration")));
      position_deceleration_.emplace_back(std::stoi(joint.parameters.at("position_deceleration")));
      velocity_acceleration_.emplace_back(std::stoi(joint.parameters.at("velocity_acceleration")));
      velocity_deceleration_.emplace_back(std::stoi(joint.parameters.at("velocity_deceleration")));
      gains_.emplace_back(
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_current")),
              std::stoi(joint.parameters.at("ki_current"))
          ),
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_speed")),
              std::stoi(joint.parameters.at("ki_speed"))
          ),
          myactuator_rmd::PiGains(
              std::stoi(joint.parameters.at("kp_position")),
              std::stoi(joint.parameters.at("ki_position"))
          )
      );
    }

    ifname_ = info_.hardware_parameters["ifname"];
    HANDLE_TS_EXCEPTIONS(driver_.emplace_back(ifname_));

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      HANDLE_TS_EXCEPTIONS(rdm_.emplace_back(driver_.back(), can_id_[i]));
      
      HANDLE_TS_EXCEPTIONS(rdm_[i].setTimeout(timeout_[i]));

      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
        position_acceleration_[i], 
        myactuator_rmd::AccelerationType::POSITION_PLANNING_ACCELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
        position_deceleration_[i], 
        myactuator_rmd::AccelerationType::POSITION_PLANNING_DECELERATION));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
        velocity_acceleration_[i], 
        myactuator_rmd::AccelerationType::VELOCITY_PLANNING_ACCELERATION ));
      HANDLE_TS_EXCEPTIONS(rdm_[i].setAcceleration(
        velocity_deceleration_[i], 
        myactuator_rmd::AccelerationType::VELOCITY_PLANNING_DECELERATION ));

      HANDLE_TS_EXCEPTIONS(rdm_[i].setControllerGains(
        gains_[i]));
    }

    control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      HANDLE_TS_EXCEPTIONS(rdm_[i].stopMotor());
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyActuatorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      HANDLE_TS_EXCEPTIONS(rdm_[i].shutdownMotor());
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MyActuatorHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocitie_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_[i]));
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

  std::vector<hardware_interface::CommandInterface>
  MyActuatorHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocitie_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces)
  {
    for (std::string key : stop_interfaces)
    {
      for (size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          control_level_[i] = integration_level_t::UNDEFINED;
        }
      }
    }

    for (std::string key : start_interfaces)
    {
      for (size_t i = 0; i < info_.joints.size(); i++)
      {
        switch (control_level_[i])
        {
        case integration_level_t::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
          {
            control_level_[i] = integration_level_t::EFFORT;
          }

        case integration_level_t::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
          {
            control_level_[i] = integration_level_t::VELOCITY;
          }

        case integration_level_t::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
          {
            control_level_[i] = integration_level_t::POSITION;
          }

        case integration_level_t::POSITION:
          break;
        }
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::perform_command_mode_switch(
      const std::vector<std::string> &, const std::vector<std::string> &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      float input_torque, input_vel, input_pos;
      float constant_torque;

      switch (control_level_[i])
      {
      case integration_level_t::UNDEFINED:
        HANDLE_RW_EXCEPTIONS(rdm_[i].stopMotor());
        break;

      case integration_level_t::EFFORT:
        hw_commands_effort_[i] = hw_effort_[i];

        input_torque = hw_commands_effort_[i];
        constant_torque = torque_constant_[i];

        HANDLE_RW_EXCEPTIONS(rdm_[i].sendTorqueSetpoint(input_torque,constant_torque));
        break;

      case integration_level_t::VELOCITY:
        hw_commands_velocitie_[i] = hw_velocitie_[i];
        hw_commands_effort_[i] = 0;

        input_vel = hw_commands_velocitie_[i] / 2 / M_PI;
        input_torque = hw_commands_effort_[i];

        HANDLE_RW_EXCEPTIONS(rdm_[i].sendVelocitySetpoint(input_vel));
        break;

      case integration_level_t::POSITION:
        hw_commands_position_[i] = hw_position_[i];
        hw_commands_velocitie_[i] = hw_velocitie_[i];
        hw_commands_effort_[i] = 0;

        input_pos = hw_commands_position_[i] * 180 /M_PI;
        input_vel = hw_commands_velocitie_[i] / 2 / M_PI;

        HANDLE_RW_EXCEPTIONS(rdm_[i].sendPositionAbsoluteSetpoint(input_pos,input_vel));
        break;
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

      HANDLE_RW_EXCEPTIONS(MotorStatus_1=rdm_[i].getMotorStatus1());
      hw_motor_temperature_[i] = MotorStatus_1.temperature;
      hw_voltage_[i] = static_cast<double>(MotorStatus_1.voltage * 1000);
      hw_motor_errors_[i] = static_cast<int>(MotorStatus_1.error_code);

      HANDLE_RW_EXCEPTIONS(MotorStatus_2=rdm_[i].getMotorStatus2());
      hw_effort_[i] = static_cast<double>(MotorStatus_2.current)*torque_constant_[i];
      hw_position_[i] = static_cast<double>(MotorStatus_2.shaft_angle * M_PI / 180);
     hw_velocitie_[i] = static_cast<double>(MotorStatus_2.shaft_speed * 2 * M_PI);

      
      HANDLE_RW_EXCEPTIONS(MotorStatus_3=rdm_[i].getMotorStatus3());
      hw_current_phase_a_[i] = static_cast<double>(MotorStatus_3.current_phase_a * 1000);
      hw_current_phase_b_[i] = static_cast<double>(MotorStatus_3.current_phase_b * 1000);
      hw_current_phase_c_[i] = static_cast<double>(MotorStatus_3.current_phase_c * 1000);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MyActuatorHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      float input_torque, input_vel, input_pos, constant_torque;

      switch (control_level_[i])
      {
      case integration_level_t::POSITION:
        input_pos = hw_commands_position_[i] * 180 /M_PI;
        input_vel = hw_commands_velocitie_[i] / 2 / M_PI;
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendPositionAbsoluteSetpoint(input_pos,input_vel));

      case integration_level_t::VELOCITY:
        input_vel = hw_commands_velocitie_[i] / 2 / M_PI;
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendVelocitySetpoint(input_vel));

      case integration_level_t::EFFORT:
        input_torque = hw_commands_effort_[i];
        constant_torque = torque_constant_[i];
        HANDLE_RW_EXCEPTIONS(rdm_[i].sendTorqueSetpoint(input_torque,constant_torque));

      case integration_level_t::UNDEFINED:
        HANDLE_RW_EXCEPTIONS(rdm_[i].stopMotor());
      }
    }

    return hardware_interface::return_type::OK;
  }
} // namespace myactuator_hardware_interface


PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware_interface::MyActuatorHardwareInterface, hardware_interface::SystemInterface)