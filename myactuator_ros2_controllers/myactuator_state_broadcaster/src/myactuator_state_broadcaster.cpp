#include "myactuator_state_broadcaster/myactuator_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace myactuator_state_broadcaster
{
MyActuatorStateBroadcaster::MyActuatorStateBroadcaster() {
}

controller_interface::CallbackReturn MyActuatorStateBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyActuatorStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration MyActuatorStateBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  if (use_all_available_interfaces())
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  }
  else
  {
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints)
    {
      for (const auto & interface : params_.interfaces)
      {
        state_interfaces_config.names.push_back(joint + "/" + interface);
      }
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn MyActuatorStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (use_all_available_interfaces())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'joints' or 'interfaces' parameter is empty. "
      "All available state interfaces will be published");
    params_.joints.clear();
    params_.interfaces.clear();
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Publishing state interfaces defined in 'joints' and 'interfaces' parameters.");
  }

  try
  {
    const std::string topic_name_prefix = params_.use_local_topics ? "~/" : "";

    motor_state_publisher_ = get_node()->create_publisher<myactuator_interfaces::msg::MotorStatus>(
      topic_name_prefix + "myactuator_states", rclcpp::SystemDefaultsQoS());

    realtime_motor_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<myactuator_interfaces::msg::MotorStatus>>(
        motor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyActuatorStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  init_joint_state_msg();

  if (
    !use_all_available_interfaces() &&
    state_interfaces_.size() != (params_.joints.size() * params_.interfaces.size()))
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists. "
      "Check ControllerManager output for more detailed information.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyActuatorStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();

  return CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(
  const std::unordered_map<std::string, T> & map, const std::vector<std::string> & keys)
{
  bool found_key = false;
  for (const auto & key_item : map)
  {
    const auto & key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool MyActuatorStateBroadcaster::init_joint_data()
{
  joint_names_.clear();
  if (state_interfaces_.empty())
  {
    return false;
  }

  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_prefix_name()) == 0)
    {
      name_if_value_mapping_[si->get_prefix_name()] = {};
    }
    // add interface name
    std::string interface_name = si->get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[si->get_prefix_name()][interface_name] = std::numeric_limits<double>::quiet_NaN();
  }

  // filter state interfaces that have at least one of the joint_states fields,
  // the rest will be ignored for this message
  for (const auto & name_ifv : name_if_value_mapping_)
  {
    const auto & interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, {"motor_error", 
                                            "motor_temperature", 
                                            "motor_voltage", 
                                            "motor_phase_a_current",
                                            "motor_phase_b_current",
                                            "motor_phase_c_current"}))
    {
      joint_names_.push_back(name_ifv.first);
    }
  }

  // Add extra joints from parameters, each joint will be added to joint_names_ and
  // name_if_value_mapping_ if it is not already there
  rclcpp::Parameter extra_joints;
  if (get_node()->get_parameter("extra_joints", extra_joints))
  {
    const std::vector<std::string> & extra_joints_names = extra_joints.as_string_array();
    for (const auto & extra_joint_name : extra_joints_names)
    {
      if (name_if_value_mapping_.count(extra_joint_name) == 0)
      {
        name_if_value_mapping_[extra_joint_name] = {
                                                    {"motor_error", 0.0}, 
                                                    {"motor_temperature", 0.0}, 
                                                    {"motor_voltage", 0.0},
                                                    {"motor_phase_a_current", 0.0},
                                                    {"motor_phase_b_current", 0.0},
                                                    {"motor_phase_c_current", 0.0}};
        joint_names_.push_back(extra_joint_name);
      }
    }
  }
  return true;
}

void MyActuatorStateBroadcaster::init_joint_state_msg()
{
  const size_t num_joints = joint_names_.size();

  // default initialization for joint state message
  auto & joint_state_msg = realtime_motor_state_publisher_->msg_;
  joint_state_msg.name = joint_names_;
  joint_state_msg.voltage.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
  joint_state_msg.temperature.resize(num_joints, std::numeric_limits<std::int64_t>::quiet_NaN());
  joint_state_msg.error_code.resize(num_joints, std::numeric_limits<std::int64_t>::quiet_NaN());
  joint_state_msg.current_phases.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    joint_state_msg.current_phases[i].name.resize(3);
    joint_state_msg.current_phases[i].name[0] =  "a";
    joint_state_msg.current_phases[i].name[1] =  "b";
    joint_state_msg.current_phases[i].name[2] =  "c";
    joint_state_msg.current_phases[i].current.resize(3); 
    joint_state_msg.current_phases[i].current[0] = std::numeric_limits<double>::quiet_NaN();
    joint_state_msg.current_phases[i].current[1] = std::numeric_limits<double>::quiet_NaN();
    joint_state_msg.current_phases[i].current[2] = std::numeric_limits<double>::quiet_NaN();
  }
}

bool MyActuatorStateBroadcaster::use_all_available_interfaces() const
{
  return params_.joints.empty() || params_.interfaces.empty();
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

controller_interface::return_type MyActuatorStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_)
  {
    std::string interface_name = state_interface.get_interface_name();
    if (map_interface_to_joint_state_.count(interface_name) > 0)
    {
      interface_name = map_interface_to_joint_state_[interface_name];
    }
    name_if_value_mapping_[state_interface.get_prefix_name()][interface_name] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(),
      state_interface.get_value());
  }

  if (realtime_motor_state_publisher_ && realtime_motor_state_publisher_->trylock())
  {
    auto & motor_state_msg = realtime_motor_state_publisher_->msg_;

    motor_state_msg.header.stamp = time;

    // update joint state message and dynamic joint state message
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      motor_state_msg.voltage[i] = get_value(name_if_value_mapping_, joint_names_[i], "motor_voltage");
      motor_state_msg.temperature[i] = static_cast<long int>(get_value(name_if_value_mapping_, joint_names_[i], "motor_temperature"));
      motor_state_msg.error_code[i] = static_cast<long int>(get_value(name_if_value_mapping_, joint_names_[i], "motor_error"));
      motor_state_msg.current_phases[i].current[0] = get_value(name_if_value_mapping_, joint_names_[i], "motor_phase_a_current");
      motor_state_msg.current_phases[i].current[1] = get_value(name_if_value_mapping_, joint_names_[i], "motor_phase_b_current");
      motor_state_msg.current_phases[i].current[2] = get_value(name_if_value_mapping_, joint_names_[i], "motor_phase_c_current");
    }
    realtime_motor_state_publisher_->unlockAndPublish();
  }


  return controller_interface::return_type::OK;
}

}  // namespace myactuator_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  myactuator_state_broadcaster::MyActuatorStateBroadcaster, controller_interface::ControllerInterface)