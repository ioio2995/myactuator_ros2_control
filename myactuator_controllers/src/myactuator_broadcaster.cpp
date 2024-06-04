#include "myactuator_controllers/myactuator_broadcaster.hpp"

#include <memory>
#include <string>

namespace myactuator_broadcaster
{
  MyActuatorBroadcaster::MyActuatorBroadcaster(): controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn MyActuatorBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MyActuatorBroadcaster::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    const bool no_interface_names_defined =
        params_.interface_names.force.x.empty() && params_.interface_names.force.y.empty() &&
        params_.interface_names.force.z.empty() && params_.interface_names.torque.x.empty() &&
        params_.interface_names.torque.y.empty() && params_.interface_names.torque.z.empty();

    if (params_.sensor_name.empty() && no_interface_names_defined)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "'sensor_name' or at least one "
          "'interface_names.[force|torque].[x|y|z]' parameter has to be specified.");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!params_.sensor_name.empty() && !no_interface_names_defined)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "both 'sensor_name' and "
          "'interface_names.[force|torque].[x|y|z]' parameters can not be specified together.");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!params_.sensor_name.empty())
    {
      myactuator_ = std::make_unique<semantic_components::ForceTorqueSensor>(
          semantic_components::ForceTorqueSensor(params_.sensor_name));
    }
    else
    {
      auto const &force_names = params_.interface_names.force;
      auto const &torque_names = params_.interface_names.torque;
      myactuator_ = std::make_unique<semantic_components::ForceTorqueSensor>(
          semantic_components::ForceTorqueSensor(
              force_names.x, force_names.y, force_names.z, torque_names.x, torque_names.y,
              torque_names.z));
    }

    try
    {
      // register ft sensor data publisher
      sensor_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "~/wrench", rclcpp::SystemDefaultsQoS());
      realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    realtime_publisher_->lock();
    realtime_publisher_->msg_.header.frame_id = params_.frame_id;
    realtime_publisher_->unlock();

    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  MyActuatorBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  MyActuatorBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = myactuator_->get_state_interface_names();
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn MyActuatorBroadcaster::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    myactuator_->assign_loaned_state_interfaces(state_interfaces_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MyActuatorBroadcaster::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    myactuator_->release_interfaces();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type MyActuatorBroadcaster::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (realtime_publisher_ && realtime_publisher_->trylock())
    {
      realtime_publisher_->msg_.header.stamp = time;
      myactuator_->get_values_as_message(realtime_publisher_->msg_.wrench);
      realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace myactuator_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    myactuator_broadcaster::MyActuatorBroadcaster,
    controller_interface::ControllerInterface)