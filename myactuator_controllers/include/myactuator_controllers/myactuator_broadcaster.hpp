#ifndef MYACTUATOR_BROADCASTER_HPP_
#define MYACTUATOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "myactuator_controllers/visibility_control.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"


namespace myactuator_broadcaster {
class MyActuatorBroadcaster : public controller_interface::ControllerInterface
{
public:

  MYACTUATOR_BROADCASTER_PUBLIC
  MyActuatorBroadcaster();

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MYACTUATOR_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MYACTUATOR_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
    std::string sensor_name_;
    std::array<std::string, 6> interface_names_;
    std::string frame_id_;
    std::string topic_name_;
  
  std::unique_ptr<semantic_components::ForceTorqueSensor> myactuator_;

  using StatePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace myactuator_broadcaster

#endif // MYACTUATOR_BROADCASTER_HPP_