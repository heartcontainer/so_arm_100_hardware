#ifndef SOARM100_INTERFACE_H
#define SOARM100_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <termios.h>
#include <map>

#include <sensor_msgs/msg/joint_state.hpp>
#include <SCServo_Linux/SCServo.h>
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>

namespace so_arm_100_controller
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  enum class ControlMode
  {
    POSITION = 1,
    VELOCITY = 2,
    TORQUE = 3
  };

  struct Joint
  {
    int motor_id = 0; // Servo ID arm(1-6) base(7-9)
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;

    double position_command = 0.0;
    double velocity_command = 0.0;
    double effort_command = 0.0;

    int zero_position = 2048; // Center positions
    int servo_direction = 1;  // Direction multipliers
    ControlMode control_mode = ControlMode::POSITION;
  };

  class SOARM100Interface : public hardware_interface::SystemInterface
  {
  public:
    SOARM100Interface();
    virtual ~SOARM100Interface();

    // LifecycleNodeInterface
    CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // SystemInterface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    SMS_STS st3215_;
    std::unordered_map<std::string, Joint> joints_;
  };

} // namespace so_arm_100_controller

#endif // SOARM100_INTERFACE_H
