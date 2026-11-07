// ros2_control SystemInterface for Handy. Translates ros2_control <-> handy_motor_api.
// No Dynamixel register logic lives here: that belongs in handy_motor_api.
#pragma once

#include <string>
#include <vector>

#include "handy_motor_api/dynamixel_bus.hpp"
#include "handy_motor_api/joint.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

namespace handy_hardware
{

class DynamixelSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Joint
  {
    handy_motor_api::JointConfig cfg;
    bool commanded = false;  // has a position command interface (followers don't)
    int leader = -1;         // index of the leader joint, for followers
    double position = 0.0;
    double velocity = 0.0;
    double command = 0.0;
  };

  std::string port_;
  int baud_ = 1000000;
  int moving_speed_ = 100;
  int torque_limit_ = 800;
  int max_failed_cycles_ = 50;  // consecutive read() failures before reporting ERROR

  std::vector<Joint> joints_;
  handy_motor_api::DynamixelBus bus_;
  int failed_cycles_ = 0;

  rclcpp::Logger logger_ = rclcpp::get_logger("handy_hardware");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace handy_hardware
