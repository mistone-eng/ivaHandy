#include "handy_hardware/dynamixel_system.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace handy_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace hi = hardware_interface;

namespace
{
std::string param(const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback = "")
{
  auto it = params.find(key);
  if (it != params.end()) {return it->second;}
  if (!fallback.empty()) {return fallback;}
  throw std::runtime_error("missing <param name=\"" + key + "\">");
}
}  // namespace

CallbackReturn DynamixelSystem::on_init(const hi::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    const auto & hw = info_.hardware_parameters;
    port_ = param(hw, "port", "/dev/ttyUSB0");
    baud_ = std::stoi(param(hw, "baud", "1000000"));
    moving_speed_ = std::stoi(param(hw, "moving_speed", "100"));
    torque_limit_ = std::stoi(param(hw, "torque_limit", "800"));
    if (moving_speed_ <= 0) {
      // 0 means "no speed limit" on Protocol 1.0 servos -- never allow it implicitly.
      throw std::runtime_error("moving_speed must be > 0 (0 = unlimited on Protocol 1.0)");
    }

    for (const auto & j : info_.joints) {
      Joint joint;
      joint.cfg.name = j.name;
      joint.cfg.id = static_cast<uint8_t>(std::stoi(param(j.parameters, "id")));
      joint.cfg.model = handy_motor_api::parse_model(param(j.parameters, "model"));
      joint.cfg.sign = std::stoi(param(j.parameters, "sign", "1"));
      joint.cfg.offset_ticks = std::stoi(param(j.parameters, "offset_ticks", "0"));
      auto f = j.parameters.find("follows");
      if (f != j.parameters.end()) {
        joint.cfg.follows = f->second;
        joint.cfg.multiplier = std::stod(param(j.parameters, "follow_multiplier", "1"));
      }
      for (const auto & ci : j.command_interfaces) {
        if (ci.name == hi::HW_IF_POSITION) {joint.commanded = true;}
        else {throw std::runtime_error(j.name + ": only 'position' command interface supported");}
      }
      joints_.push_back(joint);
    }

    // Resolve followers to their leader joint.
    for (auto & joint : joints_) {
      if (joint.cfg.follows.empty()) {continue;}
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].cfg.name == joint.cfg.follows) {joint.leader = static_cast<int>(i);}
      }
      if (joint.leader < 0 || !joints_[joint.leader].commanded) {
        throw std::runtime_error(joint.cfg.name + " follows unknown/uncommanded joint '" +
                joint.cfg.follows + "'");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Bad ros2_control description: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "%zu joints on %s @ %d baud", joints_.size(), port_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!bus_.open(port_, baud_)) {
    RCLCPP_FATAL(logger_, "%s", bus_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  bool all_found = true;
  for (const auto & j : joints_) {
    if (!bus_.ping(j.cfg.id)) {
      RCLCPP_ERROR(logger_, "%s (id %d) not responding: %s", j.cfg.name.c_str(), j.cfg.id,
        bus_.last_error().c_str());
      all_found = false;
    }
  }
  if (!all_found) {
    bus_.close();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "All %zu servos responded", joints_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  bus_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelSystem::on_activate(const rclcpp_lifecycle::State &)
{
  namespace addr = handy_motor_api::addr;
  // Start from where the arm IS, so activation never causes a jump.
  if (read(rclcpp::Time(), rclcpp::Duration(0, 0)) != return_type::OK || failed_cycles_ > 0) {
    RCLCPP_FATAL(logger_, "Could not read initial positions: %s", bus_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  for (auto & j : joints_) {
    j.command = j.position;
    const auto id = j.cfg.id;
    if (!bus_.write_u16(id, addr::MOVING_SPEED, static_cast<uint16_t>(moving_speed_)) ||
      !bus_.write_u16(id, addr::TORQUE_LIMIT, static_cast<uint16_t>(torque_limit_)) ||
      !bus_.write_u16(id, addr::GOAL_POSITION, handy_motor_api::rad_to_ticks(j.cfg, j.position)) ||
      !bus_.set_torque(id, true))
    {
      RCLCPP_FATAL(logger_, "Activating %s failed: %s", j.cfg.name.c_str(),
        bus_.last_error().c_str());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(logger_, "Torque ON (moving_speed %d, torque_limit %d)", moving_speed_,
    torque_limit_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (const auto & j : joints_) {
    if (!bus_.set_torque(j.cfg.id, false)) {
      RCLCPP_ERROR(logger_, "Torque off %s failed: %s", j.cfg.name.c_str(),
        bus_.last_error().c_str());
    }
  }
  RCLCPP_INFO(logger_, "Torque OFF");
  return CallbackReturn::SUCCESS;
}

std::vector<hi::StateInterface> DynamixelSystem::export_state_interfaces()
{
  std::vector<hi::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.cfg.name, hi::HW_IF_POSITION, &j.position);
    out.emplace_back(j.cfg.name, hi::HW_IF_VELOCITY, &j.velocity);
  }
  return out;
}

std::vector<hi::CommandInterface> DynamixelSystem::export_command_interfaces()
{
  std::vector<hi::CommandInterface> out;
  for (auto & j : joints_) {
    if (j.commanded) {out.emplace_back(j.cfg.name, hi::HW_IF_POSITION, &j.command);}
  }
  return out;
}

return_type DynamixelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  bool ok = true;
  for (auto & j : joints_) {
    uint16_t ticks = 0, raw_speed = 0;
    if (!bus_.read_position_speed(j.cfg.id, ticks, raw_speed)) {
      ok = false;  // keep the last known value
      continue;
    }
    j.position = handy_motor_api::ticks_to_rad(j.cfg, ticks);
    j.velocity = handy_motor_api::raw_speed_to_rad_s(j.cfg, raw_speed);
  }
  if (ok) {
    failed_cycles_ = 0;
    return return_type::OK;
  }
  RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000, "read: %s", bus_.last_error().c_str());
  if (++failed_cycles_ >= max_failed_cycles_) {
    RCLCPP_FATAL(logger_, "%d consecutive failed reads -- reporting ERROR", failed_cycles_);
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type DynamixelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<std::pair<uint8_t, uint16_t>> goals;
  goals.reserve(joints_.size());
  for (const auto & j : joints_) {
    double target = std::numeric_limits<double>::quiet_NaN();
    if (j.commanded) {target = j.command;}
    else if (j.leader >= 0) {target = joints_[j.leader].command * j.cfg.multiplier;}
    if (std::isfinite(target)) {goals.emplace_back(j.cfg.id, handy_motor_api::rad_to_ticks(j.cfg, target));}
  }
  if (!goals.empty() && !bus_.sync_write_goal_positions(goals)) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000, "write: %s", bus_.last_error().c_str());
  }
  return return_type::OK;
}

}  // namespace handy_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(handy_hardware::DynamixelSystem, hardware_interface::SystemInterface)
