// Joint <-> servo mapping and unit conversions. Pure math: no ROS, no serial I/O.
#pragma once

#include <cstdint>
#include <string>

#include "handy_motor_api/model.hpp"

namespace handy_motor_api
{

/// One servo as described by the <param> tags in handy_description/urdf/handy.ros2_control.xacro.
struct JointConfig
{
  std::string name;
  uint8_t id = 0;
  Model model = Model::MX;
  int sign = 1;              // +1 or -1: joint direction relative to servo tick direction
  int offset_ticks = 0;      // servo tick offset of the joint zero from the model's center tick
  std::string follows;       // non-empty: follower servo that mirrors this leader joint
  double multiplier = 1.0;   // follower goal = leader command * multiplier
};

/// Joint angle (rad) -> goal ticks, rounded and clamped to [0, max_tick].
uint16_t rad_to_ticks(const JointConfig & joint, double rad);

/// Present ticks -> joint angle (rad).
double ticks_to_rad(const JointConfig & joint, uint16_t ticks);

/// Raw PRESENT_SPEED register (bit 10 = CW) -> joint velocity (rad/s).
double raw_speed_to_rad_s(const JointConfig & joint, uint16_t raw);

}  // namespace handy_motor_api
