#include "handy_motor_api/joint.hpp"

#include <algorithm>
#include <cmath>

namespace handy_motor_api
{

// ticks = center + offset + sign * rad / rad_per_tick
uint16_t rad_to_ticks(const JointConfig & joint, double rad)
{
  const ModelSpec & s = spec(joint.model);
  const double ticks = static_cast<double>(s.center_tick) + joint.offset_ticks +
    joint.sign * rad / s.rad_per_tick();
  const double clamped = std::clamp(std::round(ticks), 0.0, static_cast<double>(s.max_tick));
  return static_cast<uint16_t>(clamped);
}

double ticks_to_rad(const JointConfig & joint, uint16_t ticks)
{
  const ModelSpec & s = spec(joint.model);
  const double rel = static_cast<double>(ticks) - s.center_tick - joint.offset_ticks;
  return joint.sign * rel * s.rad_per_tick();
}

double raw_speed_to_rad_s(const JointConfig & joint, uint16_t raw)
{
  const ModelSpec & s = spec(joint.model);
  const double magnitude = static_cast<double>(raw & 0x3FF) * s.rpm_per_unit * 2.0 * M_PI / 60.0;
  const double tick_dir = (raw & 0x400) ? -1.0 : 1.0;  // bit 10 set = CW = decreasing ticks
  return joint.sign * tick_dir * magnitude;
}

}  // namespace handy_motor_api
