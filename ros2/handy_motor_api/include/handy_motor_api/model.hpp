// Dynamixel Protocol 1.0 model data for the servos used on Handy (AX, RX, EX, MX).
// Pure data + math: no ROS, no serial I/O.
#pragma once

#include <cstdint>
#include <string>

namespace handy_motor_api
{

enum class Model { AX, RX, EX, MX };

/// Protocol 1.0 control-table addresses. Identical for AX-12, RX-28/64, EX-106+ and MX (P1 firmware).
namespace addr
{
constexpr uint16_t MODEL_NUMBER = 0;      // 2 bytes
constexpr uint16_t TORQUE_ENABLE = 24;    // 1 byte
constexpr uint16_t GOAL_POSITION = 30;    // 2 bytes
constexpr uint16_t MOVING_SPEED = 32;     // 2 bytes, 0 = max speed (no limit!)
constexpr uint16_t TORQUE_LIMIT = 34;     // 2 bytes, 0..1023
constexpr uint16_t PRESENT_POSITION = 36; // 2 bytes
constexpr uint16_t PRESENT_SPEED = 38;    // 2 bytes, bit 10 = direction (1 = CW)
}  // namespace addr

struct ModelSpec
{
  uint16_t max_tick;     // highest valid position value
  uint16_t center_tick;  // tick that maps to 0 rad (before offset)
  double range_rad;      // angle covered by 0..max_tick+1
  double rpm_per_unit;   // velocity unit for MOVING_SPEED / PRESENT_SPEED

  double rad_per_tick() const { return range_rad / (static_cast<double>(max_tick) + 1.0); }
};

/// Spec for a model. AX/RX: 300 deg / 1024, EX-106: 250.92 deg / 4096, MX: 360 deg / 4096.
const ModelSpec & spec(Model model);

/// "AX" | "RX" | "EX" | "MX" (case-insensitive). Throws std::invalid_argument otherwise.
Model parse_model(const std::string & name);
std::string to_string(Model model);

}  // namespace handy_motor_api
