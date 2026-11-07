// Thin, ROS-free wrapper around the Dynamixel SDK for one Protocol 1.0 serial bus.
// Every call returns false on failure; last_error() explains why.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace dynamixel
{
class PortHandler;
class PacketHandler;
}  // namespace dynamixel

namespace handy_motor_api
{

class DynamixelBus
{
public:
  DynamixelBus();
  ~DynamixelBus();
  DynamixelBus(const DynamixelBus &) = delete;
  DynamixelBus & operator=(const DynamixelBus &) = delete;

  bool open(const std::string & port, int baud);
  void close();
  bool is_open() const { return port_ != nullptr; }

  /// Ping a servo; on success model_number holds the MODEL_NUMBER register.
  bool ping(uint8_t id, uint16_t * model_number = nullptr);

  bool set_torque(uint8_t id, bool enable);
  bool write_u8(uint8_t id, uint16_t address, uint8_t value);
  bool write_u16(uint8_t id, uint16_t address, uint16_t value);
  bool read_u16(uint8_t id, uint16_t address, uint16_t & value);

  /// PRESENT_POSITION and PRESENT_SPEED in one 4-byte read.
  bool read_position_speed(uint8_t id, uint16_t & position, uint16_t & raw_speed);

  /// GOAL_POSITION for many servos in one SyncWrite packet: {id, ticks}.
  bool sync_write_goal_positions(const std::vector<std::pair<uint8_t, uint16_t>> & goals);

  const std::string & last_error() const { return last_error_; }

private:
  bool check(int comm_result, uint8_t dxl_error, uint8_t id, const char * what);

  dynamixel::PortHandler * port_ = nullptr;
  dynamixel::PacketHandler * packet_ = nullptr;
  std::string last_error_;
};

}  // namespace handy_motor_api
