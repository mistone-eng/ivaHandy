#include "handy_motor_api/dynamixel_bus.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

#include "handy_motor_api/model.hpp"

namespace handy_motor_api
{

namespace
{
constexpr float kProtocol = 1.0F;
}

DynamixelBus::DynamixelBus() = default;

DynamixelBus::~DynamixelBus() { close(); }

bool DynamixelBus::open(const std::string & port, int baud)
{
  close();
  port_ = dynamixel::PortHandler::getPortHandler(port.c_str());
  packet_ = dynamixel::PacketHandler::getPacketHandler(kProtocol);  // singleton, not owned
  if (!port_->openPort()) {
    last_error_ = "cannot open " + port + " (is the U2D2 plugged in? are you in the dialout group?)";
    delete port_;
    port_ = nullptr;
    return false;
  }
  if (!port_->setBaudRate(baud)) {
    last_error_ = "cannot set baud rate " + std::to_string(baud) + " on " + port;
    close();
    return false;
  }
  return true;
}

void DynamixelBus::close()
{
  if (port_ != nullptr) {
    port_->closePort();
    delete port_;
    port_ = nullptr;
  }
}

bool DynamixelBus::check(int comm_result, uint8_t dxl_error, uint8_t id, const char * what)
{
  if (comm_result != COMM_SUCCESS) {
    last_error_ = std::string(what) + " id " + std::to_string(id) + ": " +
      packet_->getTxRxResult(comm_result);
    return false;
  }
  if (dxl_error != 0) {
    // Hardware alarm bits (overload, overheat, angle limit, ...). The transfer itself worked.
    last_error_ = std::string(what) + " id " + std::to_string(id) + ": " +
      packet_->getRxPacketError(dxl_error);
    return false;
  }
  return true;
}

bool DynamixelBus::ping(uint8_t id, uint16_t * model_number)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  uint16_t model = 0;
  uint8_t err = 0;
  const int rc = packet_->ping(port_, id, &model, &err);
  if (model_number != nullptr) {*model_number = model;}
  return check(rc, err, id, "ping");
}

bool DynamixelBus::write_u8(uint8_t id, uint16_t address, uint8_t value)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  uint8_t err = 0;
  return check(packet_->write1ByteTxRx(port_, id, address, value, &err), err, id, "write1");
}

bool DynamixelBus::write_u16(uint8_t id, uint16_t address, uint16_t value)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  uint8_t err = 0;
  return check(packet_->write2ByteTxRx(port_, id, address, value, &err), err, id, "write2");
}

bool DynamixelBus::read_u16(uint8_t id, uint16_t address, uint16_t & value)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  uint8_t err = 0;
  return check(packet_->read2ByteTxRx(port_, id, address, &value, &err), err, id, "read2");
}

bool DynamixelBus::set_torque(uint8_t id, bool enable)
{
  return write_u8(id, addr::TORQUE_ENABLE, enable ? 1 : 0);
}

bool DynamixelBus::read_position_speed(uint8_t id, uint16_t & position, uint16_t & raw_speed)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  // PRESENT_POSITION (36-37) and PRESENT_SPEED (38-39) are contiguous: one 4-byte read.
  uint32_t data = 0;
  uint8_t err = 0;
  if (!check(packet_->read4ByteTxRx(port_, id, addr::PRESENT_POSITION, &data, &err), err, id,
    "read state"))
  {
    return false;
  }
  position = static_cast<uint16_t>(data & 0xFFFF);
  raw_speed = static_cast<uint16_t>(data >> 16);
  return true;
}

bool DynamixelBus::sync_write_goal_positions(
  const std::vector<std::pair<uint8_t, uint16_t>> & goals)
{
  if (!is_open()) {last_error_ = "bus not open"; return false;}
  dynamixel::GroupSyncWrite group(port_, packet_, addr::GOAL_POSITION, 2);
  for (const auto & [id, ticks] : goals) {
    uint8_t param[2] = {DXL_LOBYTE(ticks), DXL_HIBYTE(ticks)};
    if (!group.addParam(id, param)) {
      last_error_ = "sync write: duplicate id " + std::to_string(id);
      return false;
    }
  }
  const int rc = group.txPacket();  // broadcast: no status packets come back
  if (rc != COMM_SUCCESS) {
    last_error_ = std::string("sync write: ") + packet_->getTxRxResult(rc);
    return false;
  }
  return true;
}

}  // namespace handy_motor_api
