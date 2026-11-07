// handy_motor_cli: talk to Handy's Dynamixels with NO ROS running.
// Use it to check wiring/IDs before launching ros2_control, or to make the arm limp in a hurry.
//
//   handy_motor_cli [--port /dev/ttyUSB0] [--baud 1000000] <command>
//     scan [max_id]            ping ids 1..max_id (default 20), print model numbers
//     read <id>                raw PRESENT_POSITION / PRESENT_SPEED
//     torque <id|all> on|off   enable/disable torque ("all" = ids 1-9)
//     goal <id> <ticks> [spd]  move one servo to raw ticks at MOVING_SPEED spd (default 50, slow)
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "handy_motor_api/dynamixel_bus.hpp"
#include "handy_motor_api/model.hpp"

using handy_motor_api::DynamixelBus;
namespace addr = handy_motor_api::addr;

namespace
{
int usage()
{
  std::cerr <<
    "usage: handy_motor_cli [--port P] [--baud B] <command>\n"
    "  scan [max_id]            ping ids 1..max_id (default 20)\n"
    "  read <id>                raw present position / speed\n"
    "  torque <id|all> on|off   enable/disable torque (all = ids 1-9)\n"
    "  goal <id> <ticks> [spd]  move one servo to raw ticks (moving speed default 50)\n";
  return 2;
}

int fail(const DynamixelBus & bus)
{
  std::cerr << "error: " << bus.last_error() << "\n";
  return 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyUSB0";
  int baud = 1000000;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) {port = argv[++i];}
    else if (a == "--baud" && i + 1 < argc) {baud = std::atoi(argv[++i]);}
    else {args.push_back(a);}
  }
  if (args.empty()) {return usage();}

  DynamixelBus bus;
  if (!bus.open(port, baud)) {return fail(bus);}
  const std::string & cmd = args[0];

  if (cmd == "scan") {
    const int max_id = args.size() > 1 ? std::atoi(args[1].c_str()) : 20;
    int found = 0;
    for (int id = 1; id <= max_id; ++id) {
      uint16_t model = 0;
      if (bus.ping(static_cast<uint8_t>(id), &model)) {
        std::cout << "id " << id << "  model_number " << model << "\n";
        ++found;
      }
    }
    std::cout << found << " servo(s) found on " << port << " @ " << baud << "\n";
    return found > 0 ? 0 : 1;
  }

  if (cmd == "read" && args.size() == 2) {
    const auto id = static_cast<uint8_t>(std::atoi(args[1].c_str()));
    uint16_t pos = 0, spd = 0;
    if (!bus.read_position_speed(id, pos, spd)) {return fail(bus);}
    std::cout << "id " << int(id) << "  position " << pos << "  speed_raw " << spd
              << ((spd & 0x400) ? " (CW)" : " (CCW)") << "\n";
    return 0;
  }

  if (cmd == "torque" && args.size() == 3 && (args[2] == "on" || args[2] == "off")) {
    std::vector<uint8_t> ids;
    if (args[1] == "all") {for (uint8_t id = 1; id <= 9; ++id) {ids.push_back(id);}}
    else {ids.push_back(static_cast<uint8_t>(std::atoi(args[1].c_str())));}
    int rc = 0;
    for (auto id : ids) {
      if (!bus.set_torque(id, args[2] == "on")) {std::cerr << bus.last_error() << "\n"; rc = 1;}
    }
    std::cout << "torque " << args[2] << " sent to " << ids.size() << " servo(s)\n";
    return rc;
  }

  if (cmd == "goal" && args.size() >= 3) {
    const auto id = static_cast<uint8_t>(std::atoi(args[1].c_str()));
    const auto ticks = static_cast<uint16_t>(std::atoi(args[2].c_str()));
    const auto speed = static_cast<uint16_t>(args.size() > 3 ? std::atoi(args[3].c_str()) : 50);
    if (speed == 0) {std::cerr << "refusing speed 0 (= unlimited on Protocol 1.0)\n"; return 2;}
    if (!bus.write_u16(id, addr::MOVING_SPEED, speed) || !bus.set_torque(id, true) ||
      !bus.write_u16(id, addr::GOAL_POSITION, ticks))
    {
      return fail(bus);
    }
    std::cout << "id " << int(id) << " -> " << ticks << " ticks at speed " << speed << "\n";
    return 0;
  }

  return usage();
}
