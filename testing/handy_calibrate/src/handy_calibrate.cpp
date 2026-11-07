// handy_calibrate: measure Handy's joint zero offsets and ranges with NO ROS running.
//
//   ros2 run handy_calibrate handy_calibrate check            live joint angles with current calibration
//   ros2 run handy_calibrate handy_calibrate zero             measure offset_ticks -> calibration.yaml
//   ros2 run handy_calibrate handy_calibrate limits [--out F] measure joint ranges by hand -> YAML
//
// Options: --port P  --baud B  (default: bus section of handy_description/config/hardware.yaml)
//          --cal F   calibration file to read/write (default: handy_description's calibration.yaml)
//
// Stop ros2_control first (`handy kill`): only one program can own the serial port.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "handy_motor_api/dynamixel_bus.hpp"
#include "handy_motor_api/joint.hpp"
#include "handy_motor_api/model.hpp"

namespace fs = std::filesystem;
using namespace handy_motor_api;
using Clock = std::chrono::steady_clock;

namespace
{

struct Setup
{
  std::string port;
  int baud = 1000000;
  std::vector<JointConfig> joints;  // ordered as in hardware.yaml
  fs::path cal_file;
};

int usage()
{
  std::cerr <<
    "usage: handy_calibrate [--port P] [--baud B] [--cal FILE] <check|zero|limits> [--out FILE]\n"
    "  check   print live joint angles using the current calibration (Enter to stop)\n"
    "  zero    hold the arm in the URDF zero pose -> writes offset_ticks to calibration.yaml\n"
    "  limits  move each joint through its range by hand -> writes measured limits YAML\n";
  return 2;
}

bool confirm(const std::string & question)
{
  std::cout << question << " [type 'yes']: " << std::flush;
  std::string answer;
  std::getline(std::cin, answer);
  return answer == "yes";
}

void wait_enter(const std::string & prompt)
{
  std::cout << prompt << std::flush;
  std::string line;
  std::getline(std::cin, line);
}

Setup load_setup(const std::string & cal_override)
{
  const fs::path cfg = fs::path(ament_index_cpp::get_package_share_directory("handy_description")) /
    "config";
  Setup s;
  const YAML::Node hw = YAML::LoadFile((cfg / "hardware.yaml").string());
  s.port = hw["bus"]["port"].as<std::string>();
  s.baud = hw["bus"]["baud"].as<int>();

  // With `colcon build --symlink-install` the installed file is a symlink into src/, so resolving
  // it means calibration lands in the source tree (and in git), not in a throwaway install/ copy.
  s.cal_file = cal_override.empty() ? fs::path(cfg / "calibration.yaml") : fs::path(cal_override);
  if (fs::is_symlink(s.cal_file)) {s.cal_file = fs::canonical(s.cal_file);}
  YAML::Node cal;
  if (fs::exists(s.cal_file)) {cal = YAML::LoadFile(s.cal_file.string());}

  for (const auto & item : hw["joints"]) {
    JointConfig j;
    j.name = item.first.as<std::string>();
    const YAML::Node & n = item.second;
    j.id = static_cast<uint8_t>(n["id"].as<int>());
    j.model = parse_model(n["model"].as<std::string>());
    j.sign = n["sign"] ? n["sign"].as<int>() : 1;
    if (n["follows"]) {
      j.follows = n["follows"].as<std::string>();
      j.multiplier = n["multiplier"] ? n["multiplier"].as<double>() : 1.0;
    }
    if (cal["offset_ticks"] && cal["offset_ticks"][j.name]) {
      j.offset_ticks = cal["offset_ticks"][j.name].as<int>();
    }
    s.joints.push_back(j);
  }
  return s;
}

/// Read every joint once; false if any read fails (error printed).
bool read_all(DynamixelBus & bus, const Setup & s, std::vector<uint16_t> & ticks)
{
  ticks.assign(s.joints.size(), 0);
  for (size_t i = 0; i < s.joints.size(); ++i) {
    uint16_t speed = 0;
    if (!bus.read_position_speed(s.joints[i].id, ticks[i], speed)) {
      std::cerr << s.joints[i].name << ": " << bus.last_error() << "\n";
      return false;
    }
  }
  return true;
}

bool torque_off_all(DynamixelBus & bus, const Setup & s)
{
  if (!confirm("Torque will be DISABLED on all joints -- the arm will go limp. Support it now.")) {
    return false;
  }
  bool ok = true;
  for (const auto & j : s.joints) {
    if (!bus.set_torque(j.id, false)) {std::cerr << bus.last_error() << "\n"; ok = false;}
  }
  return ok;
}

/// Runs `body` repeatedly at `hz` until the user presses Enter.
template<typename F>
void loop_until_enter(double hz, F body)
{
  std::atomic<bool> stop{false};
  std::thread input([&stop] {std::string l; std::getline(std::cin, l); stop = true;});
  const auto period = std::chrono::duration<double>(1.0 / hz);
  while (!stop) {
    const auto next = Clock::now() + std::chrono::duration_cast<Clock::duration>(period);
    body();
    std::this_thread::sleep_until(next);
  }
  input.join();
}

int cmd_check(DynamixelBus & bus, const Setup & s)
{
  std::cout << "Live joint angles (rad) using " << s.cal_file << "\nPress Enter to stop.\n";
  std::vector<uint16_t> ticks;
  loop_until_enter(5.0, [&] {
    if (!read_all(bus, s, ticks)) {return;}
    std::cout << "\r";
    for (size_t i = 0; i < s.joints.size(); ++i) {
      std::cout << s.joints[i].name.substr(6) << ":" << std::showpos << std::fixed
                << std::setprecision(2) << ticks_to_rad(s.joints[i], ticks[i]) << std::noshowpos
                << " ";
    }
    std::cout << std::flush;
  });
  std::cout << "\n";
  return 0;
}

int cmd_zero(DynamixelBus & bus, Setup & s)
{
  std::cout <<
    "ZERO CALIBRATION\n"
    "Pose the arm in the URDF zero pose (SRDF 'home': all arm joints 0, gripper 'hand_neutral').\n"
    "Compare against RViz with `ros2 launch handy_description display.launch.py` if unsure.\n\n";
  if (!torque_off_all(bus, s)) {return 1;}
  wait_enter("Hold the arm in the zero pose, then press Enter to sample...");

  constexpr int kSamples = 20;
  std::vector<double> sum(s.joints.size(), 0.0);
  std::vector<uint16_t> ticks;
  for (int k = 0; k < kSamples; ++k) {
    if (!read_all(bus, s, ticks)) {return 1;}
    for (size_t i = 0; i < ticks.size(); ++i) {sum[i] += ticks[i];}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\n" << std::left << std::setw(9) << "joint" << std::setw(6) << "id"
            << std::setw(8) << "ticks" << std::setw(8) << "old" << "new offset_ticks\n";
  std::map<std::string, int> offsets;
  for (size_t i = 0; i < s.joints.size(); ++i) {
    const auto & j = s.joints[i];
    const double mean = sum[i] / kSamples;
    const int offset = static_cast<int>(std::lround(mean)) - spec(j.model).center_tick;
    offsets[j.name] = offset;
    std::cout << std::setw(9) << j.name << std::setw(6) << int(j.id) << std::setw(8)
              << std::lround(mean) << std::setw(8) << j.offset_ticks << offset
              << "  (" << std::showpos << std::setprecision(3)
              << (offset - j.offset_ticks) * spec(j.model).rad_per_tick() << std::noshowpos
              << " rad change)\n";
  }
  if (!confirm("\nWrite these offsets to " + s.cal_file.string() + "?")) {return 1;}

  std::ofstream out(s.cal_file);
  out << "# Joint zero offsets in servo ticks: ticks_at_zero = model_center_tick + offset_ticks.\n"
         "# GENERATED by `ros2 run handy_calibrate handy_calibrate zero` -- re-run instead of "
         "hand-editing.\n"
         "offset_ticks:\n";
  for (const auto & j : s.joints) {out << "  " << j.name << ": " << offsets[j.name] << "\n";}
  std::cout << "Wrote " << s.cal_file << "\nRestart ros2_control to use it. Verify with "
               "`handy_calibrate check`.\n";
  return 0;
}

int cmd_limits(DynamixelBus & bus, const Setup & s, const std::string & out_file)
{
  std::cout <<
    "LIMIT MEASUREMENT (uses the current zero calibration)\n"
    "Move each joint slowly to BOTH mechanical/cable limits by hand. Press Enter when done.\n\n";
  if (!torque_off_all(bus, s)) {return 1;}

  std::vector<double> lo(s.joints.size(), std::numeric_limits<double>::infinity());
  std::vector<double> hi(s.joints.size(), -std::numeric_limits<double>::infinity());
  std::vector<uint16_t> ticks;
  loop_until_enter(20.0, [&] {
    if (!read_all(bus, s, ticks)) {return;}
    std::cout << "\r";
    for (size_t i = 0; i < s.joints.size(); ++i) {
      const double r = ticks_to_rad(s.joints[i], ticks[i]);
      lo[i] = std::min(lo[i], r);
      hi[i] = std::max(hi[i], r);
      std::cout << s.joints[i].name.substr(6) << "[" << std::fixed << std::setprecision(2)
                << lo[i] << "," << hi[i] << "] ";
    }
    std::cout << std::flush;
  });

  std::ofstream out(out_file);
  out << "# Measured joint ranges (rad), from `handy_calibrate limits` with calibration\n"
         "# " << s.cal_file.string() << "\n"
         "# Compare with handy_moveit_config/config/joint_limits.yaml and copy over by hand,\n"
         "# leaving a safety margin. Not used by anything automatically.\n"
         "measured_limits:\n";
  for (size_t i = 0; i < s.joints.size(); ++i) {
    out << "  " << s.joints[i].name << ": {min_position: " << std::setprecision(3) << lo[i]
        << ", max_position: " << hi[i] << "}\n";
  }
  std::cout << "\nWrote " << out_file << "\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port, cal, out = "handy_limits_measured.yaml", cmd;
  int baud = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) {port = argv[++i];}
    else if (a == "--baud" && i + 1 < argc) {baud = std::atoi(argv[++i]);}
    else if (a == "--cal" && i + 1 < argc) {cal = argv[++i];}
    else if (a == "--out" && i + 1 < argc) {out = argv[++i];}
    else if (cmd.empty()) {cmd = a;}
    else {return usage();}
  }
  if (cmd != "check" && cmd != "zero" && cmd != "limits") {return usage();}

  Setup s;
  try {
    s = load_setup(cal);
  } catch (const std::exception & e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 1;
  }
  if (!port.empty()) {s.port = port;}
  if (baud > 0) {s.baud = baud;}

  DynamixelBus bus;
  if (!bus.open(s.port, s.baud)) {
    std::cerr << bus.last_error() << "\n(is ros2_control still running? try `handy kill`)\n";
    return 1;
  }
  if (cmd == "check") {return cmd_check(bus, s);}
  if (cmd == "zero") {return cmd_zero(bus, s);}
  return cmd_limits(bus, s, out);
}
