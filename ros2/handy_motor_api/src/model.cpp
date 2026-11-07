#include "handy_motor_api/model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace handy_motor_api
{

namespace
{
constexpr double kDeg = M_PI / 180.0;
// Values from the ROBOTIS e-Manual (Protocol 1.0 control tables).
const ModelSpec kAX{1023, 512, 300.0 * kDeg, 0.111};
const ModelSpec kRX{1023, 512, 300.0 * kDeg, 0.111};
const ModelSpec kEX{4095, 2048, 250.92 * kDeg, 0.111};
const ModelSpec kMX{4095, 2048, 360.0 * kDeg, 0.114};
}  // namespace

const ModelSpec & spec(Model model)
{
  switch (model) {
    case Model::AX: return kAX;
    case Model::RX: return kRX;
    case Model::EX: return kEX;
    case Model::MX: return kMX;
  }
  throw std::invalid_argument("unknown Dynamixel model");
}

Model parse_model(const std::string & name)
{
  std::string upper(name);
  std::transform(upper.begin(), upper.end(), upper.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  if (upper == "AX") {return Model::AX;}
  if (upper == "RX") {return Model::RX;}
  if (upper == "EX") {return Model::EX;}
  if (upper == "MX") {return Model::MX;}
  throw std::invalid_argument("unknown Dynamixel model '" + name + "' (expected AX, RX, EX or MX)");
}

std::string to_string(Model model)
{
  switch (model) {
    case Model::AX: return "AX";
    case Model::RX: return "RX";
    case Model::EX: return "EX";
    case Model::MX: return "MX";
  }
  return "?";
}

}  // namespace handy_motor_api
