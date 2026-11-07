#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "handy_motor_api/joint.hpp"
#include "handy_motor_api/model.hpp"

using namespace handy_motor_api;

namespace
{
JointConfig joint(Model m, int sign = 1, int offset = 0)
{
  JointConfig j;
  j.name = "j";
  j.id = 1;
  j.model = m;
  j.sign = sign;
  j.offset_ticks = offset;
  return j;
}
}  // namespace

TEST(Model, ParseIsCaseInsensitive)
{
  EXPECT_EQ(parse_model("mx"), Model::MX);
  EXPECT_EQ(parse_model("AX"), Model::AX);
  EXPECT_THROW(parse_model("XM"), std::invalid_argument);
}

TEST(Model, ResolutionMatchesEManual)
{
  EXPECT_NEAR(spec(Model::AX).rad_per_tick() * 180.0 / M_PI, 0.29, 0.005);
  EXPECT_NEAR(spec(Model::MX).rad_per_tick() * 180.0 / M_PI, 0.088, 0.0005);
  EXPECT_NEAR(spec(Model::EX).rad_per_tick() * 180.0 / M_PI, 0.06127, 0.0001);
}

TEST(Conversions, ZeroIsCenterTick)
{
  EXPECT_EQ(rad_to_ticks(joint(Model::AX), 0.0), 512);
  EXPECT_EQ(rad_to_ticks(joint(Model::MX), 0.0), 2048);
  EXPECT_EQ(rad_to_ticks(joint(Model::EX), 0.0), 2048);
  EXPECT_DOUBLE_EQ(ticks_to_rad(joint(Model::MX), 2048), 0.0);
}

TEST(Conversions, RoundTripAllModels)
{
  for (Model m : {Model::AX, Model::RX, Model::EX, Model::MX}) {
    for (int sign : {1, -1}) {
      const auto j = joint(m, sign, 17);
      for (double rad : {-1.2, -0.3, 0.0, 0.25, 1.0}) {
        EXPECT_NEAR(ticks_to_rad(j, rad_to_ticks(j, rad)), rad, spec(m).rad_per_tick())
          << to_string(m) << " sign " << sign << " rad " << rad;
      }
    }
  }
}

TEST(Conversions, SignFlipsDirection)
{
  EXPECT_GT(rad_to_ticks(joint(Model::MX, 1), 0.5), 2048);
  EXPECT_LT(rad_to_ticks(joint(Model::MX, -1), 0.5), 2048);
}

TEST(Conversions, OffsetShiftsZero)
{
  EXPECT_EQ(rad_to_ticks(joint(Model::MX, 1, 100), 0.0), 2148);
  EXPECT_DOUBLE_EQ(ticks_to_rad(joint(Model::MX, 1, 100), 2148), 0.0);
}

TEST(Conversions, GoalIsClamped)
{
  EXPECT_EQ(rad_to_ticks(joint(Model::AX), 100.0), 1023);
  EXPECT_EQ(rad_to_ticks(joint(Model::AX), -100.0), 0);
  EXPECT_EQ(rad_to_ticks(joint(Model::MX), 100.0), 4095);
}

TEST(Conversions, SpeedDirectionBit)
{
  const auto j = joint(Model::MX);
  const double one_unit = 0.114 * 2.0 * M_PI / 60.0;
  EXPECT_NEAR(raw_speed_to_rad_s(j, 100), 100 * one_unit, 1e-9);           // CCW
  EXPECT_NEAR(raw_speed_to_rad_s(j, 1024 + 100), -100 * one_unit, 1e-9);   // CW
  EXPECT_NEAR(raw_speed_to_rad_s(joint(Model::MX, -1), 100), -100 * one_unit, 1e-9);
  EXPECT_DOUBLE_EQ(raw_speed_to_rad_s(j, 0), 0.0);
}
