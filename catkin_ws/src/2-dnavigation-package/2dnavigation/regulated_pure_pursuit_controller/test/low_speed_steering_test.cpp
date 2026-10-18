#include <gtest/gtest.h>
#include <limits>
#include "regulated_pure_pursuit_controller/low_speed_steering.hpp"

using regulated_pure_pursuit_controller::compensateLowSpeedAngularDeadband;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(LowSpeedSteering, ConfiguredFloorAndUnchangedExecutableCommands)
{
  for (double v : {0.02, 0.04, 0.06, 0.08}) {
    const double raw = v * 0.5;
    EXPECT_DOUBLE_EQ(std::max(raw, 0.025),
      compensateLowSpeedAngularDeadband(v, 0.5, raw, 0.0, 0.025, 0.025));
    EXPECT_DOUBLE_EQ(-std::max(raw, 0.025),
      compensateLowSpeedAngularDeadband(v, -0.5, -raw, 0.0, 0.025, 0.025));
  }
  EXPECT_DOUBLE_EQ(0.025, compensateLowSpeedAngularDeadband(0.05, 0.5, 0.025, 0, 0.025, 0.025));
  EXPECT_DOUBLE_EQ(0.050, compensateLowSpeedAngularDeadband(0.02, 0.5, 0.010, 0, 0.045, 0.050));
}

TEST(LowSpeedSteering, HysteresisAndReversal)
{
  EXPECT_DOUBLE_EQ(0.008, compensateLowSpeedAngularDeadband(0.02, 0.4, 0.008, 0, 0.025, 0.025));
  EXPECT_DOUBLE_EQ(0.025, compensateLowSpeedAngularDeadband(0.02, 0.4, 0.008, 0.025, 0.025, 0.025));
  EXPECT_DOUBLE_EQ(0.004, compensateLowSpeedAngularDeadband(0.02, 0.2, 0.004, 0.025, 0.025, 0.025));
  EXPECT_DOUBLE_EQ(-0.010, compensateLowSpeedAngularDeadband(0.02, -0.5, -0.010, 0.025, 0.025, 0.025));
}

TEST(LowSpeedSteering, StopSpeedBoundariesAndDisabledConfiguration)
{
  for (double v : {0.0, 0.0149, 0.10, 0.12}) {
    EXPECT_DOUBLE_EQ(v * 0.5, compensateLowSpeedAngularDeadband(v, 0.5, v * 0.5, 0, 0.025, 0.025));
  }
  EXPECT_DOUBLE_EQ(0.025, compensateLowSpeedAngularDeadband(0.015, 0.5, 0.0075, 0, 0.025, 0.025));
  EXPECT_DOUBLE_EQ(0, compensateLowSpeedAngularDeadband(0.02, 1, 0, 0.025, 0.025, 0.025));
  for (double floor : {0.0, 0.02, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_DOUBLE_EQ(0.01, compensateLowSpeedAngularDeadband(0.02, 0.5, 0.01, 0, 0.025, floor));
  }
  EXPECT_DOUBLE_EQ(0.01, compensateLowSpeedAngularDeadband(0.02, 0.5, 0.01, 0, 0, 0.025));
}
