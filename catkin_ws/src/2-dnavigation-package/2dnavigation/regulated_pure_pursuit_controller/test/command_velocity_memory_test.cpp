#include <limits>

#include <gtest/gtest.h>

#include "regulated_pure_pursuit_controller/command_velocity_memory.hpp"

namespace regulated_pure_pursuit_controller
{

TEST(CommandVelocityMemory, RetainsRecentCommand)
{
  EXPECT_DOUBLE_EQ(0.06, retainCommandVelocityIfFresh(0.06, 0.1, 0.5));
  EXPECT_DOUBLE_EQ(0.06, retainCommandVelocityIfFresh(0.06, 0.5, 0.5));
}

TEST(CommandVelocityMemory, ClearsExpiredOrDisabledCommand)
{
  EXPECT_DOUBLE_EQ(0.0, retainCommandVelocityIfFresh(0.06, 0.5001, 0.5));
  EXPECT_DOUBLE_EQ(0.0, retainCommandVelocityIfFresh(0.06, 0.1, 0.0));
}

TEST(CommandVelocityMemory, ClearsInvalidTimeValues)
{
  EXPECT_DOUBLE_EQ(0.0, retainCommandVelocityIfFresh(0.06, -0.1, 0.5));
  EXPECT_DOUBLE_EQ(
    0.0,
    retainCommandVelocityIfFresh(
      0.06, std::numeric_limits<double>::infinity(), 0.5));
}

}  // namespace regulated_pure_pursuit_controller

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
