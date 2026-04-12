#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "px4adrc/msg/flat_trajectory_reference.hpp"
#include "px4adrc/common.hpp"
#include "px4adrc/types.hpp"

TEST(FlatTrajectoryReferenceMessage, DefaultConstructs)
{
  px4adrc::msg::FlatTrajectoryReference msg{};
  EXPECT_FLOAT_EQ(msg.yaw, 0.0F);
}

TEST(CommonMath, ClampScalarClampsToBounds)
{
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(-1.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(0.4, 0.0, 1.0), 0.4);
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(2.0, 0.0, 1.0), 1.0);
}

TEST(CommonMath, ThrustModelFactorOneMapsBySquareRoot)
{
  px4adrc::ControllerParams params{};
  params.max_total_thrust_n = 40.0;
  params.thrust_model_factor = 1.0;

  const std::array<double, px4adrc::kMotorCount> thrusts{{10.0, 2.5, 0.0, 5.0}};
  const auto throttles = px4adrc::motor_thrusts_to_throttles(thrusts, params);

  EXPECT_NEAR(throttles[0], 1.0, 1e-9);
  EXPECT_NEAR(throttles[1], 0.5, 1e-9);
  EXPECT_NEAR(throttles[2], 0.0, 1e-9);
  EXPECT_NEAR(throttles[3], std::sqrt(0.5), 1e-9);
}

TEST(CommonMath, CollectiveThrustNormalizesByMaxTotal)
{
  px4adrc::ControllerParams params{};
  params.max_total_thrust_n = 40.0;

  const std::array<double, px4adrc::kMotorCount> thrusts{{5.0, 5.0, 5.0, 5.0}};
  EXPECT_DOUBLE_EQ(
    px4adrc::collective_thrust_normalized_from_motor_thrusts(thrusts, params),
    0.5);
}

TEST(CommonMath, LowPassVectorReturnsInputWhenFilterDisabled)
{
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);
  const Eigen::Vector3d sample(1.0, -2.0, 3.0);

  const auto filtered = px4adrc::low_pass_vec3(previous, sample, 0.0, 0.01);
  EXPECT_TRUE(filtered.isApprox(sample, 1e-12));
}

TEST(CommonMath, LowPassVectorBlendsTowardSample)
{
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);
  const Eigen::Vector3d sample(1.0, 2.0, 3.0);

  const auto filtered = px4adrc::low_pass_vec3(previous, sample, 1.0, 0.1);

  EXPECT_NEAR(filtered.x(), 0.38586954509503757, 1e-12);
  EXPECT_NEAR(filtered.y(), 0.7717390901900751, 1e-12);
  EXPECT_NEAR(filtered.z(), 1.1576086352851127, 1e-12);
}

TEST(CommonMath, AllocationProducesHoverSymmetry)
{
  px4adrc::ControllerParams params{};
  params.arm_length_m = 0.2;
  params.yaw_moment_coeff = 0.01;
  params.max_total_thrust_n = 40.0;

  const auto thrusts = px4adrc::quad_x_allocate(20.0, Eigen::Vector3d::Zero(), params);

  for (double value : thrusts) {
    EXPECT_NEAR(value, 5.0, 1e-9);
  }
}
