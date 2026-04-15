#include <gtest/gtest.h>

#include "px4adrc/common.hpp"
#include "px4adrc/msg/flat_trajectory_reference.hpp"
#include "px4adrc/types.hpp"
#include <array>
#include <cmath>

TEST(FlatTrajectoryReferenceMessage, DefaultConstructs) {
  px4adrc::msg::FlatTrajectoryReference msg{};
  EXPECT_FLOAT_EQ(msg.yaw, 0.0F);
}

TEST(CommonMath, ClampScalarClampsToBounds) {
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(-1.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(0.4, 0.0, 1.0), 0.4);
  EXPECT_DOUBLE_EQ(px4adrc::clamp_scalar(2.0, 0.0, 1.0), 1.0);
}

TEST(CommonMath, ThrustModelFactorOneMapsBySquareRoot) {
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

TEST(CommonMath, CollectiveThrustNormalizesByMaxTotal) {
  px4adrc::ControllerParams params{};
  params.max_total_thrust_n = 40.0;

  const std::array<double, px4adrc::kMotorCount> thrusts{{5.0, 5.0, 5.0, 5.0}};
  EXPECT_DOUBLE_EQ(px4adrc::collective_thrust_normalized_from_motor_thrusts(thrusts, params), 0.5);
}

TEST(CommonMath, TiltLimitClampsHorizontalThrust) {
  Eigen::Vector3d thrust_vector_over_mass_ned(2.0, 0.0, 1.0);
  px4adrc::apply_tilt_limit(&thrust_vector_over_mass_ned, M_PI / 4.0);

  EXPECT_NEAR(thrust_vector_over_mass_ned.x(), 1.0, 1e-12);
  EXPECT_NEAR(thrust_vector_over_mass_ned.y(), 0.0, 1e-12);
  EXPECT_NEAR(thrust_vector_over_mass_ned.z(), 1.0, 1e-12);
}

TEST(CommonMath, AllocationProducesHoverSymmetry) {
  px4adrc::ControllerParams params{};
  params.arm_length_m = 0.2;
  params.yaw_moment_coeff = 0.01;
  params.max_total_thrust_n = 40.0;

  const auto thrusts = px4adrc::quad_x_allocate(20.0, Eigen::Vector3d::Zero(), params);

  for (double value : thrusts) {
    EXPECT_NEAR(value, 5.0, 1e-9);
  }
}
