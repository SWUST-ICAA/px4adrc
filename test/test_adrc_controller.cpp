#include <gtest/gtest.h>

#include <cmath>

#include "px4adrc/adrc_controller.hpp"

namespace {

px4adrc::ControlOutput run_controller_steps(
    px4adrc::AdrcController &controller, const px4adrc::VehicleState &state,
    const px4adrc::TrajectoryReference &ref, int steps, double dt) {
  px4adrc::ControlOutput output{};
  for (int i = 0; i < steps; ++i) {
    output = controller.update(state, ref, dt);
  }
  return output;
}

} // namespace

TEST(AdrcController, HoverReferenceProducesFiniteMotorOutputs) {
  px4adrc::ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 40.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};
  px4adrc::TrajectoryReference ref{};
  ref.position_ned = Eigen::Vector3d::Zero();
  ref.velocity_ned = Eigen::Vector3d::Zero();
  ref.acceleration_ned = Eigen::Vector3d::Zero();
  ref.yaw = 0.0;
  ref.valid = true;

  const auto output = controller.update(state, ref, 0.01);

  EXPECT_GT(output.total_thrust_n, 0.0);
  for (double value : output.motor_thrusts_n) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_GE(value, 0.0);
  }
}

TEST(AdrcController, ZeroReferenceGivesSymmetricHoverAtIdentity) {
  px4adrc::ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 40.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};
  px4adrc::TrajectoryReference ref{};
  ref.valid = true;

  const auto output = controller.update(state, ref, 0.01);

  EXPECT_NEAR(output.motor_thrusts_n[0], output.motor_thrusts_n[1], 1e-6);
  EXPECT_NEAR(output.motor_thrusts_n[1], output.motor_thrusts_n[2], 1e-6);
  EXPECT_NEAR(output.motor_thrusts_n[2], output.motor_thrusts_n[3], 1e-6);
}

TEST(AdrcController, PositiveAltitudeErrorIncreasesCollectiveThrust) {
  px4adrc::ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 60.0;
  for (auto &gains : params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  params.position_nlsef_gains[2].k1 = 2.0;
  params.position_nlsef_gains[2].alpha1 = 1.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};
  state.position_ned.z() = 0.0;

  px4adrc::TrajectoryReference ref{};
  ref.position_ned.z() = -1.0;
  ref.valid = true;

  const auto output = run_controller_steps(controller, state, ref, 500, 0.01);
  EXPECT_GT(output.total_thrust_n, params.mass_kg * params.gravity);
}

TEST(AdrcController, AttitudeNlsefGeneratesCorrectiveTorqueFromAttitudeError) {
  px4adrc::ControllerParams params{};
  params.max_total_thrust_n = 60.0;
  for (auto &gains : params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  for (auto &gains : params.attitude_eso_gains) {
    gains.beta1 = 0.0;
    gains.beta2 = 0.0;
    gains.beta3 = 0.0;
    gains.b0 = 0.0;
  }
  for (auto &gains : params.attitude_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  params.attitude_nlsef_gains[0].k1 = 0.8;
  params.attitude_nlsef_gains[0].alpha1 = 1.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};
  state.q_body_to_ned =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()));

  px4adrc::TrajectoryReference ref{};
  ref.valid = true;

  const auto output = run_controller_steps(controller, state, ref, 500, 0.01);

  EXPECT_LT(output.torque_frd.x(), 0.0);
  EXPECT_NEAR(output.torque_frd.y(), 0.0, 1e-6);
  EXPECT_NEAR(output.torque_frd.z(), 0.0, 1e-6);
}

TEST(AdrcController,
     AttitudeRateFeedforwardGeneratesTorqueFromAngularRateError) {
  px4adrc::ControllerParams params{};
  params.max_total_thrust_n = 60.0;
  for (auto &gains : params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  for (auto &gains : params.attitude_eso_gains) {
    gains.beta1 = 0.0;
    gains.beta2 = 0.0;
    gains.beta3 = 0.0;
    gains.b0 = 0.0;
  }
  for (auto &gains : params.attitude_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  params.attitude_td_gains[0].r = 20.0;
  params.attitude_nlsef_gains[0].k2 = 0.8;
  params.attitude_nlsef_gains[0].alpha2 = 1.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};

  px4adrc::TrajectoryReference ref{};
  ref.body_rates_frd.x() = 1.0;
  ref.valid = true;

  const auto output = run_controller_steps(controller, state, ref, 500, 0.01);

  EXPECT_GT(output.torque_frd.x(), 0.0);
  EXPECT_NEAR(output.torque_frd.y(), 0.0, 1e-6);
  EXPECT_NEAR(output.torque_frd.z(), 0.0, 1e-6);
}

TEST(AdrcController, PositionTdBandwidthControlsAltitudeResponse) {
  px4adrc::ControllerParams frozen_params{};
  frozen_params.mass_kg = 2.0;
  frozen_params.gravity = 9.81;
  frozen_params.max_total_thrust_n = 60.0;
  for (auto &gains : frozen_params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  frozen_params.position_nlsef_gains[2].k1 = 2.0;
  frozen_params.position_nlsef_gains[2].alpha1 = 1.0;
  frozen_params.position_td_gains[2].r = 0.0;
  px4adrc::AdrcController frozen_controller(frozen_params);

  px4adrc::ControllerParams responsive_params = frozen_params;
  responsive_params.position_td_gains[2].r = 20.0;
  px4adrc::AdrcController responsive_controller(responsive_params);

  px4adrc::VehicleState state{};

  px4adrc::TrajectoryReference ref{};
  ref.position_ned.z() = -1.0;
  ref.valid = true;

  const auto frozen_output =
      run_controller_steps(frozen_controller, state, ref, 500, 0.01);
  const auto responsive_output =
      run_controller_steps(responsive_controller, state, ref, 500, 0.01);

  EXPECT_NEAR(frozen_output.total_thrust_n,
              frozen_params.mass_kg * frozen_params.gravity, 1e-6);
  EXPECT_GT(responsive_output.total_thrust_n,
            responsive_params.mass_kg * responsive_params.gravity);
}

TEST(AdrcController, PositionEsoGainsAffectAltitudeResponse) {
  px4adrc::ControllerParams passive_params{};
  passive_params.mass_kg = 2.0;
  passive_params.gravity = 9.81;
  passive_params.max_total_thrust_n = 60.0;
  for (auto &gains : passive_params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  passive_params.position_td_gains[2].r = 0.0;
  passive_params.position_eso_gains[2].beta1 = 0.0;
  passive_params.position_eso_gains[2].beta2 = 0.0;
  passive_params.position_eso_gains[2].beta3 = 0.0;
  passive_params.position_eso_gains[2].b0 = 0.0;
  px4adrc::AdrcController passive_controller(passive_params);

  px4adrc::ControllerParams observing_params = passive_params;
  observing_params.position_eso_gains[2].beta1 = 80.0;
  observing_params.position_eso_gains[2].beta2 = 400.0;
  observing_params.position_eso_gains[2].beta3 = 500.0;
  observing_params.position_eso_gains[2].b0 = 1.0;
  px4adrc::AdrcController observing_controller(observing_params);

  px4adrc::VehicleState state{};
  state.position_ned.z() = 1.0;

  px4adrc::TrajectoryReference ref{};
  ref.valid = true;

  const auto passive_output =
      run_controller_steps(passive_controller, state, ref, 50, 0.01);
  const auto observing_output =
      run_controller_steps(observing_controller, state, ref, 50, 0.01);

  EXPECT_NEAR(passive_output.total_thrust_n,
              passive_params.mass_kg * passive_params.gravity, 1e-6);
  EXPECT_LT(observing_output.total_thrust_n, passive_output.total_thrust_n);
}

TEST(AdrcController, VelocityFeedforwardAffectsAltitudeResponse) {
  px4adrc::ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 60.0;
  params.position_td_gains[2].r = 0.0;
  for (auto &gains : params.position_nlsef_gains) {
    gains.k1 = 0.0;
    gains.k2 = 0.0;
  }
  params.position_nlsef_gains[2].k2 = 1.5;
  params.position_nlsef_gains[2].alpha2 = 1.0;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};

  px4adrc::TrajectoryReference ref{};
  ref.velocity_ned.z() = -1.0;
  ref.valid = true;

  const auto output = run_controller_steps(controller, state, ref, 5, 0.01);

  EXPECT_GT(output.total_thrust_n, params.mass_kg * params.gravity);
}

TEST(AdrcController, HorizontalPositionErrorProducesDifferentialMotorThrusts) {
  px4adrc::ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 60.0;
  params.arm_length_m = 0.246073;
  params.yaw_moment_coeff = 0.016;
  px4adrc::AdrcController controller(params);

  px4adrc::VehicleState state{};

  px4adrc::TrajectoryReference ref{};
  ref.position_ned.x() = 1.0;
  ref.valid = true;

  const auto output = controller.update(state, ref, 0.01);

  const bool all_equal =
      std::abs(output.motor_thrusts_n[0] - output.motor_thrusts_n[1]) < 1e-6 &&
      std::abs(output.motor_thrusts_n[1] - output.motor_thrusts_n[2]) < 1e-6 &&
      std::abs(output.motor_thrusts_n[2] - output.motor_thrusts_n[3]) < 1e-6;
  EXPECT_FALSE(all_equal);
}
