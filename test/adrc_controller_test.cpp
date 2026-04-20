#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "px4adrc/adrc_controller.hpp"

namespace px4adrc {
namespace {

constexpr double kTolerance = 1e-6;

ControllerParams make_base_params() {
  ControllerParams params{};
  params.mass_kg = 2.0;
  params.gravity = 9.81;
  params.max_total_thrust_n = 200.0;
  params.max_tilt_rad = 0.5 * M_PI;

  for (int axis = 0; axis < 3; ++axis) {
    params.position_td_gains[axis].r = 0.0;
    params.position_eso_gains[axis] = EsoGains{0.0, 0.0, 0.0, 1.0};
    params.position_nlsef_gains[axis] = NlsefGains{0.0, 0.0, 1.0, 1.0, 0.01};

    params.attitude_td_gains[axis].r = 0.0;
    params.attitude_eso_gains[axis] = EsoGains{0.0, 0.0, 0.0, 1.0};
    params.attitude_nlsef_gains[axis] = NlsefGains{0.0, 0.0, 1.0, 1.0, 0.01};
  }

  return params;
}

VehicleState make_level_state() {
  VehicleState state{};
  state.q_body_to_ned = Eigen::Quaterniond::Identity();
  state.position_ned.setZero();
  state.velocity_ned.setZero();
  state.body_rates_frd.setZero();
  return state;
}

TrajectoryReference make_zero_reference() {
  TrajectoryReference ref{};
  ref.position_ned.setZero();
  ref.velocity_ned.setZero();
  ref.acceleration_ned.setZero();
  ref.body_rates_frd.setZero();
  ref.body_torque_frd.setZero();
  ref.yaw = 0.0;
  ref.valid = true;
  return ref;
}

PositionControlOutput make_attitude_target(double roll_rad) {
  PositionControlOutput output{};
  output.total_thrust_n = 0.0;
  output.desired_q_body_to_ned = Eigen::Quaterniond(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
  return output;
}

TEST(AdrcControllerTest, AttitudeFeedbackCorrectionRespectsB0Scaling) {
  ControllerParams params_b0_1 = make_base_params();
  params_b0_1.attitude_nlsef_gains[0] = NlsefGains{1.0, 0.0, 1.0, 1.0, 0.01};
  params_b0_1.attitude_eso_gains[0].b0 = 1.0;

  ControllerParams params_b0_2 = params_b0_1;
  params_b0_2.attitude_eso_gains[0].b0 = 2.0;

  const VehicleState state = make_level_state();
  const PositionControlOutput attitude_target = make_attitude_target(0.2);
  const TrajectoryReference ref = make_zero_reference();

  AdrcController controller_b0_1(params_b0_1);
  AdrcController controller_b0_2(params_b0_2);

  const ControlOutput output_b0_1 = controller_b0_1.update_attitude(state, attitude_target, ref, 0.01);
  const ControlOutput output_b0_2 = controller_b0_2.update_attitude(state, attitude_target, ref, 0.01);

  EXPECT_NEAR(output_b0_2.torque_frd.x(), output_b0_1.torque_frd.x() * 0.5, kTolerance);
  EXPECT_NEAR(output_b0_1.torque_frd.y(), 0.0, kTolerance);
  EXPECT_NEAR(output_b0_1.torque_frd.z(), 0.0, kTolerance);
}

TEST(AdrcControllerTest, ResettingParamsRestoresInitialAttitudeResponse) {
  ControllerParams params = make_base_params();
  params.attitude_td_gains[0].r = 3.0;
  params.attitude_eso_gains[0] = EsoGains{8.0, 20.0, 30.0, 1.0};
  params.attitude_nlsef_gains[0] = NlsefGains{2.0, 0.5, 1.0, 1.0, 0.01};

  AdrcController controller(params);
  const VehicleState state = make_level_state();
  const PositionControlOutput attitude_target = make_attitude_target(0.15);
  const TrajectoryReference ref = make_zero_reference();

  const ControlOutput first_output = controller.update_attitude(state, attitude_target, ref, 0.01);
  (void)controller.update_attitude(state, attitude_target, ref, 0.01);

  controller.set_params(params);
  const ControlOutput output_after_reset = controller.update_attitude(state, attitude_target, ref, 0.01);

  EXPECT_NEAR(output_after_reset.torque_frd.x(), first_output.torque_frd.x(), kTolerance);
  EXPECT_NEAR(output_after_reset.torque_frd.y(), first_output.torque_frd.y(), kTolerance);
  EXPECT_NEAR(output_after_reset.torque_frd.z(), first_output.torque_frd.z(), kTolerance);
}

TEST(AdrcControllerTest, PositionTrackingKeepsAccelerationFeedforwardWhenFeedbackIsZero) {
  ControllerParams params = make_base_params();
  params.position_eso_gains[0].b0 = 2.0;
  params.position_eso_gains[1].b0 = 3.0;
  params.position_eso_gains[2].b0 = 4.0;

  AdrcController controller(params);
  VehicleState state = make_level_state();
  TrajectoryReference ref = make_zero_reference();

  state.position_ned = Eigen::Vector3d(1.0, -2.0, -0.5);
  state.velocity_ned = Eigen::Vector3d(0.4, -0.2, 0.1);
  ref.position_ned = state.position_ned;
  ref.velocity_ned = state.velocity_ned;
  ref.acceleration_ned = Eigen::Vector3d(1.2, -0.4, 0.3);

  const PositionControlOutput output = controller.update_position(state, ref, 0.01);

  const Eigen::Vector3d expected_thrust_vector_over_mass =
      Eigen::Vector3d(0.0, 0.0, params.gravity) - ref.acceleration_ned;
  const double expected_total_thrust = params.mass_kg * expected_thrust_vector_over_mass.norm();

  EXPECT_NEAR(output.total_thrust_n, expected_total_thrust, kTolerance);
}

TEST(AdrcControllerTest, PositionObserverFeedbackUsesObservedStateNotRawMeasurement) {
  ControllerParams slow_observer = make_base_params();
  slow_observer.position_td_gains[0].r = 0.0;
  slow_observer.position_eso_gains[0] = EsoGains{0.0, 0.0, 0.0, 1.0};
  slow_observer.position_nlsef_gains[0] = NlsefGains{1.0, 1.0, 1.0, 1.0, 0.01};

  ControllerParams fast_observer = slow_observer;
  fast_observer.position_eso_gains[0] = EsoGains{40.0, 80.0, 0.0, 1.0};

  const VehicleState state = make_level_state();
  TrajectoryReference ref = make_zero_reference();
  ref.position_ned.x() = 1.0;

  AdrcController slow_controller(slow_observer);
  AdrcController fast_controller(fast_observer);

  PositionControlOutput slow_output{};
  PositionControlOutput fast_output{};
  for (int i = 0; i < 6; ++i) {
    slow_output = slow_controller.update_position(state, ref, 0.05);
    fast_output = fast_controller.update_position(state, ref, 0.05);
  }

  EXPECT_GT(std::abs(slow_output.total_thrust_n - fast_output.total_thrust_n), 1.0e-3);
}

TEST(AdrcControllerTest, PositionTrackingUsesDirectVelocityReferenceInFlatnessMode) {
  ControllerParams params = make_base_params();
  params.position_td_gains[0].r = 4.0;
  params.position_eso_gains[0] = EsoGains{0.0, 0.0, 0.0, 1.0};
  params.position_nlsef_gains[0] = NlsefGains{0.0, 1.0, 1.0, 1.0, 0.01};

  AdrcController controller(params);
  const VehicleState state = make_level_state();

  TrajectoryReference initial_ref = make_zero_reference();
  (void)controller.update_position(state, initial_ref, 0.05);

  TrajectoryReference moving_ref = make_zero_reference();
  moving_ref.position_ned.x() = 1.0;
  moving_ref.velocity_ned.x() = 1.0;

  const PositionControlOutput output = controller.update_position(state, moving_ref, 0.05);

  const double expected_accel_x = 1.0;
  const double expected_total_thrust =
      params.mass_kg * std::sqrt(params.gravity * params.gravity + expected_accel_x * expected_accel_x);

  EXPECT_NEAR(output.total_thrust_n, expected_total_thrust, kTolerance);
}

TEST(AdrcControllerTest, PositionTrackingIgnoresTdGainInFlatnessMode) {
  ControllerParams params_without_td = make_base_params();
  params_without_td.position_td_gains[0].r = 0.0;
  params_without_td.position_eso_gains[0] = EsoGains{0.0, 0.0, 0.0, 1.0};
  params_without_td.position_nlsef_gains[0] = NlsefGains{0.0, 1.0, 1.0, 1.0, 0.01};

  ControllerParams params_with_td = params_without_td;
  params_with_td.position_td_gains[0].r = 4.0;

  const VehicleState state = make_level_state();
  TrajectoryReference initial_ref = make_zero_reference();
  TrajectoryReference moving_ref = make_zero_reference();
  moving_ref.position_ned.x() = 1.0;
  moving_ref.velocity_ned.x() = 1.0;

  AdrcController controller_without_td(params_without_td);
  AdrcController controller_with_td(params_with_td);

  (void)controller_without_td.update_position(state, initial_ref, 0.05);
  (void)controller_with_td.update_position(state, initial_ref, 0.05);

  const PositionControlOutput output_without_td = controller_without_td.update_position(state, moving_ref, 0.05);
  const PositionControlOutput output_with_td = controller_with_td.update_position(state, moving_ref, 0.05);

  EXPECT_NEAR(output_with_td.total_thrust_n, output_without_td.total_thrust_n, kTolerance);
}

TEST(AdrcControllerTest, AttitudeTrackingKeepsTorqueFeedforwardWhenFeedbackIsZero) {
  ControllerParams params = make_base_params();
  params.attitude_eso_gains[0].b0 = 2.0;
  params.attitude_eso_gains[1].b0 = 3.0;
  params.attitude_eso_gains[2].b0 = 4.0;

  AdrcController controller(params);
  const VehicleState state = make_level_state();
  PositionControlOutput attitude_target{};
  attitude_target.total_thrust_n = 0.0;
  attitude_target.desired_q_body_to_ned = state.q_body_to_ned;

  TrajectoryReference ref = make_zero_reference();
  ref.body_torque_frd = Eigen::Vector3d(0.3, -0.2, 0.1);

  const ControlOutput output = controller.update_attitude(state, attitude_target, ref, 0.01);

  EXPECT_NEAR(output.torque_frd.x(), ref.body_torque_frd.x(), kTolerance);
  EXPECT_NEAR(output.torque_frd.y(), ref.body_torque_frd.y(), kTolerance);
  EXPECT_NEAR(output.torque_frd.z(), ref.body_torque_frd.z(), kTolerance);
}

TEST(AdrcControllerTest, AttitudeObserverFeedbackUsesObservedStateNotRawMeasurement) {
  ControllerParams slow_observer = make_base_params();
  slow_observer.attitude_td_gains[0].r = 0.0;
  slow_observer.attitude_eso_gains[0] = EsoGains{0.0, 0.0, 0.0, 1.0};
  slow_observer.attitude_nlsef_gains[0] = NlsefGains{1.0, 1.0, 1.0, 1.0, 0.01};

  ControllerParams fast_observer = slow_observer;
  fast_observer.attitude_eso_gains[0] = EsoGains{40.0, 80.0, 0.0, 1.0};

  const VehicleState state = make_level_state();
  const PositionControlOutput attitude_target = make_attitude_target(0.2);
  const TrajectoryReference ref = make_zero_reference();

  AdrcController slow_controller(slow_observer);
  AdrcController fast_controller(fast_observer);

  ControlOutput slow_output{};
  ControlOutput fast_output{};
  for (int i = 0; i < 6; ++i) {
    slow_output = slow_controller.update_attitude(state, attitude_target, ref, 0.05);
    fast_output = fast_controller.update_attitude(state, attitude_target, ref, 0.05);
  }

  EXPECT_GT(std::abs(slow_output.torque_frd.x() - fast_output.torque_frd.x()), 1.0e-3);
}

} // namespace
} // namespace px4adrc
