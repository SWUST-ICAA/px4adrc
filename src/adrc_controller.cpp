#include "px4adrc/adrc_controller.hpp"

#include "px4adrc/common.hpp"

#include <algorithm>

namespace px4adrc {

namespace {

void apply_td_gains(std::array<TrackingDifferentiator, 3> &td_states, const std::array<TrackingDifferentiatorGains, 3> &gains) {
  for (int i = 0; i < 3; ++i) {
    td_states[i].r = std::max(0.0, gains[i].r);
  }
}

void apply_eso_gains(std::array<ExtendedStateObserver, 3> &eso_states, const std::array<EsoGains, 3> &gains) {
  for (int i = 0; i < 3; ++i) {
    eso_states[i].beta1 = std::max(0.0, gains[i].beta1);
    eso_states[i].beta2 = std::max(0.0, gains[i].beta2);
    eso_states[i].beta3 = std::max(0.0, gains[i].beta3);
    eso_states[i].b0 = gains[i].b0;
  }
}

Eigen::Vector3d update_position_channel(const VehicleState &state, const TrajectoryReference &ref,
                                        std::array<TrackingDifferentiator, 3> &pos_td, std::array<ExtendedStateObserver, 3> &pos_eso,
                                        std::array<double, 3> &last_accel_cmd_ned, const ControllerParams &params, double dt) {
  Eigen::Vector3d accel_cmd = ref.acceleration_ned;

  for (int i = 0; i < 3; ++i) {
    pos_td[i].h = dt;
    pos_eso[i].h = dt;

    update_tracking_differentiator(ref.position_ned[i], pos_td[i]);
    update_eso(state.position_ned[i], last_accel_cmd_ned[i], pos_eso[i]);

    const double e1 = pos_td[i].x1 - state.position_ned[i];
    const double e2 = pos_td[i].x2 + ref.velocity_ned[i] - state.velocity_ned[i];
    accel_cmd[i] += compute_nlsef(e1, e2, params.position_nlsef_gains[i]) - pos_eso[i].z3;
    last_accel_cmd_ned[i] = accel_cmd[i];
  }

  return accel_cmd;
}

Eigen::Quaterniond current_to_desired_body_quaternion(const Eigen::Quaterniond &current_q_body_to_ned,
                                                      const Eigen::Quaterniond &desired_q_body_to_ned) {
  Eigen::Quaterniond q_current_to_desired = desired_q_body_to_ned.conjugate() *
                                            align_quaternion_sign(current_q_body_to_ned, desired_q_body_to_ned);
  q_current_to_desired.normalize();
  if (q_current_to_desired.w() < 0.0) {
    q_current_to_desired.coeffs() *= -1.0;
  }
  return q_current_to_desired;
}

Eigen::Vector3d update_attitude_channel(const VehicleState &state, const Eigen::Quaterniond &desired_q_body_to_ned,
                                        const Eigen::Vector3d &desired_body_rates_frd, const Eigen::Vector3d &torque_feedforward_frd,
                                        std::array<TrackingDifferentiator, 3> &attitude_td,
                                        std::array<ExtendedStateObserver, 3> &attitude_eso,
                                        std::array<double, 3> &last_torque_cmd_desired_frd, const ControllerParams &params, double dt) {
  const Eigen::Quaterniond q_current_to_desired = current_to_desired_body_quaternion(state.q_body_to_ned, desired_q_body_to_ned);
  const Eigen::Vector3d attitude_error_desired_frd = 2.0 * q_current_to_desired.vec();
  const Eigen::Vector3d current_body_rates_desired_frd = q_current_to_desired * state.body_rates_frd;

  Eigen::Vector3d torque_cmd_desired_frd = torque_feedforward_frd;

  for (int i = 0; i < 3; ++i) {
    attitude_td[i].h = dt;
    attitude_eso[i].h = dt;

    update_tracking_differentiator(0.0, attitude_td[i]);
    update_eso(attitude_error_desired_frd[i], last_torque_cmd_desired_frd[i], attitude_eso[i]);

    const double e1 = attitude_td[i].x1 - attitude_error_desired_frd[i];
    const double e2 = attitude_td[i].x2 + desired_body_rates_frd[i] - current_body_rates_desired_frd[i];
    torque_cmd_desired_frd[i] += compute_nlsef(e1, e2, params.attitude_nlsef_gains[i]) - attitude_eso[i].z3;
    last_torque_cmd_desired_frd[i] = torque_cmd_desired_frd[i];
  }

  return q_current_to_desired.conjugate() * torque_cmd_desired_frd;
}

} // namespace

AdrcController::AdrcController(const ControllerParams &params) { set_params(params); }

void AdrcController::set_params(const ControllerParams &params) {
  params_ = params;
  apply_td_gains(pos_td_, params_.position_td_gains);
  apply_eso_gains(pos_eso_, params_.position_eso_gains);
  apply_td_gains(attitude_td_, params_.attitude_td_gains);
  apply_eso_gains(attitude_eso_, params_.attitude_eso_gains);
  last_position_accel_cmd_ned_.fill(0.0);
  last_attitude_torque_cmd_desired_frd_.fill(0.0);
}

const ControllerParams &AdrcController::params() const { return params_; }

PositionControlOutput AdrcController::update_position(const VehicleState &state, const TrajectoryReference &ref, double dt) {
  const double clamped_dt = std::clamp(dt, 1e-4, 0.05);
  const Eigen::Vector3d accel_cmd_ned = update_position_channel(state, ref, pos_td_, pos_eso_, last_position_accel_cmd_ned_, params_,
                                                                clamped_dt);

  Eigen::Vector3d thrust_vector_over_mass_ned(0.0, 0.0, params_.gravity);
  thrust_vector_over_mass_ned -= accel_cmd_ned;
  apply_tilt_limit(&thrust_vector_over_mass_ned, params_.max_tilt_rad);

  const double thrust_vector_norm = thrust_vector_over_mass_ned.norm();
  Eigen::Vector3d thrust_direction_ned = Eigen::Vector3d::UnitZ();
  if (thrust_vector_norm > 1e-9) {
    thrust_direction_ned = thrust_vector_over_mass_ned / thrust_vector_norm;
  }

  PositionControlOutput output{};
  output.total_thrust_n = clamp_scalar(params_.mass_kg * thrust_vector_norm, 0.0, params_.max_total_thrust_n);
  output.desired_q_body_to_ned = quaternion_from_thrust_direction_yaw_ned(thrust_direction_ned, ref.yaw);
  return output;
}

ControlOutput AdrcController::update_attitude(const VehicleState &state, const PositionControlOutput &position_output,
                                              const TrajectoryReference &ref, double dt) {
  const double clamped_dt = std::clamp(dt, 1e-4, 0.05);
  const Eigen::Vector3d torque_cmd = update_attitude_channel(state, position_output.desired_q_body_to_ned, ref.body_rates_frd,
                                                             ref.body_torque_frd, attitude_td_, attitude_eso_,
                                                             last_attitude_torque_cmd_desired_frd_, params_, clamped_dt);

  ControlOutput output{};
  output.total_thrust_n = position_output.total_thrust_n;
  output.torque_frd = torque_cmd;
  output.motor_thrusts_n = quad_x_allocate(output.total_thrust_n, output.torque_frd, params_);
  output.motor_throttles = motor_thrusts_to_throttles(output.motor_thrusts_n, params_);
  return output;
}

ControlOutput AdrcController::update(const VehicleState &state, const TrajectoryReference &ref, double dt) {
  const PositionControlOutput position_output = update_position(state, ref, dt);
  return update_attitude(state, position_output, ref, dt);
}

} // namespace px4adrc
