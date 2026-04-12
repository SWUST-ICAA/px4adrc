#include "px4adrc/adrc_controller.hpp"

#include "px4adrc/common.hpp"

#include <algorithm>

namespace px4adrc
{

namespace
{

void apply_td_gains(
  std::array<TrackingDifferentiator, 3> & td_states,
  const std::array<TrackingDifferentiatorGains, 3> & gains)
{
  for (int i = 0; i < 3; ++i) {
    td_states[i].r = std::max(0.0, gains[i].r);
  }
}

void apply_eso_gains(
  std::array<ExtendedStateObserver, 3> & eso_states,
  const std::array<EsoGains, 3> & gains)
{
  for (int i = 0; i < 3; ++i) {
    eso_states[i].beta1 = std::max(0.0, gains[i].beta1);
    eso_states[i].beta2 = std::max(0.0, gains[i].beta2);
    eso_states[i].beta3 = std::max(0.0, gains[i].beta3);
    eso_states[i].b0 = gains[i].b0;
  }
}

Eigen::Vector3d update_position_channel(
  const VehicleState & state,
  const TrajectoryReference & ref,
  std::array<TrackingDifferentiator, 3> & pos_td,
  std::array<ExtendedStateObserver, 3> & pos_eso,
  const ControllerParams & params,
  double dt)
{
  Eigen::Vector3d accel_cmd = ref.acceleration_ned;

  for (int i = 0; i < 3; ++i) {
    pos_td[i].h = dt;
    pos_eso[i].h = dt;

    update_tracking_differentiator(ref.position_ned[i], pos_td[i]);
    update_eso(state.position_ned[i], 0.0, pos_eso[i]);

    const double e1 = pos_td[i].x1 - state.position_ned[i];
    const double e2 =
      pos_td[i].x2 + sanitize_scalar(ref.velocity_ned[i]) - state.velocity_ned[i];
    accel_cmd[i] += compute_nlsef(e1, e2, params.position_nlsef_gains[i]) - pos_eso[i].z3;
  }

  return accel_cmd;
}

Eigen::Vector3d update_rate_channel(
  const VehicleState & state,
  const Eigen::Vector3d & desired_body_rates_frd,
  const Eigen::Vector3d & torque_feedforward_frd,
  std::array<TrackingDifferentiator, 3> & rate_td,
  std::array<ExtendedStateObserver, 3> & rate_eso,
  const ControllerParams & params,
  double dt)
{
  Eigen::Vector3d torque_cmd = torque_feedforward_frd;

  for (int i = 0; i < 3; ++i) {
    rate_td[i].h = dt;
    rate_eso[i].h = dt;

    update_tracking_differentiator(desired_body_rates_frd[i], rate_td[i]);
    update_eso(state.body_rates_frd[i], 0.0, rate_eso[i]);

    const double e1 = rate_td[i].x1 - state.body_rates_frd[i];
    const double e2 = rate_td[i].x2 - state.body_accel_frd[i];
    torque_cmd[i] += compute_nlsef(e1, e2, params.rate_nlsef_gains[i]) - rate_eso[i].z3;
  }

  return torque_cmd;
}

}  // namespace

AdrcController::AdrcController(const ControllerParams & params)
{
  set_params(params);
}

void AdrcController::set_params(const ControllerParams & params)
{
  params_ = params;
  apply_td_gains(pos_td_, params_.position_td_gains);
  apply_eso_gains(pos_eso_, params_.position_eso_gains);
  apply_td_gains(rate_td_, params_.rate_td_gains);
  apply_eso_gains(rate_eso_, params_.rate_eso_gains);
}

const ControllerParams & AdrcController::params() const
{
  return params_;
}

ControlOutput AdrcController::update(
  const VehicleState & state,
  const TrajectoryReference & ref,
  double dt)
{
  const double clamped_dt = std::clamp(dt, 1e-4, 0.05);
  const Eigen::Vector3d accel_cmd_ned =
    update_position_channel(state, ref, pos_td_, pos_eso_, params_, clamped_dt);

  Eigen::Vector3d thrust_vector_over_mass_ned(0.0, 0.0, params_.gravity);
  thrust_vector_over_mass_ned -= sanitize_vec3(accel_cmd_ned);
  apply_tilt_limit(&thrust_vector_over_mass_ned, params_.max_tilt_rad);

  const double thrust_vector_norm = thrust_vector_over_mass_ned.norm();
  const double total_thrust_n = clamp_scalar(
    params_.mass_kg * thrust_vector_norm,
    0.0,
    params_.max_total_thrust_n);

  Eigen::Vector3d thrust_direction_ned = Eigen::Vector3d::UnitZ();
  if (thrust_vector_norm > 1e-9) {
    thrust_direction_ned = thrust_vector_over_mass_ned / thrust_vector_norm;
  }

  const Eigen::Quaterniond desired_q_body_to_ned = quaternion_from_thrust_direction_yaw_ned(
    thrust_direction_ned, sanitize_scalar(ref.yaw));
  Eigen::Quaterniond q_error =
    desired_q_body_to_ned.conjugate() * align_quaternion_sign(
    state.q_body_to_ned, desired_q_body_to_ned);
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }

  const Eigen::Vector3d desired_body_rates_frd =
    sanitize_vec3(ref.body_rates_frd) - params_.attitude_kp.cwiseProduct(q_error.vec());
  const Eigen::Vector3d torque_cmd = update_rate_channel(
    state,
    desired_body_rates_frd,
    sanitize_vec3(ref.body_torque_frd),
    rate_td_,
    rate_eso_,
    params_,
    clamped_dt);

  ControlOutput output{};
  output.total_thrust_n = total_thrust_n;
  output.torque_frd = torque_cmd;
  output.motor_thrusts_n = quad_x_allocate(total_thrust_n, torque_cmd, params_);
  output.motor_throttles = motor_thrusts_to_throttles(output.motor_thrusts_n, params_);
  return output;
}

}  // namespace px4adrc
