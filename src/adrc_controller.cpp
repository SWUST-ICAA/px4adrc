#include "px4adrc/adrc_controller.hpp"

#include "px4adrc/common.hpp"

#include <algorithm>
#include <cmath>

namespace px4adrc {

namespace {

void apply_eso_gains(std::array<ExtendedStateObserver, 3> &eso_states, const std::array<EsoGains, 3> &gains,
                     const std::array<double, 3> &fallback_b0) {
  for (int i = 0; i < 3; ++i) {
    eso_states[i].beta1 = std::max(0.0, gains[i].beta1);
    eso_states[i].beta2 = std::max(0.0, gains[i].beta2);
    eso_states[i].beta3 = std::max(0.0, gains[i].beta3);
    eso_states[i].b0 = std::abs(gains[i].b0) > 1e-6 ? gains[i].b0 : fallback_b0[i];
  }
}

std::array<double, 3> position_input_gains(const ControllerParams &params) {
  std::array<double, 3> gains{};
  for (int i = 0; i < 3; ++i) {
    gains[i] = 1.0;
    if (std::abs(params.position_eso_gains[i].b0) > 1e-6) {
      gains[i] = params.position_eso_gains[i].b0;
    }
  }
  return gains;
}

std::array<double, 3> attitude_input_gains(const ControllerParams &params) {
  std::array<double, 3> gains{};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(params.attitude_eso_gains[i].b0) > 1e-6) {
      gains[i] = params.attitude_eso_gains[i].b0;
      continue;
    }

    gains[i] = 1.0 / std::max(std::abs(params.inertia_diag[i]), 1e-6);
  }
  return gains;
}

void initialize_position_states(const VehicleState &state, const TrajectoryReference &ref,
                                std::array<ExtendedStateObserver, 3> &pos_eso,
                                std::array<double, 3> &last_accel_cmd_ned) {
  for (int i = 0; i < 3; ++i) {
    pos_eso[i].z1 = state.position_ned[i];
    pos_eso[i].z2 = state.velocity_ned[i];
    pos_eso[i].z3 = 0.0;
    last_accel_cmd_ned[i] = ref.acceleration_ned[i];
  }
}

void initialize_attitude_states(const Eigen::Vector3d &attitude_error_desired_frd,
                                const Eigen::Vector3d &desired_body_rates_frd,
                                const Eigen::Vector3d &current_body_rates_desired_frd,
                                const Eigen::Vector3d &torque_feedforward_frd,
                                std::array<ExtendedStateObserver, 3> &attitude_eso,
                                std::array<double, 3> &last_torque_cmd_desired_frd) {
  for (int i = 0; i < 3; ++i) {
    attitude_eso[i].z1 = attitude_error_desired_frd[i];
    attitude_eso[i].z2 = desired_body_rates_frd[i] - current_body_rates_desired_frd[i];
    attitude_eso[i].z3 = 0.0;
    last_torque_cmd_desired_frd[i] = torque_feedforward_frd[i];
  }
}

Eigen::Vector3d update_position_channel(const VehicleState &state, const TrajectoryReference &ref,
                                        std::array<ExtendedStateObserver, 3> &pos_eso,
                                        std::array<double, 3> &last_accel_cmd_ned, const ControllerParams &params, double dt) {
  Eigen::Vector3d accel_cmd = ref.acceleration_ned;

  for (int i = 0; i < 3; ++i) {
    pos_eso[i].h = dt;

    update_eso(state.position_ned[i], last_accel_cmd_ned[i], pos_eso[i]);

    const double e1 = ref.position_ned[i] - pos_eso[i].z1;
    const double e2 = ref.velocity_ned[i] - pos_eso[i].z2;
    const double feedback_correction = compute_nlsef(e1, e2, params.position_nlsef_gains[i]) - pos_eso[i].z3;
    accel_cmd[i] += feedback_correction / pos_eso[i].b0;
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
                                        std::array<ExtendedStateObserver, 3> &attitude_eso,
                                        std::array<double, 3> &last_torque_cmd_desired_frd, bool *states_initialized,
                                        const ControllerParams &params, double dt) {
  const Eigen::Quaterniond q_current_to_desired = current_to_desired_body_quaternion(state.q_body_to_ned, desired_q_body_to_ned);
  const Eigen::Vector3d attitude_error_desired_frd = 2.0 * q_current_to_desired.vec();
  const Eigen::Vector3d current_body_rates_desired_frd = q_current_to_desired * state.body_rates_frd;

  if (states_initialized != nullptr && !*states_initialized) {
    initialize_attitude_states(attitude_error_desired_frd, desired_body_rates_frd, current_body_rates_desired_frd,
                               torque_feedforward_frd, attitude_eso, last_torque_cmd_desired_frd);
    *states_initialized = true;
  }

  Eigen::Vector3d torque_cmd_desired_frd = torque_feedforward_frd;

  for (int i = 0; i < 3; ++i) {
    attitude_eso[i].h = dt;

    update_eso(attitude_error_desired_frd[i], last_torque_cmd_desired_frd[i], attitude_eso[i]);

    const double e1 = -attitude_eso[i].z1;
    const double e2 = -attitude_eso[i].z2;
    const double feedback_correction = compute_nlsef(e1, e2, params.attitude_nlsef_gains[i]) - attitude_eso[i].z3;
    torque_cmd_desired_frd[i] += feedback_correction / attitude_eso[i].b0;
    last_torque_cmd_desired_frd[i] = torque_cmd_desired_frd[i];
  }

  return q_current_to_desired.conjugate() * torque_cmd_desired_frd;
}

} // namespace

AdrcController::AdrcController(const ControllerParams &params) { set_params(params); }

void AdrcController::set_params(const ControllerParams &params) {
  params_ = params;
  pos_eso_ = {};
  attitude_eso_ = {};
  apply_eso_gains(pos_eso_, params_.position_eso_gains, position_input_gains(params_));
  apply_eso_gains(attitude_eso_, params_.attitude_eso_gains, attitude_input_gains(params_));
  last_position_accel_cmd_ned_.fill(0.0);
  last_attitude_torque_cmd_desired_frd_.fill(0.0);
  position_states_initialized_ = false;
  attitude_states_initialized_ = false;
}

const ControllerParams &AdrcController::params() const { return params_; }

PositionControlOutput AdrcController::update_position(const VehicleState &state, const TrajectoryReference &ref, double dt) {
  const double clamped_dt = std::clamp(dt, 1e-4, 0.05);
  if (!position_states_initialized_) {
    initialize_position_states(state, ref, pos_eso_, last_position_accel_cmd_ned_);
    position_states_initialized_ = true;
  }

  const Eigen::Vector3d accel_cmd_ned = update_position_channel(state, ref, pos_eso_, last_position_accel_cmd_ned_, params_, clamped_dt);

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
                                                             ref.body_torque_frd, attitude_eso_, last_attitude_torque_cmd_desired_frd_,
                                                             &attitude_states_initialized_, params_, clamped_dt);

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
