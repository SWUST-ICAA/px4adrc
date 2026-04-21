#pragma once

#include "px4adrc/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace px4adrc {

struct WrenchFrd {
  double total_thrust_n{0.0};
  Eigen::Vector3d torque_frd{Eigen::Vector3d::Zero()};
};

inline double clamp_scalar(double value, double lower, double upper) { return std::min(std::max(value, lower), upper); }

inline void apply_tilt_limit(Eigen::Vector3d *thrust_vector_over_mass_ned, double max_tilt_rad) {
  if (thrust_vector_over_mass_ned == nullptr || max_tilt_rad <= 0.0) {
    return;
  }

  thrust_vector_over_mass_ned->z() = std::max(thrust_vector_over_mass_ned->z(), 1e-3);

  const double horizontal_norm = std::hypot(thrust_vector_over_mass_ned->x(), thrust_vector_over_mass_ned->y());
  const double max_horizontal = thrust_vector_over_mass_ned->z() * std::tan(max_tilt_rad);

  if (horizontal_norm <= max_horizontal || horizontal_norm <= 1e-9) {
    return;
  }

  const double scale = max_horizontal / horizontal_norm;
  thrust_vector_over_mass_ned->x() *= scale;
  thrust_vector_over_mass_ned->y() *= scale;
}

inline Eigen::Quaterniond align_quaternion_sign(const Eigen::Quaterniond &quaternion, const Eigen::Quaterniond &anchor) {
  Eigen::Quaterniond aligned = quaternion.normalized();
  const Eigen::Quaterniond normalized_anchor = anchor.normalized();
  if (aligned.coeffs().dot(normalized_anchor.coeffs()) < 0.0) {
    aligned.coeffs() *= -1.0;
  }
  return aligned;
}

inline Eigen::Quaterniond quaternion_from_thrust_direction_yaw_ned(const Eigen::Vector3d &thrust_direction_ned, double yaw) {
  Eigen::Vector3d z_body_ned = thrust_direction_ned;
  if (z_body_ned.norm() <= 1e-9) {
    z_body_ned = Eigen::Vector3d::UnitZ();
  } else {
    z_body_ned.normalize();
  }

  Eigen::Vector3d x_course_ned(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d y_body_ned = z_body_ned.cross(x_course_ned);
  if (y_body_ned.norm() <= 1e-9) {
    x_course_ned = Eigen::Vector3d::UnitX();
    y_body_ned = z_body_ned.cross(x_course_ned);
  }
  y_body_ned.normalize();

  Eigen::Vector3d x_body_ned = y_body_ned.cross(z_body_ned);
  x_body_ned.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_body_ned;
  rotation.col(1) = y_body_ned;
  rotation.col(2) = z_body_ned;
  return Eigen::Quaterniond(rotation).normalized();
}

inline double per_motor_max_thrust_n(const ControllerParams &params) {
  return params.max_total_thrust_n / static_cast<double>(kMotorCount);
}

inline std::array<double, kMotorCount> clamp_motor_thrusts(const std::array<double, kMotorCount> &motor_thrusts_n,
                                                           const ControllerParams &params) {
  std::array<double, kMotorCount> clamped{};
  const double upper = per_motor_max_thrust_n(params);
  for (int i = 0; i < kMotorCount; ++i) {
    clamped[i] = clamp_scalar(motor_thrusts_n[i], 0.0, upper);
  }
  return clamped;
}

inline std::array<double, kMotorCount> motor_thrusts_to_throttles(const std::array<double, kMotorCount> &motor_thrusts_n,
                                                                  const ControllerParams &params) {
  std::array<double, kMotorCount> throttles{};
  const auto thrusts = clamp_motor_thrusts(motor_thrusts_n, params);
  const double max_motor = per_motor_max_thrust_n(params);
  const double factor = clamp_scalar(params.thrust_model_factor, 0.0, 1.0);

  if (max_motor <= 1e-9) {
    return throttles;
  }

  for (int i = 0; i < kMotorCount; ++i) {
    const double rel = clamp_scalar(thrusts[i] / max_motor, 0.0, 1.0);
    if (factor <= 1e-9) {
      throttles[i] = rel;
      continue;
    }

    const double a = factor;
    const double b = 1.0 - factor;
    const double tmp1 = b / (2.0 * a);
    const double tmp2 = (b * b) / (4.0 * a * a);
    throttles[i] = clamp_scalar(-tmp1 + std::sqrt(tmp2 + rel / a), 0.0, 1.0);
  }

  return throttles;
}

inline double collective_thrust_normalized_from_motor_thrusts(const std::array<double, kMotorCount> &motor_thrusts_n,
                                                              const ControllerParams &params) {
  const auto thrusts = clamp_motor_thrusts(motor_thrusts_n, params);
  double sum = 0.0;
  for (double value : thrusts) {
    sum += value;
  }

  if (params.max_total_thrust_n <= 1e-9) {
    return 0.0;
  }

  return clamp_scalar(sum / params.max_total_thrust_n, 0.0, 1.0);
}

inline std::array<double, kMotorCount> quad_x_allocate(double total_thrust_n, const Eigen::Vector3d &torque_frd,
                                                       const ControllerParams &params) {
  std::array<double, kMotorCount> thrusts{};
  const double arm_xy = params.arm_length_m / std::sqrt(2.0);
  const double yaw_coeff = std::max(params.yaw_moment_coeff, 1e-9);

  thrusts[0] = 0.25 * total_thrust_n - 0.25 * torque_frd.x() / arm_xy + 0.25 * torque_frd.y() / arm_xy + 0.25 * torque_frd.z() / yaw_coeff;
  thrusts[1] = 0.25 * total_thrust_n + 0.25 * torque_frd.x() / arm_xy - 0.25 * torque_frd.y() / arm_xy + 0.25 * torque_frd.z() / yaw_coeff;
  thrusts[2] = 0.25 * total_thrust_n + 0.25 * torque_frd.x() / arm_xy + 0.25 * torque_frd.y() / arm_xy - 0.25 * torque_frd.z() / yaw_coeff;
  thrusts[3] = 0.25 * total_thrust_n - 0.25 * torque_frd.x() / arm_xy - 0.25 * torque_frd.y() / arm_xy - 0.25 * torque_frd.z() / yaw_coeff;

  return clamp_motor_thrusts(thrusts, params);
}

inline WrenchFrd quad_x_wrench_from_motor_thrusts(const std::array<double, kMotorCount> &motor_thrusts_n,
                                                  const ControllerParams &params) {
  const auto thrusts = clamp_motor_thrusts(motor_thrusts_n, params);
  const double arm_xy = params.arm_length_m / std::sqrt(2.0);
  const double yaw_coeff = std::max(params.yaw_moment_coeff, 1e-9);

  WrenchFrd wrench{};
  for (double thrust : thrusts) {
    wrench.total_thrust_n += thrust;
  }

  if (arm_xy <= 1e-9) {
    return wrench;
  }

  wrench.torque_frd.x() = arm_xy * (-thrusts[0] + thrusts[1] + thrusts[2] - thrusts[3]);
  wrench.torque_frd.y() = arm_xy * (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3]);
  wrench.torque_frd.z() = yaw_coeff * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3]);
  return wrench;
}

} // namespace px4adrc
