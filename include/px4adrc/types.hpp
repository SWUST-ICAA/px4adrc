#pragma once

#include "px4adrc/adrc_blocks.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>

namespace px4adrc {

constexpr int kMotorCount = 4;

struct ControllerParams {
  double mass_kg{1.0};
  double gravity{9.81};
  Eigen::Vector3d inertia_diag{0.01, 0.01, 0.02};
  double arm_length_m{0.2};
  double yaw_moment_coeff{0.01};
  double max_total_thrust_n{40.0};
  double thrust_model_factor{1.0};
  double max_tilt_rad{35.0 * M_PI / 180.0};
  std::array<TrackingDifferentiatorGains, 3> position_td_gains{{
      TrackingDifferentiatorGains{1.0},
      TrackingDifferentiatorGains{1.0},
      TrackingDifferentiatorGains{1.0},
  }};
  std::array<EsoGains, 3> position_eso_gains{{
      EsoGains{1.0, 1.0, 1.0, 1.0},
      EsoGains{1.0, 1.0, 1.0, 1.0},
      EsoGains{1.0, 1.0, 1.0, 1.0},
  }};
  std::array<NlsefGains, 3> position_nlsef_gains{{
      NlsefGains{2.5, 1.5, 0.8, 0.5, 0.01},
      NlsefGains{2.5, 1.5, 0.8, 0.5, 0.01},
      NlsefGains{2.0, 1.2, 0.8, 0.5, 0.01},
  }};
  std::array<TrackingDifferentiatorGains, 3> attitude_td_gains{{
      TrackingDifferentiatorGains{1.0},
      TrackingDifferentiatorGains{1.0},
      TrackingDifferentiatorGains{1.0},
  }};
  std::array<EsoGains, 3> attitude_eso_gains{{
      EsoGains{1.0, 1.0, 1.0, 1.0},
      EsoGains{1.0, 1.0, 1.0, 1.0},
      EsoGains{1.0, 1.0, 1.0, 1.0},
  }};
  std::array<NlsefGains, 3> attitude_nlsef_gains{{
      NlsefGains{0.8, 0.05, 1.0, 1.0, 0.01},
      NlsefGains{0.8, 0.05, 1.0, 1.0, 0.01},
      NlsefGains{0.5, 0.03, 1.0, 1.0, 0.01},
  }};
};

struct VehicleState {
  uint64_t timestamp_us{0};
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rates_frd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_accel_frd{Eigen::Vector3d::Zero()};
};

struct TrajectoryReference {
  uint64_t timestamp_us{0};
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_rates_frd{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_torque_frd{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  bool valid{false};
};

struct ControlOutput {
  double total_thrust_n{0.0};
  Eigen::Vector3d torque_frd{Eigen::Vector3d::Zero()};
  std::array<double, kMotorCount> motor_thrusts_n{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, kMotorCount> motor_throttles{{0.0, 0.0, 0.0, 0.0}};
};

} // namespace px4adrc
