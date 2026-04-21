#pragma once

#include "px4adrc/adrc_blocks.hpp"
#include "px4adrc/types.hpp"

#include <array>

namespace px4adrc {

class AdrcController {
public:
  explicit AdrcController(const ControllerParams &params);

  void set_params(const ControllerParams &params);
  const ControllerParams &params() const;

  PositionControlOutput update_position(const VehicleState &state, const TrajectoryReference &ref, double dt);
  ControlOutput update_attitude(const VehicleState &state, const PositionControlOutput &position_output, const TrajectoryReference &ref,
                                double dt);
  ControlOutput update(const VehicleState &state, const TrajectoryReference &ref, double dt);

private:
  ControllerParams params_{};
  std::array<TrackingDifferentiator, 3> position_td_{};
  std::array<TrackingDifferentiator, 3> attitude_td_{};
  std::array<ExtendedStateObserver, 3> pos_eso_{};
  std::array<ExtendedStateObserver, 3> attitude_eso_{};
  std::array<double, 3> last_position_accel_cmd_ned_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_attitude_feedback_torque_cmd_desired_frd_{{0.0, 0.0, 0.0}};
  bool position_td_initialized_{false};
  bool attitude_td_initialized_{false};
  bool position_states_initialized_{false};
  bool attitude_states_initialized_{false};
};

} // namespace px4adrc
