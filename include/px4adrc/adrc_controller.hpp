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

  ControlOutput update(const VehicleState &state,
                       const TrajectoryReference &ref, double dt);

private:
  ControllerParams params_{};
  std::array<TrackingDifferentiator, 3> pos_td_{};
  std::array<ExtendedStateObserver, 3> pos_eso_{};
  std::array<TrackingDifferentiator, 3> attitude_td_{};
  std::array<ExtendedStateObserver, 3> attitude_eso_{};
  std::array<double, 3> last_position_accel_cmd_ned_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_attitude_torque_cmd_desired_frd_{{0.0, 0.0, 0.0}};
};

} // namespace px4adrc
