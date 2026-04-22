# px4adrc

ROS2 PX4 quadrotor controller package using a dual-loop ADRC structure and direct motor actuation.

## Features

- PX4-native `NED` and `FRD` control path
- Position loop driven by `/fmu/out/vehicle_local_position`
- Attitude state from `/fmu/out/vehicle_attitude`
- Attitude loop driven by `/fmu/out/vehicle_angular_velocity`
- Fixed Quad-X motor order for PX4 `direct_actuator` mode
- Per-motor thrust mapped to throttle using PX4 `THR_MDL_FAC`
- Continuous offboard requests after node startup
- Manual arm required before automatic takeoff
- Package-local `FlatTrajectoryReference` message and trajectory publisher

## Motor Order

The controller uses this fixed PX4 direct-actuator order:

1. front-right, CCW
2. back-left, CCW
3. front-left, CW
4. back-right, CW

No runtime `motor_output_map` parameter is kept.

## Flight Flow

1. Start the node.
2. The node continuously requests PX4 offboard mode.
3. Manually arm the vehicle.
4. The node performs automatic takeoff to the configured height.
5. After takeoff hold, the node publishes `start_tracking=true` on `/mission/start_tracking`.
6. The trajectory publisher starts emitting `/px4adrc/reference`.
7. The controller switches to tracking when a fresh reference arrives.

## State Inputs

- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_angular_velocity`
- `/fmu/out/vehicle_status_v1`

The controller requires valid `vehicle_local_position` XY/Z and VXY/VZ estimates before running control.

## Reference Contract

- `position_ned`, `velocity_ned`, `acceleration_ned`, `jerk_ned`, `snap_ned`: PX4 world frame `NED`
- `yaw`, `yaw_rate`, `yaw_acceleration`: desired heading and heading derivatives in `NED`

## Controller Parameters

Controller tuning lives in [config/px4adrc.yaml](/home/nanwan/work_dir/px4adrc_ws/src/px4adrc/config/px4adrc.yaml):

- `position_adrc.td_r`
- `position_adrc.eso_beta1/beta2/beta3/b0`
- `position_adrc.nlsef_k1/k2/alpha1/alpha2/delta`
- `attitude_adrc.attitude_eso_beta1/beta2/beta3/b0`
- `attitude_adrc.attitude_nlsef_k1/k2/alpha1/alpha2/delta`

## Notes

The node publishes both `/fmu/in/actuator_motors` and `/fmu/in/vehicle_thrust_setpoint` when using `direct_actuator`.
