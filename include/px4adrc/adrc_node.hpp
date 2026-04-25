#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>

#include "px4adrc/adrc_controller.hpp"
#include "px4adrc/msg/flat_trajectory_reference.hpp"

namespace px4adrc {

enum class MissionState {
  WAIT_FOR_STATE,
  REQUEST_OFFBOARD,
  WAIT_FOR_MANUAL_ARM,
  TAKEOFF,
  HOLD,
  TRACKING,
};

std::string to_string(MissionState state);

MissionState next_mission_state(MissionState current, bool offboard_enabled, bool armed, bool takeoff_complete, bool reference_fresh);

bool should_publish_start_tracking_signal(MissionState state, bool hold_time_elapsed, bool already_sent);

TrajectoryReference trajectory_reference_from_msg(const px4adrc::msg::FlatTrajectoryReference &msg, uint64_t timestamp_us);
TrajectoryReference non_tracking_reference(const TrajectoryReference &hold_ref, MissionState mission_state, double takeoff_target_z_ned);

class AdrcNode : public rclcpp::Node {
public:
  explicit AdrcNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_parameters();
  void create_ros_interfaces();

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void reference_callback(const px4adrc::msg::FlatTrajectoryReference::SharedPtr msg);

  void run_position_loop(uint64_t now_us);
  void run_attitude_loop(uint64_t now_us);
  void update_mission_state(uint64_t now_us);
  TrajectoryReference active_reference(uint64_t now_us) const;
  bool ready_for_control() const;
  void reset_controller_state();
  void update_hold_reference(double target_z_ned);
  bool has_fresh_reference(uint64_t now_us) const;
  double compute_loop_dt(uint64_t now_us, uint64_t *last_loop_us);

  void publish_offboard_control_mode(uint64_t now_us);
  void publish_vehicle_command(uint16_t command, float param1, float param2, float param3);
  void publish_start_tracking_signal(bool enabled) const;
  void publish_actuator_motors(uint64_t timestamp_us, uint64_t timestamp_sample_us,
                               const std::array<double, kMotorCount> &motor_throttles) const;
  void publish_vehicle_thrust_setpoint(uint64_t timestamp_us, uint64_t timestamp_sample_us,
                                       const std::array<double, kMotorCount> &motor_thrusts_n) const;

  AdrcController controller_;
  VehicleState state_{};
  TrajectoryReference hold_ref_{};
  TrajectoryReference external_ref_{};
  PositionControlOutput latest_position_output_{};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4adrc::msg::FlatTrajectoryReference>::SharedPtr reference_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_tracking_pub_;

  MissionState mission_state_{MissionState::WAIT_FOR_STATE};
  bool has_local_position_{false};
  bool has_attitude_{false};
  bool has_angular_velocity_{false};
  bool has_status_{false};
  bool is_armed_{false};
  bool is_offboard_{false};
  bool offboard_ever_engaged_{false};
  bool exit_requested_{false};
  bool start_tracking_sent_{false};
  bool has_position_output_{false};

  uint64_t last_position_control_us_{0};
  uint64_t last_attitude_control_us_{0};
  uint64_t last_reference_receive_us_{0};
  uint64_t hold_enter_us_{0};
  double captured_yaw_{0.0};

  std::string topic_reference_{"/px4adrc/reference"};
  std::string topic_start_tracking_{"/mission/start_tracking"};
  double takeoff_target_z_ned_{-2.0};
  double takeoff_z_threshold_m_{0.2};
  double takeoff_hold_time_s_{2.0};
  double reference_timeout_s_{0.3};
};

} // namespace px4adrc
