#include "px4adrc/adrc_node.hpp"

#include "px4adrc/common.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace px4adrc
{

namespace
{

uint64_t now_micros(rclcpp::Clock & clock)
{
  return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
}

double quaternion_to_yaw(const Eigen::Quaterniond & q_body_to_ned)
{
  const Eigen::Quaterniond q = q_body_to_ned.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

bool is_finite_vec3(const std::array<float, 3> & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

std::array<double, 3> vector_param_or_default(
  rclcpp::Node & node,
  const std::string & name,
  const std::array<double, 3> & defaults)
{
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != 3) {
    return defaults;
  }

  return {values[0], values[1], values[2]};
}

void assign_axis_gains(
  std::array<NlsefGains, 3> & target,
  const std::array<double, 3> & k1,
  const std::array<double, 3> & k2,
  const std::array<double, 3> & alpha1,
  const std::array<double, 3> & alpha2,
  const std::array<double, 3> & delta)
{
  for (int i = 0; i < 3; ++i) {
    target[i].k1 = k1[i];
    target[i].k2 = k2[i];
    target[i].alpha1 = alpha1[i];
    target[i].alpha2 = alpha2[i];
    target[i].delta = delta[i];
  }
}

void assign_td_axis_gains(
  std::array<TrackingDifferentiatorGains, 3> & target,
  const std::array<double, 3> & r)
{
  for (int i = 0; i < 3; ++i) {
    target[i].r = std::max(0.0, r[i]);
  }
}

void assign_eso_axis_gains(
  std::array<EsoGains, 3> & target,
  const std::array<double, 3> & beta1,
  const std::array<double, 3> & beta2,
  const std::array<double, 3> & beta3,
  const std::array<double, 3> & b0)
{
  for (int i = 0; i < 3; ++i) {
    target[i].beta1 = std::max(0.0, beta1[i]);
    target[i].beta2 = std::max(0.0, beta2[i]);
    target[i].beta3 = std::max(0.0, beta3[i]);
    target[i].b0 = b0[i];
  }
}

void assign_vector3(Eigen::Vector3d & target, const std::array<double, 3> & values)
{
  target = Eigen::Vector3d(values[0], values[1], values[2]);
}

}  // namespace

bool reference_message_has_valid_position(const px4adrc::msg::FlatTrajectoryReference & msg)
{
  return std::isfinite(msg.position_ned.x) &&
         std::isfinite(msg.position_ned.y) &&
         std::isfinite(msg.position_ned.z);
}

TrajectoryReference trajectory_reference_from_msg(
  const px4adrc::msg::FlatTrajectoryReference & msg,
  uint64_t timestamp_us)
{
  TrajectoryReference ref{};
  ref.timestamp_us = timestamp_us;

  if (!reference_message_has_valid_position(msg)) {
    return ref;
  }

  ref.position_ned = Eigen::Vector3d(
    msg.position_ned.x,
    msg.position_ned.y,
    msg.position_ned.z);
  ref.velocity_ned = Eigen::Vector3d(
    sanitize_scalar(msg.velocity_ned.x),
    sanitize_scalar(msg.velocity_ned.y),
    sanitize_scalar(msg.velocity_ned.z));
  ref.acceleration_ned = Eigen::Vector3d(
    sanitize_scalar(msg.acceleration_ned.x),
    sanitize_scalar(msg.acceleration_ned.y),
    sanitize_scalar(msg.acceleration_ned.z));
  ref.body_rates_frd = Eigen::Vector3d(
    sanitize_scalar(msg.body_rates_frd.x),
    sanitize_scalar(msg.body_rates_frd.y),
    sanitize_scalar(msg.body_rates_frd.z));
  ref.body_torque_frd = Eigen::Vector3d(
    sanitize_scalar(msg.body_torque_frd.x),
    sanitize_scalar(msg.body_torque_frd.y),
    sanitize_scalar(msg.body_torque_frd.z));
  ref.yaw = sanitize_scalar(msg.yaw);
  ref.valid = true;
  return ref;
}

std::string to_string(MissionState state)
{
  switch (state) {
    case MissionState::WAIT_FOR_STATE:
      return "WAIT_FOR_STATE";
    case MissionState::REQUEST_OFFBOARD:
      return "REQUEST_OFFBOARD";
    case MissionState::WAIT_FOR_MANUAL_ARM:
      return "WAIT_FOR_MANUAL_ARM";
    case MissionState::TAKEOFF:
      return "TAKEOFF";
    case MissionState::HOLD:
      return "HOLD";
    case MissionState::TRACKING:
      return "TRACKING";
  }
  return "UNKNOWN";
}

MissionState next_mission_state(
  MissionState current,
  bool offboard_enabled,
  bool armed,
  bool takeoff_complete,
  bool reference_fresh)
{
  switch (current) {
    case MissionState::WAIT_FOR_STATE:
      return MissionState::REQUEST_OFFBOARD;
    case MissionState::REQUEST_OFFBOARD:
      return offboard_enabled ? MissionState::WAIT_FOR_MANUAL_ARM : MissionState::REQUEST_OFFBOARD;
    case MissionState::WAIT_FOR_MANUAL_ARM:
      return armed ? MissionState::TAKEOFF : MissionState::WAIT_FOR_MANUAL_ARM;
    case MissionState::TAKEOFF:
      return takeoff_complete ? MissionState::HOLD : MissionState::TAKEOFF;
    case MissionState::HOLD:
      return MissionState::HOLD;
    case MissionState::TRACKING:
      return reference_fresh ? MissionState::TRACKING : MissionState::HOLD;
  }
  return current;
}

bool should_publish_start_tracking_signal(
  MissionState state,
  bool hold_time_elapsed,
  bool already_sent)
{
  return state == MissionState::HOLD && hold_time_elapsed && !already_sent;
}

AdrcNode::AdrcNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("px4adrc", options), controller_(ControllerParams{})
{
  declare_parameters();
  load_parameters();
  create_ros_interfaces();
}

void AdrcNode::declare_parameters()
{
  const ControllerParams defaults{};
  this->declare_parameter<std::string>("topics.reference", topic_reference_);
  this->declare_parameter<std::string>("topics.start_tracking", topic_start_tracking_);
  this->declare_parameter<double>("control_rate_hz", control_rate_hz_);
  this->declare_parameter<double>("mission.takeoff_target_z", takeoff_target_z_ned_);
  this->declare_parameter<double>("mission.takeoff_z_threshold", takeoff_z_threshold_m_);
  this->declare_parameter<double>("mission.takeoff_hold_time_s", takeoff_hold_time_s_);
  this->declare_parameter<double>("mission.reference_timeout_s", reference_timeout_s_);
  this->declare_parameter<double>("position_adrc.max_tilt_deg", 35.0);
  this->declare_parameter<std::vector<double>>(
    "position_adrc.td_r",
    {defaults.position_td_gains[0].r, defaults.position_td_gains[1].r, defaults.position_td_gains[2].r});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.eso_beta1",
    {defaults.position_eso_gains[0].beta1, defaults.position_eso_gains[1].beta1, defaults.position_eso_gains[2].beta1});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.eso_beta2",
    {defaults.position_eso_gains[0].beta2, defaults.position_eso_gains[1].beta2, defaults.position_eso_gains[2].beta2});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.eso_beta3",
    {defaults.position_eso_gains[0].beta3, defaults.position_eso_gains[1].beta3, defaults.position_eso_gains[2].beta3});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.eso_b0",
    {defaults.position_eso_gains[0].b0, defaults.position_eso_gains[1].b0, defaults.position_eso_gains[2].b0});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.nlsef_k1",
    {defaults.position_nlsef_gains[0].k1, defaults.position_nlsef_gains[1].k1, defaults.position_nlsef_gains[2].k1});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.nlsef_k2",
    {defaults.position_nlsef_gains[0].k2, defaults.position_nlsef_gains[1].k2, defaults.position_nlsef_gains[2].k2});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.nlsef_alpha1",
    {defaults.position_nlsef_gains[0].alpha1, defaults.position_nlsef_gains[1].alpha1, defaults.position_nlsef_gains[2].alpha1});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.nlsef_alpha2",
    {defaults.position_nlsef_gains[0].alpha2, defaults.position_nlsef_gains[1].alpha2, defaults.position_nlsef_gains[2].alpha2});
  this->declare_parameter<std::vector<double>>(
    "position_adrc.nlsef_delta",
    {defaults.position_nlsef_gains[0].delta, defaults.position_nlsef_gains[1].delta, defaults.position_nlsef_gains[2].delta});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.attitude_kp",
    {defaults.attitude_kp.x(), defaults.attitude_kp.y(), defaults.attitude_kp.z()});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_td_r",
    {defaults.rate_td_gains[0].r, defaults.rate_td_gains[1].r, defaults.rate_td_gains[2].r});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_eso_beta1",
    {defaults.rate_eso_gains[0].beta1, defaults.rate_eso_gains[1].beta1, defaults.rate_eso_gains[2].beta1});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_eso_beta2",
    {defaults.rate_eso_gains[0].beta2, defaults.rate_eso_gains[1].beta2, defaults.rate_eso_gains[2].beta2});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_eso_beta3",
    {defaults.rate_eso_gains[0].beta3, defaults.rate_eso_gains[1].beta3, defaults.rate_eso_gains[2].beta3});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_eso_b0",
    {defaults.rate_eso_gains[0].b0, defaults.rate_eso_gains[1].b0, defaults.rate_eso_gains[2].b0});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_nlsef_k1",
    {defaults.rate_nlsef_gains[0].k1, defaults.rate_nlsef_gains[1].k1, defaults.rate_nlsef_gains[2].k1});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_nlsef_k2",
    {defaults.rate_nlsef_gains[0].k2, defaults.rate_nlsef_gains[1].k2, defaults.rate_nlsef_gains[2].k2});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_nlsef_alpha1",
    {defaults.rate_nlsef_gains[0].alpha1, defaults.rate_nlsef_gains[1].alpha1, defaults.rate_nlsef_gains[2].alpha1});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_nlsef_alpha2",
    {defaults.rate_nlsef_gains[0].alpha2, defaults.rate_nlsef_gains[1].alpha2, defaults.rate_nlsef_gains[2].alpha2});
  this->declare_parameter<std::vector<double>>(
    "attitude_adrc.rate_nlsef_delta",
    {defaults.rate_nlsef_gains[0].delta, defaults.rate_nlsef_gains[1].delta, defaults.rate_nlsef_gains[2].delta});
  this->declare_parameter<double>("filters.velocity_diff_cutoff_hz", velocity_diff_cutoff_hz_);
  this->declare_parameter<double>("filters.body_rate_diff_cutoff_hz", body_rate_diff_cutoff_hz_);
  this->declare_parameter<double>("vehicle.mass_kg", 1.0);
  this->declare_parameter<double>("vehicle.gravity", 9.81);
  this->declare_parameter<std::vector<double>>("vehicle.inertia_diag", {0.01, 0.01, 0.02});
  this->declare_parameter<double>("vehicle.arm_length_m", 0.2);
  this->declare_parameter<double>("vehicle.yaw_moment_coeff", 0.01);
  this->declare_parameter<double>("vehicle.max_total_thrust_n", 40.0);
  this->declare_parameter<double>("motors.thrust_model_factor", 1.0);
}

void AdrcNode::load_parameters()
{
  topic_reference_ = this->get_parameter("topics.reference").as_string();
  topic_start_tracking_ = this->get_parameter("topics.start_tracking").as_string();
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  takeoff_target_z_ned_ = this->get_parameter("mission.takeoff_target_z").as_double();
  takeoff_z_threshold_m_ = std::max(0.0, this->get_parameter("mission.takeoff_z_threshold").as_double());
  takeoff_hold_time_s_ = std::max(0.0, this->get_parameter("mission.takeoff_hold_time_s").as_double());
  reference_timeout_s_ = std::max(0.0, this->get_parameter("mission.reference_timeout_s").as_double());
  velocity_diff_cutoff_hz_ = std::max(
    0.0, this->get_parameter("filters.velocity_diff_cutoff_hz").as_double());
  body_rate_diff_cutoff_hz_ = std::max(
    0.0, this->get_parameter("filters.body_rate_diff_cutoff_hz").as_double());

  ControllerParams params{};
  params.mass_kg = this->get_parameter("vehicle.mass_kg").as_double();
  params.gravity = this->get_parameter("vehicle.gravity").as_double();
  params.arm_length_m = this->get_parameter("vehicle.arm_length_m").as_double();
  params.yaw_moment_coeff = this->get_parameter("vehicle.yaw_moment_coeff").as_double();
  params.max_total_thrust_n = this->get_parameter("vehicle.max_total_thrust_n").as_double();
  params.thrust_model_factor = this->get_parameter("motors.thrust_model_factor").as_double();
  params.max_tilt_rad = this->get_parameter("position_adrc.max_tilt_deg").as_double() * M_PI / 180.0;

  const auto inertia_diag = this->get_parameter("vehicle.inertia_diag").as_double_array();
  if (inertia_diag.size() == 3) {
    params.inertia_diag = Eigen::Vector3d(inertia_diag[0], inertia_diag[1], inertia_diag[2]);
  }

  assign_td_axis_gains(
    params.position_td_gains,
    vector_param_or_default(
      *this, "position_adrc.td_r",
      {params.position_td_gains[0].r, params.position_td_gains[1].r, params.position_td_gains[2].r}));
  assign_eso_axis_gains(
    params.position_eso_gains,
    vector_param_or_default(
      *this, "position_adrc.eso_beta1",
      {params.position_eso_gains[0].beta1, params.position_eso_gains[1].beta1, params.position_eso_gains[2].beta1}),
    vector_param_or_default(
      *this, "position_adrc.eso_beta2",
      {params.position_eso_gains[0].beta2, params.position_eso_gains[1].beta2, params.position_eso_gains[2].beta2}),
    vector_param_or_default(
      *this, "position_adrc.eso_beta3",
      {params.position_eso_gains[0].beta3, params.position_eso_gains[1].beta3, params.position_eso_gains[2].beta3}),
    vector_param_or_default(
      *this, "position_adrc.eso_b0",
      {params.position_eso_gains[0].b0, params.position_eso_gains[1].b0, params.position_eso_gains[2].b0}));
  assign_axis_gains(
    params.position_nlsef_gains,
    vector_param_or_default(
      *this, "position_adrc.nlsef_k1",
      {params.position_nlsef_gains[0].k1, params.position_nlsef_gains[1].k1, params.position_nlsef_gains[2].k1}),
    vector_param_or_default(
      *this, "position_adrc.nlsef_k2",
      {params.position_nlsef_gains[0].k2, params.position_nlsef_gains[1].k2, params.position_nlsef_gains[2].k2}),
    vector_param_or_default(
      *this, "position_adrc.nlsef_alpha1",
      {params.position_nlsef_gains[0].alpha1, params.position_nlsef_gains[1].alpha1, params.position_nlsef_gains[2].alpha1}),
    vector_param_or_default(
      *this, "position_adrc.nlsef_alpha2",
      {params.position_nlsef_gains[0].alpha2, params.position_nlsef_gains[1].alpha2, params.position_nlsef_gains[2].alpha2}),
    vector_param_or_default(
      *this, "position_adrc.nlsef_delta",
      {params.position_nlsef_gains[0].delta, params.position_nlsef_gains[1].delta, params.position_nlsef_gains[2].delta}));

  assign_vector3(
    params.attitude_kp,
    vector_param_or_default(
      *this, "attitude_adrc.attitude_kp",
      {params.attitude_kp.x(), params.attitude_kp.y(), params.attitude_kp.z()}));
  assign_td_axis_gains(
    params.rate_td_gains,
    vector_param_or_default(
      *this, "attitude_adrc.rate_td_r",
      {params.rate_td_gains[0].r, params.rate_td_gains[1].r, params.rate_td_gains[2].r}));
  assign_eso_axis_gains(
    params.rate_eso_gains,
    vector_param_or_default(
      *this, "attitude_adrc.rate_eso_beta1",
      {params.rate_eso_gains[0].beta1, params.rate_eso_gains[1].beta1, params.rate_eso_gains[2].beta1}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_eso_beta2",
      {params.rate_eso_gains[0].beta2, params.rate_eso_gains[1].beta2, params.rate_eso_gains[2].beta2}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_eso_beta3",
      {params.rate_eso_gains[0].beta3, params.rate_eso_gains[1].beta3, params.rate_eso_gains[2].beta3}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_eso_b0",
      {params.rate_eso_gains[0].b0, params.rate_eso_gains[1].b0, params.rate_eso_gains[2].b0}));
  assign_axis_gains(
    params.rate_nlsef_gains,
    vector_param_or_default(
      *this, "attitude_adrc.rate_nlsef_k1",
      {params.rate_nlsef_gains[0].k1, params.rate_nlsef_gains[1].k1, params.rate_nlsef_gains[2].k1}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_nlsef_k2",
      {params.rate_nlsef_gains[0].k2, params.rate_nlsef_gains[1].k2, params.rate_nlsef_gains[2].k2}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_nlsef_alpha1",
      {params.rate_nlsef_gains[0].alpha1, params.rate_nlsef_gains[1].alpha1, params.rate_nlsef_gains[2].alpha1}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_nlsef_alpha2",
      {params.rate_nlsef_gains[0].alpha2, params.rate_nlsef_gains[1].alpha2, params.rate_nlsef_gains[2].alpha2}),
    vector_param_or_default(
      *this, "attitude_adrc.rate_nlsef_delta",
      {params.rate_nlsef_gains[0].delta, params.rate_nlsef_gains[1].delta, params.rate_nlsef_gains[2].delta}));

  controller_.set_params(params);
}

void AdrcNode::create_ros_interfaces()
{
  const auto qos_px4_out = rclcpp::SensorDataQoS();
  const auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10));
  const auto qos_start_tracking = rclcpp::QoS(1).reliable().transient_local();

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos_px4_out,
    std::bind(&AdrcNode::odom_callback, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1",
    qos_px4_out,
    std::bind(&AdrcNode::vehicle_status_callback, this, std::placeholders::_1));
  reference_sub_ = this->create_subscription<px4adrc::msg::FlatTrajectoryReference>(
    topic_reference_,
    qos_default,
    std::bind(&AdrcNode::reference_callback, this, std::placeholders::_1));

  offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos_default);
  actuator_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
    "/fmu/in/actuator_motors", qos_default);
  thrust_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
    "/fmu/in/vehicle_thrust_setpoint", qos_default);
  command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos_default);
  start_tracking_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    topic_start_tracking_, qos_start_tracking);
  publish_start_tracking_signal(false);

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AdrcNode::control_loop, this));
}

void AdrcNode::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!is_finite_vec3(msg->position)) {
    return;
  }

  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "vehicle_odometry pose frame is not NED, controller still interprets data as NED");
  }

  const uint64_t timestamp_us = msg->timestamp;
  double dt = 0.0;
  if (last_odom_us_ != 0 && timestamp_us > last_odom_us_) {
    dt = std::clamp(static_cast<double>(timestamp_us - last_odom_us_) * 1e-6, 1e-4, 0.05);
  }
  last_odom_us_ = timestamp_us;

  state_.timestamp_us = timestamp_us;
  state_.position_ned = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);

  Eigen::Quaterniond q_body_to_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  q_body_to_ned.normalize();
  state_.q_body_to_ned = q_body_to_ned;
  captured_yaw_ = quaternion_to_yaw(q_body_to_ned);

  const Eigen::Vector3d velocity_raw(
    std::isfinite(msg->velocity[0]) ? msg->velocity[0] : 0.0,
    std::isfinite(msg->velocity[1]) ? msg->velocity[1] : 0.0,
    std::isfinite(msg->velocity[2]) ? msg->velocity[2] : 0.0);

  if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
    state_.velocity_ned = q_body_to_ned * velocity_raw;
  } else {
    state_.velocity_ned = velocity_raw;
  }

  state_.body_rates_frd = Eigen::Vector3d(
    std::isfinite(msg->angular_velocity[0]) ? msg->angular_velocity[0] : 0.0,
    std::isfinite(msg->angular_velocity[1]) ? msg->angular_velocity[1] : 0.0,
    std::isfinite(msg->angular_velocity[2]) ? msg->angular_velocity[2] : 0.0);

  if (dt > 0.0) {
    const Eigen::Vector3d raw_acceleration_ned = (state_.velocity_ned - last_velocity_ned_) / dt;
    const Eigen::Vector3d raw_body_accel_frd = (state_.body_rates_frd - last_body_rates_frd_) / dt;
    state_.acceleration_ned = low_pass_vec3(
      state_.acceleration_ned, raw_acceleration_ned, velocity_diff_cutoff_hz_, dt);
    state_.body_accel_frd = low_pass_vec3(
      state_.body_accel_frd, raw_body_accel_frd, body_rate_diff_cutoff_hz_, dt);
  }
  last_velocity_ned_ = state_.velocity_ned;
  last_body_rates_frd_ = state_.body_rates_frd;

  has_state_ = true;
}

void AdrcNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool was_offboard = is_offboard_;
  is_armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  is_offboard_ = (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
  has_status_ = true;

  if (is_offboard_) {
    offboard_ever_engaged_ = true;
  }

  if (offboard_ever_engaged_ && was_offboard && !is_offboard_) {
    exit_requested_ = true;
  }
}

void AdrcNode::reference_callback(const px4adrc::msg::FlatTrajectoryReference::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!reference_message_has_valid_position(*msg)) {
    external_ref_.valid = false;
    return;
  }

  const uint64_t timestamp_us = now_micros(*this->get_clock());
  external_ref_ = trajectory_reference_from_msg(*msg, timestamp_us);
  last_reference_receive_us_ = timestamp_us;
}

double AdrcNode::compute_control_dt(uint64_t now_us)
{
  double dt = 1.0 / std::max(control_rate_hz_, 1.0);
  if (last_control_us_ != 0 && now_us > last_control_us_) {
    dt = static_cast<double>(now_us - last_control_us_) * 1e-6;
  }
  last_control_us_ = now_us;
  return std::clamp(dt, 1e-4, 0.05);
}

void AdrcNode::update_hold_reference(double target_z_ned)
{
  hold_ref_.timestamp_us = state_.timestamp_us;
  hold_ref_.position_ned = state_.position_ned;
  hold_ref_.position_ned.z() = target_z_ned;
  hold_ref_.velocity_ned.setZero();
  hold_ref_.acceleration_ned.setZero();
  hold_ref_.body_rates_frd.setZero();
  hold_ref_.body_torque_frd.setZero();
  hold_ref_.yaw = captured_yaw_;
  hold_ref_.valid = true;
}

bool AdrcNode::has_fresh_reference(uint64_t now_us) const
{
  if (!external_ref_.valid || last_reference_receive_us_ == 0) {
    return false;
  }

  return static_cast<double>(now_us - last_reference_receive_us_) * 1e-6 <= reference_timeout_s_;
}

void AdrcNode::publish_offboard_control_mode(uint64_t now_us)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = now_us;
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = true;
  offboard_mode_pub_->publish(msg);
}

void AdrcNode::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = state_.timestamp_us != 0 ? state_.timestamp_us : now_micros(*this->get_clock());
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  command_pub_->publish(msg);
}

void AdrcNode::publish_start_tracking_signal(bool enabled) const
{
  if (!start_tracking_pub_) {
    return;
  }

  std_msgs::msg::Bool msg{};
  msg.data = enabled;
  start_tracking_pub_->publish(msg);
}

void AdrcNode::publish_actuator_motors(
  uint64_t timestamp_us,
  uint64_t timestamp_sample_us,
  const std::array<double, kMotorCount> & motor_throttles) const
{
  px4_msgs::msg::ActuatorMotors msg{};
  msg.timestamp = timestamp_us;
  msg.timestamp_sample = timestamp_sample_us;
  msg.reversible_flags = 0U;

  for (float & value : msg.control) {
    value = std::numeric_limits<float>::quiet_NaN();
  }

  for (int i = 0; i < kMotorCount; ++i) {
    msg.control[i] = static_cast<float>(motor_throttles[i]);
  }

  actuator_pub_->publish(msg);
}

void AdrcNode::publish_vehicle_thrust_setpoint(
  uint64_t timestamp_us,
  uint64_t timestamp_sample_us,
  const std::array<double, kMotorCount> & motor_thrusts_n) const
{
  px4_msgs::msg::VehicleThrustSetpoint msg{};
  msg.timestamp = timestamp_us;
  msg.timestamp_sample = timestamp_sample_us;
  msg.xyz[0] = 0.0F;
  msg.xyz[1] = 0.0F;
  msg.xyz[2] = static_cast<float>(
    -collective_thrust_normalized_from_motor_thrusts(motor_thrusts_n, controller_.params()));
  thrust_pub_->publish(msg);
}

void AdrcNode::control_loop()
{
  if (exit_requested_) {
    RCLCPP_ERROR(this->get_logger(), "Offboard mode exited after engagement, shutting down controller");
    rclcpp::shutdown();
    return;
  }

  if (!has_state_ || !has_status_) {
    return;
  }

  const uint64_t now_us = state_.timestamp_us != 0 ? state_.timestamp_us : now_micros(*this->get_clock());
  const double dt = compute_control_dt(now_us);
  publish_offboard_control_mode(now_us);

  if (mission_state_ == MissionState::WAIT_FOR_STATE) {
    update_hold_reference(state_.position_ned.z());
    mission_state_ = MissionState::REQUEST_OFFBOARD;
  }

  if (mission_state_ == MissionState::REQUEST_OFFBOARD) {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      6.0F,
      0.0F);
  }

  bool takeoff_complete = false;
  if (mission_state_ == MissionState::TAKEOFF || mission_state_ == MissionState::HOLD ||
    mission_state_ == MissionState::TRACKING)
  {
    takeoff_complete = std::abs(state_.position_ned.z() - takeoff_target_z_ned_) <= takeoff_z_threshold_m_;
  }

  const bool reference_fresh = has_fresh_reference(now_us);
  const MissionState next_state = next_mission_state(
    mission_state_, is_offboard_, is_armed_, takeoff_complete, reference_fresh);

  if (mission_state_ != next_state) {
    if (next_state == MissionState::WAIT_FOR_MANUAL_ARM) {
      update_hold_reference(state_.position_ned.z());
    } else if (next_state == MissionState::TAKEOFF) {
      update_hold_reference(takeoff_target_z_ned_);
    } else if (next_state == MissionState::HOLD) {
      update_hold_reference(takeoff_target_z_ned_);
      hold_enter_us_ = now_us;
    }
    mission_state_ = next_state;
  }

  const bool hold_time_elapsed =
    mission_state_ == MissionState::HOLD &&
    static_cast<double>(now_us - hold_enter_us_) * 1e-6 >= takeoff_hold_time_s_;
  if (should_publish_start_tracking_signal(mission_state_, hold_time_elapsed, start_tracking_sent_)) {
    publish_start_tracking_signal(true);
    start_tracking_sent_ = true;
  }

  if (mission_state_ == MissionState::HOLD && hold_time_elapsed && reference_fresh) {
    mission_state_ = MissionState::TRACKING;
  }

  TrajectoryReference active_ref = hold_ref_;
  if (mission_state_ == MissionState::TRACKING && reference_fresh) {
    active_ref = external_ref_;
  } else if (mission_state_ == MissionState::TAKEOFF) {
    active_ref = hold_ref_;
    active_ref.position_ned.z() = takeoff_target_z_ned_;
  }

  const auto output = controller_.update(state_, active_ref, dt);
  publish_actuator_motors(now_us, state_.timestamp_us, output.motor_throttles);
  publish_vehicle_thrust_setpoint(now_us, state_.timestamp_us, output.motor_thrusts_n);
}

}  // namespace px4adrc
