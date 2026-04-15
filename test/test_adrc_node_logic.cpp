#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#include "px4adrc/adrc_node.hpp"

namespace {

std::string package_root_from_test_file() {
  const std::string file = __FILE__;
  const std::string marker = "/test/test_adrc_node_logic.cpp";
  const auto marker_pos = file.rfind(marker);
  return marker_pos == std::string::npos ? std::string(".") : file.substr(0, marker_pos);
}

std::string read_text_file(const std::string &path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

} // namespace

TEST(AdrcNodeLogic, StateNamesRemainStable) {
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::WAIT_FOR_STATE), std::string("WAIT_FOR_STATE"));
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::WAIT_FOR_MANUAL_ARM), std::string("WAIT_FOR_MANUAL_ARM"));
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::TRACKING), std::string("TRACKING"));
}

TEST(AdrcNodeLogic, OffboardWithoutArmWaitsForManualArm) {
  EXPECT_EQ(px4adrc::next_mission_state(px4adrc::MissionState::REQUEST_OFFBOARD, true, false, false, false),
            px4adrc::MissionState::WAIT_FOR_MANUAL_ARM);
}

TEST(AdrcNodeLogic, ReferenceTimeoutReturnsToHold) {
  EXPECT_EQ(px4adrc::next_mission_state(px4adrc::MissionState::TRACKING, true, true, false, false), px4adrc::MissionState::HOLD);
}

TEST(AdrcNodeLogic, NonTrackingReferenceKeepsHoldLogicButUsesCurrentYaw) {
  px4adrc::TrajectoryReference hold_ref{};
  hold_ref.valid = true;
  hold_ref.position_ned = Eigen::Vector3d(0.1, -0.2, -0.3);
  hold_ref.velocity_ned = Eigen::Vector3d(1.0, -1.0, 0.5);
  hold_ref.acceleration_ned = Eigen::Vector3d(2.0, -2.0, 1.0);
  hold_ref.yaw = 0.0;

  const auto wait_ref = px4adrc::non_tracking_reference(hold_ref, px4adrc::MissionState::WAIT_FOR_MANUAL_ARM, -1.0, 1.782249);
  EXPECT_DOUBLE_EQ(wait_ref.position_ned.x(), 0.1);
  EXPECT_DOUBLE_EQ(wait_ref.position_ned.y(), -0.2);
  EXPECT_DOUBLE_EQ(wait_ref.position_ned.z(), -0.3);
  EXPECT_DOUBLE_EQ(wait_ref.velocity_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(wait_ref.velocity_ned.y(), -1.0);
  EXPECT_DOUBLE_EQ(wait_ref.acceleration_ned.x(), 2.0);
  EXPECT_DOUBLE_EQ(wait_ref.acceleration_ned.y(), -2.0);
  EXPECT_DOUBLE_EQ(wait_ref.yaw, 1.782249);

  const auto takeoff_ref = px4adrc::non_tracking_reference(hold_ref, px4adrc::MissionState::TAKEOFF, -1.0, 1.782249);
  EXPECT_DOUBLE_EQ(takeoff_ref.position_ned.x(), 0.1);
  EXPECT_DOUBLE_EQ(takeoff_ref.position_ned.y(), -0.2);
  EXPECT_DOUBLE_EQ(takeoff_ref.position_ned.z(), -1.0);
  EXPECT_DOUBLE_EQ(takeoff_ref.yaw, 1.782249);
}

TEST(AdrcNodeLogic, HoldAfterTakeoffRequestsStartTrackingOnce) {
  EXPECT_TRUE(px4adrc::should_publish_start_tracking_signal(px4adrc::MissionState::HOLD, true, false));
  EXPECT_FALSE(px4adrc::should_publish_start_tracking_signal(px4adrc::MissionState::HOLD, true, true));
  EXPECT_FALSE(px4adrc::should_publish_start_tracking_signal(px4adrc::MissionState::TAKEOFF, true, false));
}

TEST(AdrcNodeLogic, ReferenceMessageConversionPreservesPositionFields) {
  px4adrc::msg::FlatTrajectoryReference msg{};
  msg.position_ned.x = std::numeric_limits<double>::quiet_NaN();
  msg.position_ned.y = 0.0;
  msg.position_ned.z = -2.0;
  msg.yaw = 1.2F;

  const auto ref = px4adrc::trajectory_reference_from_msg(msg, 7U);

  EXPECT_TRUE(ref.valid);
  EXPECT_EQ(ref.timestamp_us, 7U);
  EXPECT_TRUE(std::isnan(ref.position_ned.x()));
  EXPECT_DOUBLE_EQ(ref.position_ned.y(), 0.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.z(), -2.0);
  EXPECT_FLOAT_EQ(ref.yaw, 1.2F);
}

TEST(AdrcNodeLogic, ReferenceMessageConversionPreservesOptionalFields) {
  px4adrc::msg::FlatTrajectoryReference msg{};
  msg.position_ned.x = 1.0;
  msg.position_ned.y = 2.0;
  msg.position_ned.z = -3.0;
  msg.velocity_ned.x = std::numeric_limits<double>::infinity();
  msg.velocity_ned.y = 4.0;
  msg.velocity_ned.z = -5.0;
  msg.acceleration_ned.x = 0.1;
  msg.acceleration_ned.y = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration_ned.z = 0.3;
  msg.body_rates_frd.x = 0.4;
  msg.body_rates_frd.y = std::numeric_limits<double>::quiet_NaN();
  msg.body_rates_frd.z = 0.6;
  msg.body_torque_frd.x = std::numeric_limits<double>::infinity();
  msg.body_torque_frd.y = -0.2;
  msg.body_torque_frd.z = 0.3;
  msg.yaw = std::numeric_limits<float>::quiet_NaN();

  const auto ref = px4adrc::trajectory_reference_from_msg(msg, 42U);

  EXPECT_TRUE(ref.valid);
  EXPECT_EQ(ref.timestamp_us, 42U);
  EXPECT_DOUBLE_EQ(ref.position_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.y(), 2.0);
  EXPECT_DOUBLE_EQ(ref.position_ned.z(), -3.0);
  EXPECT_TRUE(std::isinf(ref.velocity_ned.x()));
  EXPECT_DOUBLE_EQ(ref.velocity_ned.y(), 4.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.z(), -5.0);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.x(), 0.1);
  EXPECT_TRUE(std::isnan(ref.acceleration_ned.y()));
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.z(), 0.3);
  EXPECT_DOUBLE_EQ(ref.body_rates_frd.x(), 0.4);
  EXPECT_TRUE(std::isnan(ref.body_rates_frd.y()));
  EXPECT_DOUBLE_EQ(ref.body_rates_frd.z(), 0.6);
  EXPECT_TRUE(std::isinf(ref.body_torque_frd.x()));
  EXPECT_DOUBLE_EQ(ref.body_torque_frd.y(), -0.2);
  EXPECT_DOUBLE_EQ(ref.body_torque_frd.z(), 0.3);
  EXPECT_TRUE(std::isnan(ref.yaw));
}

TEST(AdrcNodeLogic, RuntimeArtifactsExistAndMatchCurrentContract) {
  const auto package_root = package_root_from_test_file();
  const auto controller_config_path = package_root + "/config/px4adrc.yaml";
  const auto reference_config_path = package_root + "/config/flatness_reference.yaml";
  const auto launch_path = package_root + "/launch/px4adrc.launch.py";
  const auto readme_path = package_root + "/README.md";
  const auto script_path = package_root + "/scripts/flatness_reference_publisher.py";
  const auto node_source_path = package_root + "/src/adrc_node.cpp";

  ASSERT_FALSE(read_text_file(controller_config_path).empty());
  ASSERT_FALSE(read_text_file(reference_config_path).empty());
  ASSERT_FALSE(read_text_file(launch_path).empty());
  ASSERT_FALSE(read_text_file(readme_path).empty());
  ASSERT_FALSE(read_text_file(script_path).empty());
  ASSERT_FALSE(read_text_file(node_source_path).empty());

  const auto controller_config_text = read_text_file(controller_config_path);
  const auto reference_config_text = read_text_file(reference_config_path);
  const auto launch_text = read_text_file(launch_path);
  const auto readme_text = read_text_file(readme_path);
  const auto script_text = read_text_file(script_path);
  const auto node_source_text = read_text_file(node_source_path);

  EXPECT_NE(controller_config_text.find("reference:"), std::string::npos);
  EXPECT_NE(controller_config_text.find("start_tracking"), std::string::npos);
  EXPECT_EQ(controller_config_text.find("control_rate_hz"), std::string::npos);
  EXPECT_EQ(controller_config_text.find("velocity_diff_cutoff_hz"), std::string::npos);
  EXPECT_EQ(controller_config_text.find("body_rate_diff_cutoff_hz"), std::string::npos);
  EXPECT_EQ(controller_config_text.find("attitude_kp"), std::string::npos);
  EXPECT_NE(controller_config_text.find("attitude_td_r"), std::string::npos);
  EXPECT_NE(controller_config_text.find("eso_beta1"), std::string::npos);
  EXPECT_NE(reference_config_text.find("reference:"), std::string::npos);
  EXPECT_NE(reference_config_text.find("start_tracking"), std::string::npos);
  EXPECT_NE(launch_text.find("px4adrc.yaml"), std::string::npos);
  EXPECT_NE(launch_text.find("flatness_reference.yaml"), std::string::npos);
  EXPECT_NE(launch_text.find("flatness_reference_publisher.py"), std::string::npos);
  EXPECT_NE(launch_text.find("DeclareLaunchArgument"), std::string::npos);
  EXPECT_NE(launch_text.find("launch_reference"), std::string::npos);
  EXPECT_NE(readme_text.find("/fmu/out/vehicle_local_position"), std::string::npos);
  EXPECT_NE(readme_text.find("/fmu/out/vehicle_attitude"), std::string::npos);
  EXPECT_NE(readme_text.find("/fmu/out/vehicle_angular_velocity"), std::string::npos);
  EXPECT_NE(readme_text.find("desired body frame `FRD`"), std::string::npos);
  EXPECT_EQ(readme_text.find("/fmu/out/vehicle_odometry"), std::string::npos);
  EXPECT_NE(script_text.find("from px4adrc.msg import FlatTrajectoryReference"), std::string::npos);
  EXPECT_NE(script_text.find("start_tracking"), std::string::npos);
  EXPECT_NE(script_text.find("/fmu/out/vehicle_local_position"), std::string::npos);
  EXPECT_NE(script_text.find("/fmu/out/vehicle_attitude"), std::string::npos);
  EXPECT_EQ(script_text.find("/fmu/out/vehicle_odometry"), std::string::npos);
  EXPECT_NE(node_source_text.find("/fmu/out/vehicle_local_position"), std::string::npos);
  EXPECT_NE(node_source_text.find("/fmu/out/vehicle_attitude"), std::string::npos);
  EXPECT_NE(node_source_text.find("/fmu/out/vehicle_angular_velocity"), std::string::npos);
  EXPECT_NE(node_source_text.find("msg->xy_valid"), std::string::npos);
  EXPECT_NE(node_source_text.find("msg->z_valid"), std::string::npos);
  EXPECT_NE(node_source_text.find("msg->v_xy_valid"), std::string::npos);
  EXPECT_NE(node_source_text.find("msg->v_z_valid"), std::string::npos);
  EXPECT_NE(node_source_text.find("reset_controller_state"), std::string::npos);
  EXPECT_NE(node_source_text.find("if (!is_armed_)"), std::string::npos);
  EXPECT_NE(node_source_text.find("quiet_NaN"), std::string::npos);
  EXPECT_EQ(node_source_text.find("/fmu/out/vehicle_odometry"), std::string::npos);
  EXPECT_EQ(node_source_text.find("create_wall_timer"), std::string::npos);
  EXPECT_EQ(node_source_text.find("sanitize_scalar"), std::string::npos);
  EXPECT_EQ(node_source_text.find("sanitize_vec3"), std::string::npos);
}
