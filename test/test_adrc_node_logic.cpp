#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#include "px4adrc/adrc_node.hpp"

namespace
{

std::string package_root_from_test_file()
{
  const std::string file = __FILE__;
  const std::string marker = "/test/test_adrc_node_logic.cpp";
  const auto marker_pos = file.rfind(marker);
  return marker_pos == std::string::npos ? std::string(".") : file.substr(0, marker_pos);
}

std::string read_text_file(const std::string & path)
{
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

}  // namespace

TEST(AdrcNodeLogic, StateNamesRemainStable)
{
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::WAIT_FOR_STATE), std::string("WAIT_FOR_STATE"));
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::WAIT_FOR_MANUAL_ARM), std::string("WAIT_FOR_MANUAL_ARM"));
  EXPECT_EQ(px4adrc::to_string(px4adrc::MissionState::TRACKING), std::string("TRACKING"));
}

TEST(AdrcNodeLogic, OffboardWithoutArmWaitsForManualArm)
{
  EXPECT_EQ(
    px4adrc::next_mission_state(
      px4adrc::MissionState::REQUEST_OFFBOARD,
      true,
      false,
      false,
      false),
    px4adrc::MissionState::WAIT_FOR_MANUAL_ARM);
}

TEST(AdrcNodeLogic, ReferenceTimeoutReturnsToHold)
{
  EXPECT_EQ(
    px4adrc::next_mission_state(
      px4adrc::MissionState::TRACKING,
      true,
      true,
      false,
      false),
    px4adrc::MissionState::HOLD);
}

TEST(AdrcNodeLogic, HoldAfterTakeoffRequestsStartTrackingOnce)
{
  EXPECT_TRUE(px4adrc::should_publish_start_tracking_signal(
    px4adrc::MissionState::HOLD, true, false));
  EXPECT_FALSE(px4adrc::should_publish_start_tracking_signal(
    px4adrc::MissionState::HOLD, true, true));
  EXPECT_FALSE(px4adrc::should_publish_start_tracking_signal(
    px4adrc::MissionState::TAKEOFF, true, false));
}

TEST(AdrcNodeLogic, ReferenceMessageRejectsNonFinitePosition)
{
  px4adrc::msg::FlatTrajectoryReference msg{};
  msg.position_ned.x = std::numeric_limits<double>::quiet_NaN();
  msg.position_ned.y = 0.0;
  msg.position_ned.z = -2.0;

  EXPECT_FALSE(px4adrc::reference_message_has_valid_position(msg));
}

TEST(AdrcNodeLogic, ReferenceMessageConversionSanitizesOptionalFields)
{
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
  EXPECT_DOUBLE_EQ(ref.velocity_ned.x(), 0.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.y(), 4.0);
  EXPECT_DOUBLE_EQ(ref.velocity_ned.z(), -5.0);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.x(), 0.1);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.y(), 0.0);
  EXPECT_DOUBLE_EQ(ref.acceleration_ned.z(), 0.3);
  EXPECT_DOUBLE_EQ(ref.body_rates_frd.x(), 0.4);
  EXPECT_DOUBLE_EQ(ref.body_rates_frd.y(), 0.0);
  EXPECT_DOUBLE_EQ(ref.body_rates_frd.z(), 0.6);
  EXPECT_DOUBLE_EQ(ref.body_torque_frd.x(), 0.0);
  EXPECT_DOUBLE_EQ(ref.body_torque_frd.y(), -0.2);
  EXPECT_DOUBLE_EQ(ref.body_torque_frd.z(), 0.3);
  EXPECT_DOUBLE_EQ(ref.yaw, 0.0);
}

TEST(AdrcNodeLogic, RuntimeArtifactsExistAndMatchCurrentContract)
{
  const auto package_root = package_root_from_test_file();
  const auto controller_config_path = package_root + "/config/px4adrc.yaml";
  const auto reference_config_path = package_root + "/config/flatness_reference.yaml";
  const auto launch_path = package_root + "/launch/px4adrc.launch.py";
  const auto script_path = package_root + "/scripts/flatness_reference_publisher.py";

  ASSERT_FALSE(read_text_file(controller_config_path).empty());
  ASSERT_FALSE(read_text_file(reference_config_path).empty());
  ASSERT_FALSE(read_text_file(launch_path).empty());
  ASSERT_FALSE(read_text_file(script_path).empty());

  const auto controller_config_text = read_text_file(controller_config_path);
  const auto reference_config_text = read_text_file(reference_config_path);
  const auto launch_text = read_text_file(launch_path);
  const auto script_text = read_text_file(script_path);

  EXPECT_NE(controller_config_text.find("reference:"), std::string::npos);
  EXPECT_NE(controller_config_text.find("start_tracking"), std::string::npos);
  EXPECT_NE(controller_config_text.find("attitude_kp"), std::string::npos);
  EXPECT_NE(controller_config_text.find("td_r"), std::string::npos);
  EXPECT_NE(controller_config_text.find("eso_beta1"), std::string::npos);
  EXPECT_NE(reference_config_text.find("reference:"), std::string::npos);
  EXPECT_NE(reference_config_text.find("start_tracking"), std::string::npos);
  EXPECT_NE(launch_text.find("px4adrc.yaml"), std::string::npos);
  EXPECT_NE(launch_text.find("flatness_reference.yaml"), std::string::npos);
  EXPECT_NE(launch_text.find("flatness_reference_publisher.py"), std::string::npos);
  EXPECT_NE(script_text.find("from px4adrc.msg import FlatTrajectoryReference"), std::string::npos);
  EXPECT_NE(script_text.find("start_tracking"), std::string::npos);
}
