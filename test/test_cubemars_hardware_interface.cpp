// Taken from https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/test/mock_components/test_generic_system.cpp

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.1);  // 0.1 seconds for easier math
const auto COMPARE_DELTA = 0.0001;
}  // namespace

class TestGenericSystem : public ::testing::Test
{
// public:
//   void test_generic_system_with_mimic_joint(std::string & urdf, const std::string & component_name);
//   void test_generic_system_with_mock_sensor_commands(
//     std::string & urdf, const std::string & component_name);
//   void test_generic_system_with_mock_gpio_commands(
//     std::string & urdf, const std::string & component_name);

protected:
  void SetUp() override
  {
    hardware_system_2dof_ =
      R"(
        <ros2_control name="MockHardwareSystem" type="system">
          <hardware>
            <plugin>cubemars_hardware_interface/CubemarsHardwareInterface</plugin>
            <param name="can_interface">vcan0</param>
          </hardware>

          <joint name="joint1">
            <param name="can_id">14</param>
            <param name="kp">5.0</param>
            <param name="kd">5.0</param>
            <param name="kp_min">0</param>
            <param name="kp_max">500</param>
            <param name="kd_min">0</param>
            <param name="kd_max">5</param>
            <command_interface name="position">
              <param name="min">-12.5</param>
              <param name="max">12.5</param>
            </command_interface>
            <command_interface name="velocity">
              <param name="min">-25.64</param>
              <param name="max">25.64</param>
            </command_interface>
            <command_interface name="effort">
              <param name="min">-18.0</param>
              <param name="max">18.0</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
            <state_interface name="temperature"/>
          </joint>
          <joint name="joint2">
            <param name="can_id">15</param>
            <param name="kp">5.0</param>
            <param name="kd">5.0</param>
            <param name="kp_min">0</param>
            <param name="kp_max">500</param>
            <param name="kd_min">0</param>
            <param name="kd_max">5</param>
            <command_interface name="position">
              <param name="min">-12.5</param>
              <param name="max">12.5</param>
            </command_interface>
            <command_interface name="velocity">
              <param name="min">-25.64</param>
              <param name="max">25.64</param>
            </command_interface>
            <command_interface name="effort">
              <param name="min">-18.0</param>
              <param name="max">18.0</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
            <state_interface name="temperature"/>
          </joint>
        </ros2_control>
      )";
  }

  std::string hardware_system_2dof_;
  rclcpp::Node node_ = rclcpp::Node("TestGenericSystem");
};

// Forward declaration
namespace hardware_interface
{
class ResourceStorage;
}

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestGenericSystem;

  // FRIEND_TEST(TestGenericSystem, load_generic_system_2dof);
  FRIEND_TEST(TestGenericSystem, generic_system_2dof_symetric_interfaces);
  // FRIEND_TEST(TestGenericSystem, generic_system_2dof_other_interfaces);
  // FRIEND_TEST(TestGenericSystem, generic_system_2dof_sensor);
  // FRIEND_TEST(TestGenericSystem, generic_system_2dof_sensor_mock_command);
  // FRIEND_TEST(TestGenericSystem, generic_system_2dof_sensor_mock_command_True);
  // FRIEND_TEST(TestGenericSystem, hardware_system_2dof_with_mimic_joint);
  // FRIEND_TEST(TestGenericSystem, valid_urdf_ros2_control_system_robot_with_gpio);
  // FRIEND_TEST(TestGenericSystem, valid_urdf_ros2_control_system_robot_with_gpio_mock_command);
  // FRIEND_TEST(TestGenericSystem, valid_urdf_ros2_control_system_robot_with_gpio_mock_command_True);

  explicit TestableResourceManager(rclcpp::Node & node)
  : hardware_interface::ResourceManager(
      node.get_node_clock_interface(), node.get_node_logging_interface())
  {
  }

  explicit TestableResourceManager(
    rclcpp::Node & node, const std::string & urdf, bool activate_all = false)
  : hardware_interface::ResourceManager(
      urdf, node.get_node_clock_interface(), node.get_node_logging_interface(), activate_all, 100)
  {
  }
};

void set_components_state(
  TestableResourceManager & rm, const std::vector<std::string> & components, const uint8_t state_id,
  const std::string & state_name)
{
  for (const auto & component : components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
                              TestableResourceManager & rm,
                              const std::vector<std::string> & components = {"GenericSystem2dof"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto activate_components = [](
                             TestableResourceManager & rm,
                             const std::vector<std::string> & components = {"GenericSystem2dof"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components = [](
                               TestableResourceManager & rm,
                               const std::vector<std::string> & components = {"GenericSystem2dof"})
{
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};



TEST_F(TestGenericSystem, load_generic_system_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(node_, urdf));
}

TEST_F(TestGenericSystem, generic_system_2dof_symetric_interfaces)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
              ros2_control_test_assets::urdf_tail;
  TestableResourceManager rm(node_, urdf);
  // Activate components to get all interfaces available
  activate_components(rm, {"MockHardwareSystem"});

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(8u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint2/position"));

  ASSERT_EQ(6u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("joint2/position"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1p_s = rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j2p_s = rm.claim_state_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j1p_c = rm.claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2p_c = rm.claim_command_interface("joint2/position");

  // ASSERT_EQ(1.57, j1p_s.get_value());
  // ASSERT_EQ(0.7854, j2p_s.get_value());
  // ASSERT_TRUE(std::isnan(j1p_c.get_value()));
  // ASSERT_TRUE(std::isnan(j2p_c.get_value()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}