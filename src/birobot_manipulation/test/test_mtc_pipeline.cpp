#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "birobot_manipulation/mtc_pick_place_node.hpp"

class MtcPipelineTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(MtcPipelineTest, TestNodeInitializationAndLifecycle)
{
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<birobot_manipulation::MtcPickPlaceNode>(options);
  EXPECT_NE(node, nullptr);
  EXPECT_EQ(node->getExecutionState(), birobot_manipulation::TaskExecutionState::IDLE);

  // Test configure lifecycle transition
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Test activate lifecycle transition
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Test deactivate transition
  state = node->deactivate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Test cleanup transition
  state = node->cleanup();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
