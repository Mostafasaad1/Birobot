#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"

#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

class BtNodesTest : public ::testing::Test
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

TEST_F(BtNodesTest, TestPoseParsing)
{
  auto pose = birobot_manipulation::parsePoseString("dummy_pose_string");
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 0.10);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 0.05);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.15);
}

TEST_F(BtNodesTest, TestBehaviorTreeFactoryRegistrationAndXmlParsing)
{
  auto node = rclcpp::Node::make_shared("test_bt_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(birobot_manipulation::registerBirobotNodes(factory, node, tf_buffer));

  // Verify that the collaborative handover XML definition parses and builds correctly
  const std::string xml_text = R"(<root BTCPP_format="4">
    <BehaviorTree ID="CollaborativeHandoverTree">
      <Sequence name="CollaborativeHandoverSequence">
        <DetectObject target_pose="{target_pose}" object_id="{object_id}"/>
        <GripperControl gripper="arm1" action="open"/>
        <GripperControl gripper="arm2" action="open"/>
        <ArmPickMtc arm="arm_1" object_id="{object_id}" target_pose="{target_pose}"/>
        <Parallel success_count="2" failure_count="1">
          <MoveNamedPose arm="arm_1" named_pose="handover"/>
          <MoveNamedPose arm="arm_2" named_pose="handover"/>
        </Parallel>
        <GripperControl gripper="arm2" action="close"/>
        <TransferOwnership object_id="{object_id}" from_link="arm1_gripper_tcp" to_link="arm2_gripper_tcp"/>
        <GripperControl gripper="arm1" action="open"/>
        <CartesianRetract arm="arm_1" dx="-0.130" dy="0.0" dz="0.0"/>
        <MoveNamedPose arm="arm_1" named_pose="home"/>
        <MoveNamedPose arm="arm_2" named_pose="drop_off"/>
        <GripperControl gripper="arm2" action="open"/>
        <MoveNamedPose arm="arm_2" named_pose="home"/>
      </Sequence>
    </BehaviorTree>
  </root>)";

  BT::Tree tree;
  EXPECT_NO_THROW(tree = factory.createTreeFromText(xml_text));
  ASSERT_NE(tree.rootNode(), nullptr);
  EXPECT_EQ(tree.rootNode()->name(), "CollaborativeHandoverSequence");
}
