#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "tf2_ros/buffer.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"
#include "birobot_manipulation/auto_pick_bt_nodes.hpp"

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
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.08);
}

TEST_F(BtNodesTest, TestAutoPickPoseParsing)
{
  std::string xml_pose_str = "{x: 0.3, y: 0.3, z: 0.2, frame_id: world, qw: 1.0}";
  auto pose = birobot_manipulation::parsePoseString(xml_pose_str);
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 0.3);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 0.3);
  EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.2);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
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

TEST_F(BtNodesTest, TestAutoPickTreeRegistrationAndXmlParsing)
{
  auto node = rclcpp::Node::make_shared("test_auto_pick_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(birobot_manipulation::registerBirobotNodes(factory, node, tf_buffer));
  EXPECT_NO_THROW(birobot_manipulation::registerAutoPickNodes(factory, node, tf_buffer));

  std::string xml_file = ament_index_cpp::get_package_share_directory("birobot_manipulation") +
    "/config/bt_trees/auto_pick_place.xml";

  BT::Tree tree;
  EXPECT_NO_THROW(tree = factory.createTreeFromFile(xml_file));
  ASSERT_NE(tree.rootNode(), nullptr);
  EXPECT_EQ(tree.rootNode()->name(), "AutoPickPlaceMain");

  // Also verify FaultHandlerTree can be created
  BT::Tree fault_tree;
  EXPECT_NO_THROW(fault_tree = factory.createTree("FaultHandlerTree", tree.rootBlackboard()));
  ASSERT_NE(fault_tree.rootNode(), nullptr);
  EXPECT_EQ(fault_tree.rootNode()->name(), "FaultHandler");
}

TEST_F(BtNodesTest, TestRandomizeQueueNode)
{
  auto node = rclcpp::Node::make_shared("test_randomize_node");
  BT::NodeConfig config;
  auto bb = BT::Blackboard::create();
  config.blackboard = bb;

  std::vector<geometry_msgs::msg::PoseStamped> queue(3);
  queue[0].pose.position.x = 1.0;
  queue[1].pose.position.x = 2.0;
  queue[2].pose.position.x = 3.0;
  std::vector<std::string> ids = {"obj1", "obj2", "obj3"};

  bb->set("pick_queue", queue);
  bb->set("pick_queue_ids", ids);

  config.input_ports["pick_queue"] = "{pick_queue}";
  config.input_ports["pick_queue_ids"] = "{pick_queue_ids}";
  config.output_ports["pick_queue"] = "{pick_queue}";
  config.output_ports["pick_queue_ids"] = "{pick_queue_ids}";

  birobot_manipulation::RandomizeQueueNode rand_node("RandomizeQueue", config, node);
  EXPECT_EQ(rand_node.executeTick(), BT::NodeStatus::SUCCESS);

  std::vector<geometry_msgs::msg::PoseStamped> result_queue;
  std::vector<std::string> result_ids;
  bb->get("pick_queue", result_queue);
  bb->get("pick_queue_ids", result_ids);

  EXPECT_EQ(result_queue.size(), 3u);
  EXPECT_EQ(result_ids.size(), 3u);
}

TEST_F(BtNodesTest, TestPopObjectNode)
{
  auto node = rclcpp::Node::make_shared("test_pop_node");
  BT::NodeConfig config;
  auto bb = BT::Blackboard::create();
  config.blackboard = bb;

  std::vector<geometry_msgs::msg::PoseStamped> queue(1);
  queue[0].pose.position.x = 42.0;
  std::vector<std::string> ids = {"target_obj"};

  auto cycle_res = std::make_shared<birobot_manipulation::CycleResult>();
  cycle_res->objects_queued = 1;

  bb->set("pick_queue", queue);
  bb->set("pick_queue_ids", ids);
  bb->set("cycle_result", cycle_res);

  config.input_ports["pick_queue"] = "{pick_queue}";
  config.input_ports["pick_queue_ids"] = "{pick_queue_ids}";
  config.input_ports["cycle_result"] = "{cycle_result}";
  config.output_ports["pick_queue"] = "{pick_queue}";
  config.output_ports["pick_queue_ids"] = "{pick_queue_ids}";
  config.output_ports["current_object_pose"] = "{current_object_pose}";
  config.output_ports["current_object_id"] = "{current_object_id}";
  config.output_ports["cycle_result"] = "{cycle_result}";

  birobot_manipulation::PopObjectNode pop_node("PopObject", config, node);

  // First pop should succeed
  EXPECT_EQ(pop_node.executeTick(), BT::NodeStatus::SUCCESS);
  std::string cur_id;
  bb->get("current_object_id", cur_id);
  EXPECT_EQ(cur_id, "target_obj");

  // Second pop with empty queue should return FAILURE
  EXPECT_EQ(pop_node.executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, TestSkipObjectNode)
{
  auto node = rclcpp::Node::make_shared("test_skip_node");
  BT::NodeConfig config;
  auto bb = BT::Blackboard::create();
  config.blackboard = bb;

  auto cycle_res = std::make_shared<birobot_manipulation::CycleResult>();
  bb->set("cycle_result", cycle_res);
  bb->set("object_id", std::string("unreachable_obj"));

  config.input_ports["object_id"] = "{object_id}";
  config.input_ports["cycle_result"] = "{cycle_result}";
  config.output_ports["cycle_result"] = "{cycle_result}";

  birobot_manipulation::SkipObjectNode skip_node("SkipObject", config, node);
  EXPECT_EQ(skip_node.executeTick(), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(cycle_res->is_current_skipped);
  EXPECT_EQ(cycle_res->objects_skipped, 1);
}

TEST_F(BtNodesTest, TestAbortCycleNode)
{
  auto node = rclcpp::Node::make_shared("test_abort_node");
  BT::NodeConfig config;
  auto bb = BT::Blackboard::create();
  config.blackboard = bb;

  auto cycle_res = std::make_shared<birobot_manipulation::CycleResult>();
  bb->set("cycle_result", cycle_res);
  bb->set("fault_code", std::string("CAMERA_FAULT"));
  bb->set("fault_detail", std::string("No point cloud"));

  config.input_ports["fault_code"] = "{fault_code}";
  config.input_ports["fault_detail"] = "{fault_detail}";
  config.input_ports["cycle_result"] = "{cycle_result}";
  config.output_ports["cycle_result"] = "{cycle_result}";

  birobot_manipulation::AbortCycleNode abort_node("AbortCycle", config, node);
  EXPECT_EQ(abort_node.executeTick(), BT::NodeStatus::SUCCESS);

  EXPECT_FALSE(cycle_res->success);
  EXPECT_EQ(cycle_res->fault_code, "CAMERA_FAULT");
  EXPECT_EQ(cycle_res->fault_detail, "No point cloud");
}
