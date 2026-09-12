#ifndef BIROBOT_MANIPULATION__BT_NODES__AUTO_PICK_BT_NODES_HPP_
#define BIROBOT_MANIPULATION__BT_NODES__AUTO_PICK_BT_NODES_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/empty.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace birobot_manipulation
{

// ── CycleResult Data Structure ───────────────────────────────────────────────
struct CycleResult {
  bool success = true;
  int32_t objects_detected = 0;
  int32_t objects_queued = 0;
  int32_t objects_picked = 0;
  int32_t objects_skipped = 0;
  int32_t objects_low_confidence = 0;
  std::string fault_code = "";
  std::string fault_detail = "";
  rclcpp::Time start_time;
  
  CycleResult() {}
};

// ── 1. ScanObjectsNode ───────────────────────────────────────────────────────
class ScanObjectsNode : public BT::StatefulActionNode
{
public:
  ScanObjectsNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("confidence_threshold", 0.70, "Minimum confidence (0.0-1.0)"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("pick_queue"),
      BT::OutputPort<std::vector<std::string>>("pick_queue_ids")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

// ── 2. RandomizeQueueNode ────────────────────────────────────────────────────
class RandomizeQueueNode : public BT::SyncActionNode
{
public:
  RandomizeQueueNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("pick_queue"),
      BT::BidirectionalPort<std::vector<std::string>>("pick_queue_ids")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ── 3. PopObjectNode ─────────────────────────────────────────────────────────
class PopObjectNode : public BT::SyncActionNode
{
public:
  PopObjectNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("pick_queue"),
      BT::BidirectionalPort<std::vector<std::string>>("pick_queue_ids"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_object_pose"),
      BT::OutputPort<std::string>("current_object_id")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ── 4. VerifyGraspNode ───────────────────────────────────────────────────────
class VerifyGraspNode : public BT::SyncActionNode
{
public:
  VerifyGraspNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_id"),
      BT::BidirectionalPort<CycleResult>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
};

// ── 5. SkipObjectNode ────────────────────────────────────────────────────────
class SkipObjectNode : public BT::SyncActionNode
{
public:
  SkipObjectNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_id"),
      BT::BidirectionalPort<CycleResult>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ── 6. ArmPlaceMtcNode ───────────────────────────────────────────────────────
class ArmPlaceMtcNode : public BT::StatefulActionNode
{
public:
  ArmPlaceMtcNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose", "Fixed bin placement pose")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_detach_pub_;
  std::string current_arm_;
};

// ── 7. HomeArmNode ───────────────────────────────────────────────────────────
class HomeArmNode : public BT::StatefulActionNode
{
public:
  HomeArmNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group to home")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};

// ── 8. AbortCycleNode ────────────────────────────────────────────────────────
class AbortCycleNode : public BT::SyncActionNode
{
public:
  AbortCycleNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("fault_code"),
      BT::InputPort<std::string>("fault_detail"),
      BT::BidirectionalPort<CycleResult>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ── 9. ReportCycleNode ───────────────────────────────────────────────────────
class ReportCycleNode : public BT::SyncActionNode
{
public:
  ReportCycleNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<CycleResult>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// ── Factory Registration ─────────────────────────────────────────────────────
void registerAutoPickNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

}  // namespace birobot_manipulation

#endif  // BIROBOT_MANIPULATION__BT_NODES__AUTO_PICK_BT_NODES_HPP_
