#ifndef BIROBOT_MANIPULATION__AUTO_PICK_BT_NODES_HPP_
#define BIROBOT_MANIPULATION__AUTO_PICK_BT_NODES_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <future>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

namespace birobot_manipulation
{

/**
 * @brief Cycle outcome summary shared across Behavior Tree nodes and action server.
 */
struct CycleResult
{
  bool success{false};
  int32_t objects_detected{0};
  int32_t objects_queued{0};
  int32_t objects_picked{0};
  int32_t objects_skipped{0};
  int32_t objects_low_confidence{0};
  std::string fault_code{""};
  std::string fault_detail{""};
  double cycle_duration_sec{0.0};
  bool is_current_skipped{false};
  std::chrono::steady_clock::time_point start_time{std::chrono::steady_clock::now()};
};

using FeedbackFn = std::function<void(
  const std::string & current_phase,
  int32_t queue_remaining,
  const std::string & current_object_id)>;

/**
 * @brief ScanObjectsNode: Detects objects via depth camera and populates pick queue.
 */
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
      BT::InputPort<double>("confidence_threshold", 0.70, "Confidence threshold between 0.0 and 1.0"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("pick_queue"),
      BT::OutputPort<std::vector<std::string>>("pick_queue_ids"),
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_poses_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_object_cloud_;

  std::chrono::steady_clock::time_point start_scan_time_;
  geometry_msgs::msg::PoseArray::SharedPtr last_pose_array_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_object_cloud_msg_;
  builtin_interfaces::msg::Time last_consumed_stamp_;
  std::mutex data_mutex_;
  double timeout_sec_{25.0};
};

/**
 * @brief RandomizeQueueNode: Shuffles the pick queue to avoid systematic bias.
 */
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

/**
 * @brief PopObjectNode: Pops the next object from the queue. Returns FAILURE when empty.
 */
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
      BT::OutputPort<std::string>("current_object_id"),
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief ArmPickMtcNode: Executes staged pick of the object using MoveGroup/MTC.
 */
class AutoArmPickMtcNode : public BT::StatefulActionNode
{
public:
  AutoArmPickMtcNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group"),
      BT::InputPort<std::string>("object_id", "auto_obj_1", "Object ID"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target grasp pose"),
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm2_attach_pub_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
  std::string current_object_id_;
};

/**
 * @brief SkipObjectNode: Records a planning/reach failure and skips to next object.
 */
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
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief VerifyGraspNode: Verifies grasp in MoveIt planning scene; triggers abort on grasp miss.
 */
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
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
};

/**
 * @brief ArmPlaceMtcNode: Places the held object at the fixed bin pose.
 */
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose", "Target place pose"),
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm2_detach_pub_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};

/**
 * @brief HomeArmNode: Returns arm to home named target.
 */
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
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};

/**
 * @brief AbortCycleNode: Marks cycle as aborted, sets fault information.
 */
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
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief ReportCycleNode: Outputs machine-readable and human-readable cycle summary.
 */
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
      BT::BidirectionalPort<std::shared_ptr<CycleResult>>("cycle_result")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Registers all auto-pick BT nodes with the given factory.
 */
void registerAutoPickNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

}  // namespace birobot_manipulation

#endif  // BIROBOT_MANIPULATION__AUTO_PICK_BT_NODES_HPP_
