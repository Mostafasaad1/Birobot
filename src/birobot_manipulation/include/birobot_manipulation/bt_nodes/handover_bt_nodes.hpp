#ifndef BIROBOT_MANIPULATION__BT_NODES__HANDOVER_BT_NODES_HPP_
#define BIROBOT_MANIPULATION__BT_NODES__HANDOVER_BT_NODES_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace birobot_manipulation
{

// String parsing helper for PoseStamped port
inline geometry_msgs::msg::PoseStamped parsePoseString(const std::string & /*str*/)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  // Default values
  pose.pose.position.x = 0.10;
  pose.pose.position.y = 0.05;
  pose.pose.position.z = 0.08;
  pose.pose.orientation.x = 1.0;
  pose.pose.orientation.w = 0.0;
  return pose;
}

/**
 * @brief DetectObjectNode: Queries perception TF frames or provides table object pose.
 */
class DetectObjectNode : public BT::SyncActionNode
{
public:
  DetectObjectNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      BT::OutputPort<std::string>("object_id")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

/**
 * @brief GripperControlNode: Opens or closes the specified arm gripper.
 */
class GripperControlNode : public BT::StatefulActionNode
{
public:
  GripperControlNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("gripper", "arm1", "Gripper to control (arm1 or arm2)"),
      BT::InputPort<std::string>("action", "open", "Action: open or close")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm1_hand_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm2_hand_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};

/**
 * @brief ArmPickMtcNode: Executes staged pick of the object using MoveGroup/MTC.
 */
class ArmPickMtcNode : public BT::StatefulActionNode
{
public:
  ArmPickMtcNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group"),
      BT::InputPort<std::string>("object_id", "irregular_object_1", "Object ID"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target grasp pose")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm1_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm2_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
  std::string current_object_id_;
};

/**
 * @brief MoveNamedPoseNode: Moves an arm group to an SRDF named pose (e.g. handover, drop_off, home).
 */
class MoveNamedPoseNode : public BT::StatefulActionNode
{
public:
  MoveNamedPoseNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("arm", "arm_1", "Arm planning group (arm_1, arm_2, dual_arms)"),
      BT::InputPort<std::string>("named_pose", "home", "Target named pose")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm1_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm2_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_dual_;
  std::future<moveit::core::MoveItErrorCode> execution_future_;
};

/**
 * @brief TransferOwnershipNode: Updates MoveIt Planning Scene to transfer object attachment.
 */
class TransferOwnershipNode : public BT::SyncActionNode
{
public:
  TransferOwnershipNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_id", "irregular_object_1", "Target object ID"),
      BT::InputPort<std::string>("from_link", "arm1_gripper_tcp", "Source link to detach"),
      BT::InputPort<std::string>("to_link", "arm2_gripper_tcp", "Destination link to attach")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
};

/**
 * @brief Registers all custom Birobot BT nodes with the given factory.
 */
void registerBirobotNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

}  // namespace birobot_manipulation

namespace BT
{
template <>
inline geometry_msgs::msg::PoseStamped convertFromString(StringView str)
{
  return birobot_manipulation::parsePoseString(std::string(str));
}
}  // namespace BT

#endif  // BIROBOT_MANIPULATION__BT_NODES__HANDOVER_BT_NODES_HPP_
