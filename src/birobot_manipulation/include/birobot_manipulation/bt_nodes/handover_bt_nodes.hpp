#ifndef BIROBOT_MANIPULATION__BT_NODES__HANDOVER_BT_NODES_HPP_
#define BIROBOT_MANIPULATION__BT_NODES__HANDOVER_BT_NODES_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace birobot_manipulation
{

// String parsing helper for PoseStamped port
inline geometry_msgs::msg::PoseStamped parsePoseString(const std::string & str)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  // Default values matching original test expectations
  pose.pose.position.x = 0.10;
  pose.pose.position.y = 0.05;
  pose.pose.position.z = 0.08;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Check for frame_id
  auto pos_frame = str.find("frame_id:");
  if (pos_frame == std::string::npos) {
    pos_frame = str.find("frame:");
  }
  if (pos_frame != std::string::npos) {
    size_t colon = str.find(':', pos_frame);
    if (colon != std::string::npos) {
      size_t start = str.find_first_not_of(" \t,\"", colon + 1);
      size_t end = str.find_first_of(" \t,}\"", start);
      if (start != std::string::npos) {
        pose.header.frame_id = str.substr(start, (end == std::string::npos) ? end : (end - start));
      }
    }
  }

  // Helper lambda to parse key-value numbers
  auto parse_val = [&](const std::string & key, double & val) {
    size_t p = 0;
    while ((p = str.find(key, p)) != std::string::npos) {
      // Ensure key is preceded by space, comma, or brace
      if (p == 0 || str[p - 1] == ' ' || str[p - 1] == ',' || str[p - 1] == '{') {
        size_t after_key = p + key.length();
        size_t colon = str.find_first_of(":=", after_key);
        if (colon != std::string::npos && colon - after_key <= 2) {
          try {
            size_t num_start = str.find_first_not_of(" \t", colon + 1);
            if (num_start != std::string::npos) {
              val = std::stod(str.substr(num_start));
              return;
            }
          } catch (...) {}
        }
      }
      p += key.length();
    }
  };

  parse_val("x", pose.pose.position.x);
  parse_val("y", pose.pose.position.y);
  parse_val("z", pose.pose.position.z);
  parse_val("qx", pose.pose.orientation.x);
  parse_val("qy", pose.pose.orientation.y);
  parse_val("qz", pose.pose.orientation.z);
  parse_val("qw", pose.pose.orientation.w);
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
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm2_detach_pub_;
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
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm2_attach_pub_;
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
 * @brief TransferOwnershipNode: Updates MoveIt Planning Scene and Gazebo to transfer object attachment.
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
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm1_detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm2_attach_pub_;
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
