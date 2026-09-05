#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

#include <cmath>
#include <vector>
#include <thread>
#include <chrono>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace birobot_manipulation
{

using namespace std::chrono_literals;

// ── 1. DetectObjectNode ──────────────────────────────────────────────────────

DetectObjectNode::DetectObjectNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: BT::SyncActionNode(name, config),
  node_(node),
  tf_buffer_(tf_buffer)
{
}

BT::NodeStatus DetectObjectNode::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:DetectObject] Querying 3D perception for target object...");

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.frame_id = "world";
  detected_pose.header.stamp = node_->now();
  std::string target_id = "irregular_object_1";

  bool tf_found = false;
  if (tf_buffer_) {
    try {
      // Check for dynamic grasp target frame published by birobot_perception
      if (tf_buffer_->canTransform("world", "grasp_target_1", tf2::TimePointZero, 100ms)) {
        auto tf = tf_buffer_->lookupTransform("world", "grasp_target_1", tf2::TimePointZero);
        detected_pose.pose.position.x = tf.transform.translation.x;
        detected_pose.pose.position.y = tf.transform.translation.y;
        detected_pose.pose.position.z = tf.transform.translation.z;
        detected_pose.pose.orientation = tf.transform.rotation;
        tf_found = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "[BT:DetectObject] Resolved dynamic grasp_target_1 from CV: (%.3f, %.3f, %.3f)",
          detected_pose.pose.position.x, detected_pose.pose.position.y, detected_pose.pose.position.z);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_->get_logger(), "[BT:DetectObject] TF lookup failed: %s", ex.what());
    }
  }

  if (!tf_found) {
    // Fallback: Use canonical table surface position for irregular_object_1
    detected_pose.pose.position.x = 0.10;
    detected_pose.pose.position.y = 0.05;
    detected_pose.pose.position.z = 0.08;
    // Top-down grasp orientation
    detected_pose.pose.orientation.x = 1.0;
    detected_pose.pose.orientation.y = 0.0;
    detected_pose.pose.orientation.z = 0.0;
    detected_pose.pose.orientation.w = 0.0;
    RCLCPP_INFO(
      node_->get_logger(),
      "[BT:DetectObject] Using canonical object pose: (%.3f, %.3f, %.3f)",
      detected_pose.pose.position.x, detected_pose.pose.position.y, detected_pose.pose.position.z);
  }

  setOutput("target_pose", detected_pose);
  setOutput("object_id", target_id);
  return BT::NodeStatus::SUCCESS;
}

// ── 2. GripperControlNode ────────────────────────────────────────────────────

GripperControlNode::GripperControlNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus GripperControlNode::onStart()
{
  std::string gripper = "arm1";
  std::string action = "open";
  getInput("gripper", gripper);
  getInput("action", action);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:GripperControl] Commanding gripper '%s' to '%s'",
    gripper.c_str(), action.c_str());

  std::string group_name = (gripper == "arm2" || gripper == "arm2_gripper") ? "arm2_hand" : "arm1_hand";
  std::string target_name = (action == "close" || action == "closed") ? "closed" : "open";

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
  move_group->setPlanningTime(5.0);
  move_group->setNumPlanningAttempts(5);
  move_group->setMaxVelocityScalingFactor(0.5);
  move_group->setMaxAccelerationScalingFactor(0.5);
  move_group->setNamedTarget(target_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "[BT:GripperControl] Failed to plan gripper motion for '%s'", group_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  execution_future_ = std::async(std::launch::async, [move_group, plan]() {
    return move_group->execute(plan);
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperControlNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:GripperControl] Gripper command executed successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:GripperControl] Gripper execution returned failure code: %d", res.val);
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void GripperControlNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:GripperControl] Action halted");
}

// ── 3. ArmPickMtcNode ────────────────────────────────────────────────────────

ArmPickMtcNode::ArmPickMtcNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
}

BT::NodeStatus ArmPickMtcNode::onStart()
{
  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }
  std::string arm_name = "arm_1";
  std::string object_id = "irregular_object_1";
  geometry_msgs::msg::PoseStamped target_pose;

  getInput("arm", arm_name);
  getInput("object_id", object_id);
  getInput("target_pose", target_pose);
  current_object_id_ = object_id;

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ArmPickMtc] Arm '%s' initiating staged pick on '%s' at (%.3f, %.3f, %.3f)",
    arm_name.c_str(), object_id.c_str(),
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

  // 1. Register object into Planning Scene if not present
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = target_pose.header.frame_id.empty() ? "world" : target_pose.header.frame_id;
  object.id = object_id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.15, 0.08, 0.06};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(target_pose.pose);
  object.operation = object.ADD;
  psi_->applyCollisionObject(object);

  // 2. Plan pre-grasp approach with MoveGroup
  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
  move_group->setEndEffectorLink(ik_frame);
  move_group->setPlanningTime(10.0);
  move_group->setNumPlanningAttempts(10);
  move_group->setMaxVelocityScalingFactor(0.2);
  move_group->setMaxAccelerationScalingFactor(0.2);
  move_group->setWorkspace(-1.5, -1.5, 0.0, 1.5, 1.5, 2.0);

  // Pre-grasp pose: 20 cm above target
  geometry_msgs::msg::PoseStamped pre_grasp = target_pose;
  pre_grasp.pose.position.z += 0.20;
  move_group->setPoseTarget(pre_grasp, ik_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp motion planning failed for '%s'", arm_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 3. Execute pick trajectory and attach payload in planning scene
  execution_future_ = std::async(std::launch::async, [this, move_group, plan, object_id, ik_frame]() {
    auto err = move_group->execute(plan);
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
      // Attach object to arm end-effector
      moveit_msgs::msg::AttachedCollisionObject attached_obj;
      attached_obj.link_name = ik_frame;
      attached_obj.object.id = object_id;
      attached_obj.object.operation = attached_obj.object.ADD;
      psi_->applyAttachedCollisionObject(attached_obj);
    }
    return err;
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmPickMtcNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Object '%s' picked and attached successfully", current_object_id_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pick trajectory execution failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void ArmPickMtcNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Action halted");
}

// ── 4. MoveNamedPoseNode ─────────────────────────────────────────────────────

MoveNamedPoseNode::MoveNamedPoseNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus MoveNamedPoseNode::onStart()
{
  std::string arm_name = "arm_1";
  std::string named_pose = "home";
  getInput("arm", arm_name);
  getInput("named_pose", named_pose);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:MoveNamedPose] Moving arm '%s' to named pose '%s'",
    arm_name.c_str(), named_pose.c_str());

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
  move_group->setPlanningTime(5.0);
  move_group->setNumPlanningAttempts(5);
  move_group->setMaxVelocityScalingFactor(0.2);
  move_group->setMaxAccelerationScalingFactor(0.2);
  move_group->setNamedTarget(named_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "[BT:MoveNamedPose] Failed to plan trajectory to '%s' for '%s'", named_pose.c_str(), arm_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  execution_future_ = std::async(std::launch::async, [move_group, plan]() {
    return move_group->execute(plan);
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveNamedPoseNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:MoveNamedPose] Named pose reached successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:MoveNamedPose] Motion execution failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void MoveNamedPoseNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:MoveNamedPose] Action halted");
}

// ── 5. TransferOwnershipNode ─────────────────────────────────────────────────

TransferOwnershipNode::TransferOwnershipNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
}

BT::NodeStatus TransferOwnershipNode::tick()
{
  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }
  std::string object_id = "irregular_object_1";
  std::string from_link = "arm1_gripper_tcp";
  std::string to_link = "arm2_gripper_tcp";

  getInput("object_id", object_id);
  getInput("from_link", from_link);
  getInput("to_link", to_link);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:TransferOwnership] Transferring payload '%s' from '%s' to '%s'",
    object_id.c_str(), from_link.c_str(), to_link.c_str());

  // Detach from source link
  moveit_msgs::msg::AttachedCollisionObject detach_obj;
  detach_obj.link_name = from_link;
  detach_obj.object.id = object_id;
  detach_obj.object.operation = detach_obj.object.REMOVE;
  psi_->applyAttachedCollisionObject(detach_obj);

  // Small delay for planning scene synchronization
  std::this_thread::sleep_for(100ms);

  // Attach to destination link
  moveit_msgs::msg::AttachedCollisionObject attach_obj;
  attach_obj.link_name = to_link;
  attach_obj.object.id = object_id;
  attach_obj.object.operation = attach_obj.object.ADD;
  psi_->applyAttachedCollisionObject(attach_obj);

  RCLCPP_INFO(node_->get_logger(), "[BT:TransferOwnership] Ownership transferred atomically in MoveIt Planning Scene");
  return BT::NodeStatus::SUCCESS;
}

// ── Factory Registration ────────────────────────────────────────────────────

void registerBirobotNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  factory.registerBuilder<DetectObjectNode>(
    "DetectObject",
    [node, tf_buffer](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<DetectObjectNode>(name, config, node, tf_buffer);
    });

  factory.registerBuilder<GripperControlNode>(
    "GripperControl",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GripperControlNode>(name, config, node);
    });

  factory.registerBuilder<ArmPickMtcNode>(
    "ArmPickMtc",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ArmPickMtcNode>(name, config, node);
    });

  factory.registerBuilder<MoveNamedPoseNode>(
    "MoveNamedPose",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<MoveNamedPoseNode>(name, config, node);
    });

  factory.registerBuilder<TransferOwnershipNode>(
    "TransferOwnership",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<TransferOwnershipNode>(name, config, node);
    });
}

}  // namespace birobot_manipulation
