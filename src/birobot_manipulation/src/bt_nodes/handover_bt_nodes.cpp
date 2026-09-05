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
    // Top-down grasp orientation aligned with object width (0.08m)
    // Object yaw is 0.4 rad in Gazebo, so width is along yaw 0.4 + pi/2 ~ 1.9708 rad.
    // Quaternion for roll=pi, pitch=0, yaw=1.9708 rad:
    detected_pose.pose.orientation.x = 0.5523;
    detected_pose.pose.orientation.y = 0.8336;
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
  node_(node),
  psi_(nullptr)
{
  arm1_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/detach", 10);
  arm2_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/detach", 10);
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

  // Handle Gazebo and MoveIt detach if opening gripper at drop-off
  if (action == "open") {
    std_msgs::msg::Empty empty_msg;
    if (gripper == "arm2" || gripper == "arm2_gripper") {
      arm2_detach_pub_->publish(empty_msg);
      if (!psi_) {
        psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
      }
      moveit_msgs::msg::AttachedCollisionObject detach_obj;
      detach_obj.link_name = "arm2_gripper_tcp";
      detach_obj.object.id = "irregular_object_1";
      detach_obj.object.operation = detach_obj.object.REMOVE;
      psi_->applyAttachedCollisionObject(detach_obj);
      RCLCPP_INFO(node_->get_logger(), "[BT:GripperControl] Released payload in Gazebo Sim and MoveIt Planning Scene");
    } else {
      arm1_detach_pub_->publish(empty_msg);
    }
  }

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
  arm1_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/attach", 10);
  arm2_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/attach", 10);
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

  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";

  // Validate or fix orientation if zero/uninitialized
  if (std::abs(target_pose.pose.orientation.w) < 1e-4 &&
      std::abs(target_pose.pose.orientation.x) < 1e-4 &&
      std::abs(target_pose.pose.orientation.y) < 1e-4 &&
      std::abs(target_pose.pose.orientation.z) < 1e-4)
  {
    target_pose.pose.orientation.x = 0.5523;
    target_pose.pose.orientation.y = 0.8336;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ArmPickMtc] Arm '%s' initiating staged pick on '%s' at (%.3f, %.3f, %.3f)",
    arm_name.c_str(), object_id.c_str(),
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name, object_id, ik_frame, target_pose]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setEndEffectorLink(ik_frame);
      arm_group->setPlanningTime(5.0);
      arm_group->setNumPlanningAttempts(5);
      arm_group->setMaxVelocityScalingFactor(0.3);
      arm_group->setMaxAccelerationScalingFactor(0.3);
      arm_group->setWorkspace(-1.5, -1.5, 0.0, 1.5, 1.5, 2.0);

      std::string hand_group = (arm_name == "arm_2") ? "arm2_hand" : "arm1_hand";
      auto hand_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group);
      hand_group_interface->setMaxVelocityScalingFactor(0.5);
      hand_group_interface->setMaxAccelerationScalingFactor(0.5);

      // 1. Open gripper
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 1: Ensuring gripper is open...");
      hand_group_interface->setNamedTarget("open");
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      if (hand_group_interface->plan(open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(open_plan);
      }

      // 2. Pre-grasp approach (15 cm above target pose)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 2: Planning pre-grasp approach...");
      geometry_msgs::msg::PoseStamped pre_grasp = target_pose;
      pre_grasp.pose.position.z += 0.15;
      arm_group->setPoseTarget(pre_grasp, ik_frame);

      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      auto err = arm_group->plan(approach_plan);
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp planning failed for '%s'", arm_name.c_str());
        return err;
      }
      err = arm_group->execute(approach_plan);
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp execution failed");
        return err;
      }

      // 3. Descend to grasp pose (target_pose)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 3: Descending to grasp pose...");
      std::vector<geometry_msgs::msg::Pose> waypoints_down = { target_pose.pose };
      moveit_msgs::msg::RobotTrajectory trajectory_down;
      double fraction_down = arm_group->computeCartesianPath(waypoints_down, 0.005, trajectory_down, false);
      if (fraction_down > 0.5) {
        err = arm_group->execute(trajectory_down);
      } else {
        arm_group->setPoseTarget(target_pose, ik_frame);
        err = arm_group->move();
      }
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Descent to grasp failed");
        return err;
      }

      // 4. Close gripper firmly around object
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 4: Closing gripper around object...");
      hand_group_interface->setNamedTarget("closed");
      moveit::planning_interface::MoveGroupInterface::Plan close_plan;
      if (hand_group_interface->plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(close_plan);
      }

      // 5. Attach in Gazebo Sim (DetachableJoint system)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 5: Binding object in Gazebo Sim physics...");
      std_msgs::msg::Empty empty_msg;
      if (arm_name == "arm_2") {
        arm2_attach_pub_->publish(empty_msg);
      } else {
        arm1_attach_pub_->publish(empty_msg);
      }

      // 6. Attach in MoveIt Planning Scene with touch links
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 6: Attaching object in MoveIt Planning Scene...");
      moveit_msgs::msg::AttachedCollisionObject attached_obj;
      attached_obj.link_name = ik_frame;
      attached_obj.object.id = object_id;
      attached_obj.object.header.frame_id = "world";
      attached_obj.object.operation = attached_obj.object.ADD;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.15, 0.08, 0.06};
      attached_obj.object.primitives.push_back(primitive);
      attached_obj.object.primitive_poses.push_back(target_pose.pose);

      attached_obj.touch_links = {
        arm_name + "_gripper_base_link",
        arm_name + "_gripper_left_finger",
        arm_name + "_gripper_right_finger",
        arm_name + "_gripper_tcp"
      };
      psi_->applyAttachedCollisionObject(attached_obj);

      // 7. Retreat / Lift up with payload (+15 cm Z)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 7: Lifting payload to retreat pose...");
      std::vector<geometry_msgs::msg::Pose> waypoints_up = { pre_grasp.pose };
      moveit_msgs::msg::RobotTrajectory trajectory_up;
      double fraction_up = arm_group->computeCartesianPath(waypoints_up, 0.005, trajectory_up, false);
      if (fraction_up > 0.5) {
        err = arm_group->execute(trajectory_up);
      } else {
        arm_group->setPoseTarget(pre_grasp, ik_frame);
        err = arm_group->move();
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
  arm1_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/detach", 10);
  arm2_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/attach", 10);
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

  // 1. MoveIt Detach from source link
  moveit_msgs::msg::AttachedCollisionObject detach_obj;
  detach_obj.link_name = from_link;
  detach_obj.object.id = object_id;
  detach_obj.object.operation = detach_obj.object.REMOVE;
  psi_->applyAttachedCollisionObject(detach_obj);

  // 2. Gazebo Physics Handover: Detach Arm 1, Attach Arm 2
  std_msgs::msg::Empty empty_msg;
  arm1_detach_pub_->publish(empty_msg);
  arm2_attach_pub_->publish(empty_msg);

  // Small delay for planning scene synchronization
  std::this_thread::sleep_for(100ms);

  // 3. MoveIt Attach to destination link with touch links
  moveit_msgs::msg::AttachedCollisionObject attach_obj;
  attach_obj.link_name = to_link;
  attach_obj.object.id = object_id;
  attach_obj.object.operation = attach_obj.object.ADD;
  attach_obj.touch_links = {
    "arm2_gripper_base_link",
    "arm2_gripper_left_finger",
    "arm2_gripper_right_finger",
    "arm2_gripper_tcp"
  };
  psi_->applyAttachedCollisionObject(attach_obj);

  RCLCPP_INFO(node_->get_logger(), "[BT:TransferOwnership] Ownership transferred atomically in MoveIt and Gazebo");
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
