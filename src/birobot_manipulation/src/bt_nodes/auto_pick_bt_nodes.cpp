#include "birobot_manipulation/auto_pick_bt_nodes.hpp"

#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <random>
#include <algorithm>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/empty.hpp>

namespace birobot_manipulation
{
using namespace std::chrono_literals;

// ── 1. ScanObjectsNode ───────────────────────────────────────────────────────
ScanObjectsNode::ScanObjectsNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: BT::StatefulActionNode(name, config),
  node_(node),
  tf_buffer_(tf_buffer)
{
}

BT::NodeStatus ScanObjectsNode::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:ScanObjects] Scanning for objects on /birobot/perception/target_poses");
  config().blackboard->set("current_phase", std::string("SCANNING"));
  
  double confidence_threshold = 0.70;
  getInput("confidence_threshold", confidence_threshold);

  // Subscribe to PoseArray temporarily to get the latest scan
  std::shared_ptr<geometry_msgs::msg::PoseArray> latest_msg;
  auto sub = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/birobot/perception/target_poses", 
    rclcpp::QoS(1),
    [&latest_msg](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      latest_msg = msg;
    });

  // Wait up to 5 seconds for a message
  auto start = node_->now();
  while (!latest_msg && (node_->now() - start).seconds() < 5.0) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
  }

  if (!latest_msg) {
    RCLCPP_ERROR(node_->get_logger(), "[BT:ScanObjects] Timeout waiting for perception data. Camera fault?");
    
    // Update CycleResult
    CycleResult cycle_result;
    config().blackboard->get("cycle_result", cycle_result);
    cycle_result.success = false;
    cycle_result.fault_code = "CAMERA_FAULT";
    cycle_result.fault_detail = "No valid point cloud/poses received on /birobot/perception/target_poses";
    config().blackboard->set("cycle_result", cycle_result);
    
    return BT::NodeStatus::FAILURE;
  }

  // Determine confidence threshold (using the heuristic from research.md)
  // Our perception node uses min_valid_points = 100.
  // In the real implementation, we would extract cluster size from perception directly.
  // Since we are reading PoseArray which has already passed perception's min_cluster_size filter,
  // we'll assign a simulated confidence for now based on Z-position or just default it above threshold.
  // We'll treat all received poses as passing confidence unless specifically testing.
  
  std::vector<geometry_msgs::msg::PoseStamped> pick_queue;
  std::vector<std::string> pick_queue_ids;
  
  int objects_detected = latest_msg->poses.size();
  int objects_queued = 0;
  int objects_low_confidence = 0;

  for (size_t i = 0; i < latest_msg->poses.size(); ++i) {
    // Simulated confidence check based on Z height (simulating small objects dropping below threshold)
    double simulated_confidence = 0.95; 
    if (latest_msg->poses[i].position.z < 0.055) { // Very low profile object
      simulated_confidence = 0.50; // Below 0.70 default threshold
    }

    if (simulated_confidence >= confidence_threshold) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = latest_msg->header;
      pose_stamped.pose = latest_msg->poses[i];
      pick_queue.push_back(pose_stamped);
      pick_queue_ids.push_back("auto_obj_" + std::to_string(i));
      objects_queued++;
    } else {
      objects_low_confidence++;
      RCLCPP_INFO(node_->get_logger(), "[BT:ScanObjects] Object %zu rejected (confidence %.2f < %.2f)", 
                  i, simulated_confidence, confidence_threshold);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "[BT:ScanObjects] Scan complete: %d detected, %d queued, %d rejected",
              objects_detected, objects_queued, objects_low_confidence);

  // ── Log each accepted pick pose for operator verification ──────────────
  for (size_t i = 0; i < pick_queue.size(); ++i) {
    const auto & p = pick_queue[i].pose.position;
    const auto & q = pick_queue[i].pose.orientation;
    RCLCPP_INFO(node_->get_logger(),
      "[BT:ScanObjects]   Pick[%zu] id='%s'  pos=(%.4f, %.4f, %.4f)  quat=(%.3f, %.3f, %.3f, %.3f)  frame='%s'",
      i, pick_queue_ids[i].c_str(),
      p.x, p.y, p.z,
      q.x, q.y, q.z, q.w,
      pick_queue[i].header.frame_id.c_str());
  }

  setOutput("pick_queue", pick_queue);
  setOutput("pick_queue_ids", pick_queue_ids);

  // Update CycleResult
  CycleResult cycle_result;
  config().blackboard->get("cycle_result", cycle_result);
  cycle_result.objects_detected += objects_detected;
  cycle_result.objects_queued += objects_queued;
  cycle_result.objects_low_confidence += objects_low_confidence;
  config().blackboard->set("cycle_result", cycle_result);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ScanObjectsNode::onRunning() { return BT::NodeStatus::SUCCESS; }
void ScanObjectsNode::onHalted() {}

// ── 2. RandomizeQueueNode ────────────────────────────────────────────────────
RandomizeQueueNode::RandomizeQueueNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus RandomizeQueueNode::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> pick_queue;
  std::vector<std::string> pick_queue_ids;

  if (!getInput("pick_queue", pick_queue) || !getInput("pick_queue_ids", pick_queue_ids)) {
    RCLCPP_WARN(node_->get_logger(), "[BT:RandomizeQueue] No pick queue found in blackboard");
    return BT::NodeStatus::SUCCESS;
  }

  if (pick_queue.size() > 1) {
    config().blackboard->set("current_phase", std::string("RANDOMIZING"));
    
    // Create a vector of indices and shuffle it
    std::vector<size_t> indices(pick_queue.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    
    // Apply shuffle to both queues
    std::vector<geometry_msgs::msg::PoseStamped> shuffled_queue;
    std::vector<std::string> shuffled_ids;
    
    for (size_t i : indices) {
      shuffled_queue.push_back(pick_queue[i]);
      shuffled_ids.push_back(pick_queue_ids[i]);
    }
    
    setOutput("pick_queue", shuffled_queue);
    setOutput("pick_queue_ids", shuffled_ids);
    
    RCLCPP_INFO(node_->get_logger(), "[BT:RandomizeQueue] Shuffled queue of %zu objects", pick_queue.size());
  }

  return BT::NodeStatus::SUCCESS;
}

// ── 3. PopObjectNode ─────────────────────────────────────────────────────────
PopObjectNode::PopObjectNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus PopObjectNode::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> pick_queue;
  std::vector<std::string> pick_queue_ids;

  if (!getInput("pick_queue", pick_queue) || !getInput("pick_queue_ids", pick_queue_ids)) {
    RCLCPP_ERROR(node_->get_logger(), "[BT:PopObject] Failed to get queue from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (pick_queue.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[BT:PopObject] Queue empty, exiting loop normally");
    return BT::NodeStatus::FAILURE; // This causes KeepRunningUntilFailure to exit successfully
  }

  auto current_pose = pick_queue.back();
  auto current_id = pick_queue_ids.back();
  pick_queue.pop_back();
  pick_queue_ids.pop_back();

  setOutput("pick_queue", pick_queue);
  setOutput("pick_queue_ids", pick_queue_ids);
  setOutput("current_object_pose", current_pose);
  setOutput("current_object_id", current_id);

  config().blackboard->set("queue_remaining", (int)pick_queue.size());
  
  CycleResult cr;
  if (config().blackboard->get("cycle_result", cr)) {
    int total_queued = cr.objects_queued;
    int current_index = total_queued - pick_queue.size();
    char phase_str[64];
    snprintf(phase_str, sizeof(phase_str), "PICKING_%d_OF_%d", current_index, total_queued);
    config().blackboard->set("current_phase", std::string(phase_str));
  }

  RCLCPP_INFO(node_->get_logger(), "[BT:PopObject] Popped '%s', %zu remaining in queue", 
              current_id.c_str(), pick_queue.size());

  return BT::NodeStatus::SUCCESS;
}

// ── 4. VerifyGraspNode ───────────────────────────────────────────────────────
VerifyGraspNode::VerifyGraspNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus VerifyGraspNode::tick() {
  config().blackboard->set("current_phase", std::string("VERIFYING_GRASP"));
  
  std::string object_id;
  if (!getInput("object_id", object_id)) {
    RCLCPP_WARN(node_->get_logger(), "[BT:VerifyGrasp] object_id missing from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }
  
  // Wait a short moment to ensure the planning scene is updated
  std::this_thread::sleep_for(100ms);

  // Check if object is attached
  auto attached_objects = psi_->getAttachedObjects();
  bool object_attached = false;
  
  for (const auto& pair : attached_objects) {
    if (pair.first == object_id) {
      object_attached = true;
      break;
    }
  }

  CycleResult cr;
  config().blackboard->get("cycle_result", cr);

  if (!object_attached) {
    RCLCPP_ERROR(node_->get_logger(), "[BT:VerifyGrasp] MISS! Object '%s' is not attached to gripper.", object_id.c_str());
    cr.success = false;
    cr.fault_code = "LOST_OBJECT_FAULT";
    cr.fault_detail = "No object attached to gripper after grasp attempt for " + object_id;
    config().blackboard->set("cycle_result", cr);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[BT:VerifyGrasp] Verified grasp on '%s'", object_id.c_str());
  cr.objects_picked++;
  config().blackboard->set("cycle_result", cr);
  
  return BT::NodeStatus::SUCCESS;
}

// ── 5. SkipObjectNode ────────────────────────────────────────────────────────
SkipObjectNode::SkipObjectNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus SkipObjectNode::tick() {
  std::string object_id;
  getInput("object_id", object_id);
  
  CycleResult cr;
  if (config().blackboard->get("cycle_result", cr)) {
    cr.objects_skipped++;
    config().blackboard->set("cycle_result", cr);
  }
  
  RCLCPP_WARN(node_->get_logger(), "[BT:SkipObject] Planning failed for '%s', skipping object.", object_id.c_str());
  
  // Always return SUCCESS so the Fallback block completes successfully and the loop continues
  return BT::NodeStatus::SUCCESS;
}

// ── 6. ArmPlaceMtcNode ───────────────────────────────────────────────────────
ArmPlaceMtcNode::ArmPlaceMtcNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
  auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  arm1_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/detach", qos_tl);
}

BT::NodeStatus ArmPlaceMtcNode::onStart()
{
  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }
  
  config().blackboard->set("current_phase", std::string("PLACING"));

  std::string arm_name = "arm_1";
  geometry_msgs::msg::PoseStamped place_pose;

  getInput("arm", arm_name);
  getInput("place_pose", place_pose);
  current_arm_ = arm_name;

  if (place_pose.header.frame_id.empty()) {
    place_pose.header.frame_id = "world";
  }

  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";

  RCLCPP_INFO(node_->get_logger(), "[BT:ArmPlaceMtc] Arm '%s' planning place to (%.3f, %.3f, %.3f)",
              arm_name.c_str(), place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name, ik_frame, place_pose]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setEndEffectorLink(ik_frame);
      arm_group->setPlanningTime(15.0);
      arm_group->setNumPlanningAttempts(10);
      
      std::string hand_group = (arm_name == "arm_2") ? "arm2_hand" : "arm1_hand";
      auto hand_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group);

      // 1. Move to place pose
      arm_group->setPoseTarget(place_pose, ik_frame);
      moveit::core::MoveItErrorCode err = arm_group->move();
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPlaceMtc] Failed to move to place pose");
        return err;
      }

      // 2. Detach in MoveIt
      // Get attached objects to know what we are holding
      auto attached_objects = psi_->getAttachedObjects();
      for (const auto& pair : attached_objects) {
        if (pair.second.link_name == ik_frame) {
          moveit_msgs::msg::AttachedCollisionObject detach_obj;
          detach_obj.link_name = ik_frame;
          detach_obj.object.id = pair.first;
          detach_obj.object.operation = detach_obj.object.REMOVE;
          psi_->applyAttachedCollisionObject(detach_obj);
          // Optional: remove from world entirely since it's in the bin now
          psi_->removeCollisionObjects({pair.first});
          RCLCPP_INFO(node_->get_logger(), "[BT:ArmPlaceMtc] Detached '%s' in MoveIt", pair.first.c_str());
        }
      }

      // 3. Open gripper
      hand_group_interface->setNamedTarget("open");
      hand_group_interface->move();
      
      // 4. Detach in Gazebo physics
      std_msgs::msg::Empty empty_msg;
      for (int i = 0; i < 15; ++i) {
        if (arm_name == "arm_1") arm1_detach_pub_->publish(empty_msg);
        std::this_thread::sleep_for(30ms);
      }
      
      // 5. Retreat (move up slightly)
      geometry_msgs::msg::Pose retreat_pose = place_pose.pose;
      retreat_pose.position.z += 0.15;
      std::vector<geometry_msgs::msg::Pose> waypoints_up = { retreat_pose };
      moveit_msgs::msg::RobotTrajectory trajectory_up;
      if (arm_group->computeCartesianPath(waypoints_up, 0.005, trajectory_up, false) > 0.5) {
        arm_group->execute(trajectory_up);
      } else {
        arm_group->setPoseTarget(retreat_pose, ik_frame);
        arm_group->move();
      }

      return moveit::core::MoveItErrorCode::SUCCESS;
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmPlaceMtcNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPlaceMtc] Place execution succeeded");
      config().blackboard->set("current_phase", std::string("RESCANNING"));
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:ArmPlaceMtc] Place execution failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void ArmPlaceMtcNode::onHalted() {}

// ── 7. HomeArmNode ───────────────────────────────────────────────────────────
HomeArmNode::HomeArmNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node) {}

BT::NodeStatus HomeArmNode::onStart() {
  config().blackboard->set("current_phase", std::string("HOMING"));
  
  std::string arm_name;
  if (!getInput("arm", arm_name)) {
    arm_name = "arm_1";
  }
  
  RCLCPP_INFO(node_->get_logger(), "[BT:HomeArm] Sending arm '%s' to home position...", arm_name.c_str());

  execution_future_ = std::async(std::launch::async, [this, arm_name]() -> moveit::core::MoveItErrorCode {
    auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
    arm_group->setNamedTarget("home");
    arm_group->setMaxVelocityScalingFactor(0.5);
    arm_group->setMaxAccelerationScalingFactor(0.5);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      return arm_group->execute(plan);
    }
    return moveit::core::MoveItErrorCode::FAILURE;
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HomeArmNode::onRunning() { 
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:HomeArm] Arm homed successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:HomeArm] Failed to home arm");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING; 
}
void HomeArmNode::onHalted() {}

// ── 8. AbortCycleNode ────────────────────────────────────────────────────────
AbortCycleNode::AbortCycleNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus AbortCycleNode::tick() {
  std::string fault_code, fault_detail;
  getInput("fault_code", fault_code);
  getInput("fault_detail", fault_detail);

  CycleResult cr;
  config().blackboard->get("cycle_result", cr);
  
  // Don't overwrite if it was already set by another node
  if (cr.fault_code.empty() && !fault_code.empty()) {
    cr.success = false;
    cr.fault_code = fault_code;
    cr.fault_detail = fault_detail;
    config().blackboard->set("cycle_result", cr);
  }
  
  RCLCPP_ERROR(node_->get_logger(), "[BT:AbortCycle] Cycle aborted with fault: %s", cr.fault_code.c_str());
  
  // ALWAYS return FAILURE to propagate the abort condition
  return BT::NodeStatus::FAILURE;
}

// ── 9. ReportCycleNode ───────────────────────────────────────────────────────
ReportCycleNode::ReportCycleNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node) {}

BT::NodeStatus ReportCycleNode::tick() {
  config().blackboard->set("current_phase", std::string("REPORTING"));
  
  CycleResult cr;
  if (getInput("cycle_result", cr)) {
    RCLCPP_INFO(node_->get_logger(), 
      "[BT:ReportCycle] === CYCLE SUMMARY ===\n"
      "  Success: %s\n"
      "  Objects Detected: %d\n"
      "  Objects Queued: %d\n"
      "  Objects Picked: %d\n"
      "  Objects Skipped: %d\n"
      "  Low Confidence: %d\n"
      "  Fault Code: %s\n"
      "  Duration: %.1f sec",
      cr.success ? "TRUE" : "FALSE",
      cr.objects_detected, cr.objects_queued, cr.objects_picked,
      cr.objects_skipped, cr.objects_low_confidence,
      cr.fault_code.empty() ? "NONE" : cr.fault_code.c_str(),
      (node_->now() - cr.start_time).seconds());
  } else {
    RCLCPP_WARN(node_->get_logger(), "[BT:ReportCycle] Could not read cycle_result from blackboard");
  }
  return BT::NodeStatus::SUCCESS;
}

// ── Factory Registration ─────────────────────────────────────────────────────
void registerAutoPickNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  factory.registerBuilder<ScanObjectsNode>("ScanObjectsNode", [node, tf_buffer](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<ScanObjectsNode>(name, config, node, tf_buffer);
  });
  factory.registerBuilder<RandomizeQueueNode>("RandomizeQueueNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<RandomizeQueueNode>(name, config, node);
  });
  factory.registerBuilder<PopObjectNode>("PopObjectNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<PopObjectNode>(name, config, node);
  });
  factory.registerBuilder<VerifyGraspNode>("VerifyGraspNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<VerifyGraspNode>(name, config, node);
  });
  factory.registerBuilder<SkipObjectNode>("SkipObjectNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<SkipObjectNode>(name, config, node);
  });
  factory.registerBuilder<ArmPlaceMtcNode>("ArmPlaceMtcNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<ArmPlaceMtcNode>(name, config, node);
  });
  factory.registerBuilder<HomeArmNode>("HomeArmNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<HomeArmNode>(name, config, node);
  });
  factory.registerBuilder<AbortCycleNode>("AbortCycleNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<AbortCycleNode>(name, config, node);
  });
  factory.registerBuilder<ReportCycleNode>("ReportCycleNode", [node](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<ReportCycleNode>(name, config, node);
  });
}

}  // namespace birobot_manipulation
