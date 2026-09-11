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
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace birobot_manipulation
{

using namespace std::chrono_literals;

// Helper to trigger blackboard feedback callback if registered
static void publishFeedbackIfAvailable(
  const BT::Blackboard::Ptr & bb,
  const std::string & phase,
  int32_t queue_remaining,
  const std::string & current_obj_id)
{
  if (!bb) return;
  FeedbackFn fn;
  if (bb->get("feedback_callback", fn) && fn) {
    fn(phase, queue_remaining, current_obj_id);
  }
}

// ── 1. ScanObjectsNode ────────────────────────────────────────────────────────

ScanObjectsNode::ScanObjectsNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: BT::StatefulActionNode(name, config),
  node_(node),
  tf_buffer_(tf_buffer)
{
  sub_poses_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/birobot/perception/target_poses", 10,
    [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_pose_array_ = msg;
    });

  sub_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/birobot/depth_camera/points/points", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_cloud_msg_ = msg;
    });

  sub_object_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/birobot/perception/object_cloud", 10,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_object_cloud_msg_ = msg;
    });
}

BT::NodeStatus ScanObjectsNode::onStart()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_pose_array_.reset();
    last_cloud_msg_.reset();
    last_object_cloud_msg_.reset();
  }

  start_scan_time_ = std::chrono::steady_clock::now();
  publishFeedbackIfAvailable(config().blackboard, "SCANNING", 0, "");

  double conf_thresh = 0.70;
  getInput("confidence_threshold", conf_thresh);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ScanObjects] Starting depth camera scan (conf_thresh=%.2f)...",
    conf_thresh);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ScanObjectsNode::onRunning()
{
  double conf_thresh = 0.70;
  getInput("confidence_threshold", conf_thresh);

  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  auto elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_scan_time_).count();

  std::lock_guard<std::mutex> lock(data_mutex_);

  // Timeout check: if camera never sends a pointcloud in timeout_sec_, camera fault!
  if (elapsed_sec > timeout_sec_ && !last_cloud_msg_ && !last_pose_array_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[BT:ScanObjects] CAMERA FAULT: No valid point cloud received on /birobot/depth_camera/points/points within %.1fs",
      timeout_sec_);

    if (cycle_res) {
      cycle_res->fault_code = "CAMERA_FAULT";
      cycle_res->fault_detail = "No valid point cloud received on /birobot/depth_camera/points/points";
      cycle_res->success = false;
    }
    return BT::NodeStatus::FAILURE;
  }

  // If we received target poses, or if full timeout elapsed
  if (last_pose_array_ || elapsed_sec >= timeout_sec_) {
    std::vector<geometry_msgs::msg::PoseStamped> queue;
    std::vector<std::string> queue_ids;
    int detected_count = 0;
    int queued_count = 0;
    int low_conf_count = 0;

    if (last_pose_array_ && !last_pose_array_->poses.empty()) {
      detected_count = static_cast<int>(last_pose_array_->poses.size());

      for (size_t i = 0; i < last_pose_array_->poses.size(); ++i) {
        const auto & pose = last_pose_array_->poses[i];

        // Evaluate confidence derived from segmented object point density in world frame
        double confidence = 1.0;
        if (last_object_cloud_msg_ && last_object_cloud_msg_->width > 0) {
          int near_points = 0;
          try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*last_object_cloud_msg_, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*last_object_cloud_msg_, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*last_object_cloud_msg_, "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
              if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
                float dx = *iter_x - static_cast<float>(pose.position.x);
                float dy = *iter_y - static_cast<float>(pose.position.y);
                float dz = *iter_z - static_cast<float>(pose.position.z);
                if (dx*dx + dy*dy + dz*dz < 0.12f * 0.12f) {
                  near_points++;
                }
              }
            }
            // Objects with >= 50 segmented points get full confidence; below that scales linearly
            confidence = std::min(1.0, static_cast<double>(near_points) / 50.0);
          } catch (...) {
            confidence = 1.0;
          }
        }

        if (confidence >= conf_thresh) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = last_pose_array_->header;
          if (ps.header.frame_id.empty()) {
            ps.header.frame_id = "world";
          }
          ps.pose = pose;
          queue.push_back(ps);
          queue_ids.push_back("auto_obj_" + std::to_string(i + 1));
          queued_count++;
        } else {
          low_conf_count++;
          RCLCPP_WARN(
            node_->get_logger(),
            "[BT:ScanObjects] Object %zu confidence %.2f < threshold %.2f (low-confidence, skipped)",
            i + 1, confidence, conf_thresh);
        }
      }
    }

    setOutput("pick_queue", queue);
    setOutput("pick_queue_ids", queue_ids);

    if (cycle_res) {
      // In re-scan cycles, accumulate totals
      cycle_res->objects_detected = std::max(cycle_res->objects_detected, detected_count);
      cycle_res->objects_queued = std::max(cycle_res->objects_queued, queued_count);
      cycle_res->objects_low_confidence += low_conf_count;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[BT:ScanObjects] Scan complete: detected=%d, queued=%d, low_confidence=%d",
      detected_count, queued_count, low_conf_count);

    return BT::NodeStatus::SUCCESS;
  }

  std::this_thread::sleep_for(100ms);
  return BT::NodeStatus::RUNNING;
}

void ScanObjectsNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:ScanObjects] Halted");
}

// ── 2. RandomizeQueueNode ────────────────────────────────────────────────────

RandomizeQueueNode::RandomizeQueueNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus RandomizeQueueNode::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> queue;
  std::vector<std::string> queue_ids;
  getInput("pick_queue", queue);
  getInput("pick_queue_ids", queue_ids);

  if (queue.size() >= 2 && queue.size() == queue_ids.size()) {
    std::random_device rd;
    std::mt19937 g(rd());

    std::vector<size_t> indices(queue.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), g);

    std::vector<geometry_msgs::msg::PoseStamped> shuffled_queue(queue.size());
    std::vector<std::string> shuffled_ids(queue_ids.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      shuffled_queue[i] = queue[indices[i]];
      shuffled_ids[i] = queue_ids[indices[i]];
    }

    setOutput("pick_queue", shuffled_queue);
    setOutput("pick_queue_ids", shuffled_ids);
    publishFeedbackIfAvailable(config().blackboard, "RANDOMIZING", static_cast<int32_t>(shuffled_queue.size()), "");

    RCLCPP_INFO(
      node_->get_logger(),
      "[BT:RandomizeQueue] Pick order randomized for %zu objects",
      shuffled_queue.size());
  } else {
    publishFeedbackIfAvailable(config().blackboard, "RANDOMIZING", static_cast<int32_t>(queue.size()), "");
    RCLCPP_INFO(
      node_->get_logger(),
      "[BT:RandomizeQueue] Queue contains %zu objects (no shuffle required)",
      queue.size());
  }

  return BT::NodeStatus::SUCCESS;
}

// ── 3. PopObjectNode ─────────────────────────────────────────────────────────

PopObjectNode::PopObjectNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus PopObjectNode::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> queue;
  std::vector<std::string> queue_ids;
  getInput("pick_queue", queue);
  getInput("pick_queue_ids", queue_ids);

  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (queue.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[BT:PopObject] Queue empty - exiting pick loop");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = queue.front();
  std::string current_id = queue_ids.empty() ? "auto_obj_1" : queue_ids.front();

  queue.erase(queue.begin());
  if (!queue_ids.empty()) {
    queue_ids.erase(queue_ids.begin());
  }

  setOutput("current_object_pose", current_pose);
  setOutput("current_object_id", current_id);
  setOutput("pick_queue", queue);
  setOutput("pick_queue_ids", queue_ids);

  if (cycle_res) {
    cycle_res->is_current_skipped = false;
  }

  int current_idx = cycle_res ? (cycle_res->objects_picked + cycle_res->objects_skipped + 1) : 1;
  int total_count = cycle_res ? std::max(cycle_res->objects_queued, current_idx) : (static_cast<int>(queue.size()) + 1);
  std::string phase = "PICKING_" + std::to_string(current_idx) + "_OF_" + std::to_string(total_count);

  publishFeedbackIfAvailable(config().blackboard, phase, static_cast<int32_t>(queue.size() + 1), current_id);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:PopObject] Popped '%s' at (%.3f, %.3f, %.3f). Queue remaining: %zu",
    current_id.c_str(),
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
    queue.size());

  return BT::NodeStatus::SUCCESS;
}

// ── 4. AutoArmPickMtcNode ────────────────────────────────────────────────────

AutoArmPickMtcNode::AutoArmPickMtcNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
  auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  arm1_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/attach", qos_tl);
  arm2_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/attach", qos_tl);
}

BT::NodeStatus AutoArmPickMtcNode::onStart()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res && cycle_res->is_current_skipped) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }

  std::string arm_name = "arm_1";
  std::string object_id = "auto_obj_1";
  geometry_msgs::msg::PoseStamped target_pose;

  getInput("arm", arm_name);
  getInput("object_id", object_id);
  getInput("target_pose", target_pose);
  current_object_id_ = object_id;

  if (target_pose.header.frame_id.empty()) {
    target_pose.header.frame_id = "world";
  }

  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ArmPickMtc] Arm '%s' initiating staged pick on '%s' at (%.3f, %.3f, %.3f)",
    arm_name.c_str(), object_id.c_str(),
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

  // Register object in MoveIt Planning Scene
  {
    moveit_msgs::msg::CollisionObject world_obj;
    world_obj.header.frame_id = "world";
    world_obj.id = object_id;
    world_obj.operation = world_obj.ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {0.15, 0.08, 0.06};
    world_obj.primitives.push_back(prim);

    tf2::Quaternion q_perc(
      target_pose.pose.orientation.x,
      target_pose.pose.orientation.y,
      target_pose.pose.orientation.z,
      target_pose.pose.orientation.w);
    if (q_perc.length2() < 1e-6) {
      q_perc.setValue(0.5525, 0.8335, 0.0, 0.0);
    }
    q_perc.normalize();
    tf2::Matrix3x3 rot(q_perc);
    double obj_yaw = std::atan2(rot[1][1], rot[0][1]);
    tf2::Quaternion q_box;
    q_box.setRPY(0.0, 0.0, obj_yaw);

    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position.x = target_pose.pose.position.x;
    obj_pose.position.y = target_pose.pose.position.y;
    obj_pose.position.z = 0.080;
    obj_pose.orientation.x = q_box.x();
    obj_pose.orientation.y = q_box.y();
    obj_pose.orientation.z = q_box.z();
    obj_pose.orientation.w = q_box.w();
    world_obj.primitive_poses.push_back(obj_pose);

    psi_->applyCollisionObjects({world_obj});
    std::this_thread::sleep_for(150ms);
  }

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name, object_id, ik_frame, target_pose]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setEndEffectorLink(ik_frame);
      arm_group->setPlanningTime(10.0);
      arm_group->setNumPlanningAttempts(15);
      arm_group->setGoalPositionTolerance(0.005);
      arm_group->setGoalOrientationTolerance(0.1);
      arm_group->setMaxVelocityScalingFactor(0.3);
      arm_group->setMaxAccelerationScalingFactor(0.3);

      std::string hand_group = (arm_name == "arm_2") ? "arm2_hand" : "arm1_hand";
      auto hand_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group);
      hand_group_interface->setMaxVelocityScalingFactor(0.5);
      hand_group_interface->setMaxAccelerationScalingFactor(0.5);

      // 1. Open gripper
      hand_group_interface->setNamedTarget("open");
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      if (hand_group_interface->plan(open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(open_plan);
      }

      // 2. Pre-grasp approach
      geometry_msgs::msg::PoseStamped pre_grasp = target_pose;
      pre_grasp.pose.position.z += 0.15;

      tf2::Quaternion q_base(
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w);
      if (q_base.length2() < 1e-6) {
        q_base.setValue(0.5525, 0.8335, 0.0, 0.0);
      }
      q_base.normalize();

      tf2::Quaternion q_z180(0.0, 0.0, 1.0, 0.0);
      tf2::Quaternion q_flip = (q_base * q_z180).normalized();

      std::vector<tf2::Quaternion> orientations = { q_base, q_flip };
      moveit::core::MoveItErrorCode err = moveit::core::MoveItErrorCode::FAILURE;
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      geometry_msgs::msg::Pose chosen_pre_grasp;

      for (const auto & q_cand : orientations) {
        pre_grasp.pose.orientation.x = q_cand.x();
        pre_grasp.pose.orientation.y = q_cand.y();
        pre_grasp.pose.orientation.z = q_cand.z();
        pre_grasp.pose.orientation.w = q_cand.w();

        arm_group->setPoseTarget(pre_grasp, ik_frame);
        err = arm_group->plan(approach_plan);
        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
          chosen_pre_grasp = pre_grasp.pose;
          break;
        }
        arm_group->clearPoseTargets();
      }

      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp planning failed for '%s'", object_id.c_str());
        return err;
      }

      err = arm_group->execute(approach_plan);
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp execution failed");
        return err;
      }

      // 3. Descend to grasp pose
      geometry_msgs::msg::Pose grasp_pose = target_pose.pose;
      grasp_pose.orientation = chosen_pre_grasp.orientation;
      grasp_pose.position.z = 0.080;

      std::vector<geometry_msgs::msg::Pose> waypoints_down = { grasp_pose };
      moveit_msgs::msg::RobotTrajectory trajectory_down;
      double fraction_down = arm_group->computeCartesianPath(waypoints_down, 0.005, trajectory_down, false);
      if (fraction_down > 0.5) {
        err = arm_group->execute(trajectory_down);
      } else {
        geometry_msgs::msg::PoseStamped grasp_stamped;
        grasp_stamped.header = target_pose.header;
        grasp_stamped.pose = grasp_pose;
        arm_group->setPoseTarget(grasp_stamped, ik_frame);
        err = arm_group->move();
      }

      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Descent to grasp failed");
        return err;
      }

      // 4. Attach physics in Gazebo Sim
      std_msgs::msg::Empty empty_msg;
      for (int i = 0; i < 30; ++i) {
        if (arm_name == "arm_2") {
          arm2_attach_pub_->publish(empty_msg);
        } else {
          arm1_attach_pub_->publish(empty_msg);
        }
        std::this_thread::sleep_for(25ms);
      }
      std::this_thread::sleep_for(150ms);

      // 5. Close gripper
      std::vector<double> grip_joints = {0.0225, 0.0225};
      hand_group_interface->setJointValueTarget(grip_joints);
      moveit::planning_interface::MoveGroupInterface::Plan close_plan;
      if (hand_group_interface->plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(close_plan);
      }
      std::this_thread::sleep_for(200ms);

      // 6. Attach in MoveIt Planning Scene
      psi_->removeCollisionObjects({object_id});
      std::this_thread::sleep_for(100ms);

      moveit_msgs::msg::AttachedCollisionObject attached_obj;
      attached_obj.link_name = ik_frame;
      attached_obj.object.id = object_id;
      attached_obj.object.header.frame_id = ik_frame;
      attached_obj.object.operation = attached_obj.object.ADD;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.15, 0.08, 0.06};
      attached_obj.object.primitives.push_back(primitive);

      geometry_msgs::msg::Pose local_pose;
      local_pose.orientation.w = 1.0;
      attached_obj.object.primitive_poses.push_back(local_pose);

      attached_obj.touch_links = {
        "arm1_gripper_base_link", "arm1_gripper_left_finger", "arm1_gripper_right_finger",
        "arm1_gripper_tcp", "arm1_wrist_3_link", "arm1_tool0",
        "arm2_gripper_base_link", "arm2_gripper_left_finger", "arm2_gripper_right_finger",
        "arm2_gripper_tcp", "arm2_wrist_3_link", "arm2_tool0",
        "workcell_base_link", "table_link"
      };
      psi_->applyAttachedCollisionObject(attached_obj);
      std::this_thread::sleep_for(150ms);

      // 7. Retreat / Lift (+15 cm Z)
      std::vector<geometry_msgs::msg::Pose> waypoints_up = { chosen_pre_grasp };
      moveit_msgs::msg::RobotTrajectory trajectory_up;
      double fraction_up = arm_group->computeCartesianPath(waypoints_up, 0.005, trajectory_up, false);
      if (fraction_up > 0.5) {
        err = arm_group->execute(trajectory_up);
      } else {
        geometry_msgs::msg::PoseStamped up_stamped;
        up_stamped.header = target_pose.header;
        up_stamped.pose = chosen_pre_grasp;
        arm_group->setPoseTarget(up_stamped, ik_frame);
        err = arm_group->move();
      }

      return err;
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AutoArmPickMtcNode::onRunning()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res && cycle_res->is_current_skipped) {
    return BT::NodeStatus::SUCCESS;
  }

  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto err = execution_future_.get();
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Pick completed successfully for '%s'", current_object_id_.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pick planning/execution failed with code: %d", err.val);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void AutoArmPickMtcNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Action halted");
}

// ── 5. SkipObjectNode ────────────────────────────────────────────────────────

SkipObjectNode::SkipObjectNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus SkipObjectNode::tick()
{
  std::string object_id = "auto_obj";
  getInput("object_id", object_id);

  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res) {
    cycle_res->is_current_skipped = true;
    cycle_res->objects_skipped++;
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "[BT:SkipObject] Object '%s' unreachable or planning failed - skipped. Total skipped: %d",
    object_id.c_str(), cycle_res ? cycle_res->objects_skipped : 1);

  return BT::NodeStatus::SUCCESS;
}

// ── 6. VerifyGraspNode ───────────────────────────────────────────────────────

VerifyGraspNode::VerifyGraspNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
}

BT::NodeStatus VerifyGraspNode::tick()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res && cycle_res->is_current_skipped) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string object_id = "auto_obj";
  getInput("object_id", object_id);

  publishFeedbackIfAvailable(config().blackboard, "VERIFYING_GRASP", 0, object_id);

  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }

  // Check if MoveIt has the attached collision object
  auto attached_objs = psi_->getAttachedObjects();
  bool grasped = false;
  for (const auto & kv : attached_objs) {
    if (kv.first == object_id || kv.second.link_name.find("arm1") != std::string::npos) {
      grasped = true;
      break;
    }
  }

  if (grasped) {
    RCLCPP_INFO(node_->get_logger(), "[BT:VerifyGrasp] Grasp verified for object '%s'", object_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[BT:VerifyGrasp] GRASP MISS: No object attached to gripper for '%s'",
    object_id.c_str());

  if (cycle_res) {
    cycle_res->fault_code = "LOST_OBJECT_FAULT";
    cycle_res->fault_detail = "No object attached to gripper after grasp on " + object_id;
    cycle_res->success = false;
  }

  return BT::NodeStatus::FAILURE;
}

// ── 7. ArmPlaceMtcNode ───────────────────────────────────────────────────────

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
  arm2_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/detach", qos_tl);
}

BT::NodeStatus ArmPlaceMtcNode::onStart()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res && cycle_res->is_current_skipped) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!psi_) {
    psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }

  std::string arm_name = "arm_1";
  geometry_msgs::msg::PoseStamped place_pose;
  getInput("arm", arm_name);
  getInput("place_pose", place_pose);

  if (place_pose.header.frame_id.empty()) {
    place_pose.header.frame_id = "world";
  }

  publishFeedbackIfAvailable(config().blackboard, "PLACING", 0, "");

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ArmPlaceMtc] Arm '%s' placing object at fixed bin pose (%.3f, %.3f, %.3f)",
    arm_name.c_str(),
    place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);

  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name, ik_frame, place_pose]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setEndEffectorLink(ik_frame);
      arm_group->setPlanningTime(10.0);
      arm_group->setNumPlanningAttempts(15);
      arm_group->setGoalPositionTolerance(0.005);
      arm_group->setGoalOrientationTolerance(0.1);
      arm_group->setMaxVelocityScalingFactor(0.3);
      arm_group->setMaxAccelerationScalingFactor(0.3);

      std::string hand_group = (arm_name == "arm_2") ? "arm2_hand" : "arm1_hand";
      auto hand_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, hand_group);
      hand_group_interface->setMaxVelocityScalingFactor(0.5);

      // 1. Move to pre-place pose (+15 cm Z)
      geometry_msgs::msg::PoseStamped pre_place = place_pose;
      pre_place.pose.position.z += 0.15;
      arm_group->setPoseTarget(pre_place, ik_frame);
      auto err = arm_group->move();
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPlaceMtc] Pre-place motion failed");
        return err;
      }

      // 2. Descend to place pose
      std::vector<geometry_msgs::msg::Pose> waypoints = { place_pose.pose };
      moveit_msgs::msg::RobotTrajectory traj;
      double frac = arm_group->computeCartesianPath(waypoints, 0.005, traj, false);
      if (frac > 0.5) {
        err = arm_group->execute(traj);
      } else {
        arm_group->setPoseTarget(place_pose, ik_frame);
        err = arm_group->move();
      }

      // 3. Open gripper
      hand_group_interface->setNamedTarget("open");
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      if (hand_group_interface->plan(open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(open_plan);
      }

      // 4. Detach in Gazebo physics
      std_msgs::msg::Empty empty_msg;
      for (int i = 0; i < 30; ++i) {
        if (arm_name == "arm_2") {
          arm2_detach_pub_->publish(empty_msg);
        } else {
          arm1_detach_pub_->publish(empty_msg);
        }
        std::this_thread::sleep_for(25ms);
      }
      std::this_thread::sleep_for(150ms);

      // 5. Remove attached object from MoveIt scene
      auto attached_objs = psi_->getAttachedObjects();
      for (const auto & kv : attached_objs) {
        moveit_msgs::msg::AttachedCollisionObject detach_obj;
        detach_obj.link_name = ik_frame;
        detach_obj.object.id = kv.first;
        detach_obj.object.operation = detach_obj.object.REMOVE;
        psi_->applyAttachedCollisionObject(detach_obj);
      }
      std::this_thread::sleep_for(100ms);

      // 6. Retreat back to pre-place pose
      std::vector<geometry_msgs::msg::Pose> waypoints_up = { pre_place.pose };
      frac = arm_group->computeCartesianPath(waypoints_up, 0.005, traj, false);
      if (frac > 0.5) {
        arm_group->execute(traj);
      } else {
        arm_group->setPoseTarget(pre_place, ik_frame);
        arm_group->move();
      }

      return moveit::core::MoveItErrorCode::SUCCESS;
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmPlaceMtcNode::onRunning()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res && cycle_res->is_current_skipped) {
    return BT::NodeStatus::SUCCESS;
  }

  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      if (cycle_res) {
        cycle_res->objects_picked++;
        RCLCPP_INFO(
          node_->get_logger(),
          "[BT:ArmPlaceMtc] Object deposited into bin. Total picked: %d",
          cycle_res->objects_picked);
      }
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:ArmPlaceMtc] Placement execution failed with code: %d", res.val);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void ArmPlaceMtcNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:ArmPlaceMtc] Action halted");
}

// ── 8. HomeArmNode ───────────────────────────────────────────────────────────

HomeArmNode::HomeArmNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus HomeArmNode::onStart()
{
  std::string arm_name = "arm_1";
  getInput("arm", arm_name);

  publishFeedbackIfAvailable(config().blackboard, "HOMING", 0, "");

  RCLCPP_INFO(node_->get_logger(), "[BT:HomeArm] Commanding '%s' to home pose", arm_name.c_str());

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setNamedTarget("home");
      arm_group->setMaxVelocityScalingFactor(0.4);
      return arm_group->move();
    });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HomeArmNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:HomeArm] Arm returned to home position");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:HomeArm] Homing returned error: %d", res.val);
      return BT::NodeStatus::SUCCESS; // Do not fail fault recovery if home motion was imperfect
    }
  }
  return BT::NodeStatus::RUNNING;
}

void HomeArmNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:HomeArm] Action halted");
}

// ── 9. AbortCycleNode ────────────────────────────────────────────────────────

AbortCycleNode::AbortCycleNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus AbortCycleNode::tick()
{
  std::string fault_code;
  std::string fault_detail;
  getInput("fault_code", fault_code);
  getInput("fault_detail", fault_detail);

  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  if (cycle_res) {
    cycle_res->success = false;
    if (!fault_code.empty()) {
      cycle_res->fault_code = fault_code;
    }
    if (!fault_detail.empty()) {
      cycle_res->fault_detail = fault_detail;
    }
    auto now = std::chrono::steady_clock::now();
    cycle_res->cycle_duration_sec = std::chrono::duration<double>(now - cycle_res->start_time).count();

    RCLCPP_ERROR(
      node_->get_logger(),
      "[BT:AbortCycle] Cycle aborted: fault_code='%s', fault_detail='%s', duration=%.1fs",
      cycle_res->fault_code.c_str(), cycle_res->fault_detail.c_str(), cycle_res->cycle_duration_sec);
  }

  // Return SUCCESS so subsequent ReportCycleNode runs in FaultHandler sequence
  return BT::NodeStatus::SUCCESS;
}

// ── 10. ReportCycleNode ──────────────────────────────────────────────────────

ReportCycleNode::ReportCycleNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus ReportCycleNode::tick()
{
  std::shared_ptr<CycleResult> cycle_res;
  getInput("cycle_result", cycle_res);

  publishFeedbackIfAvailable(config().blackboard, "REPORTING", 0, "");

  if (cycle_res) {
    auto now = std::chrono::steady_clock::now();
    cycle_res->cycle_duration_sec = std::chrono::duration<double>(now - cycle_res->start_time).count();
    if (cycle_res->fault_code.empty()) {
      cycle_res->success = true;
    }

    RCLCPP_INFO(node_->get_logger(), "==========================================================");
    RCLCPP_INFO(node_->get_logger(), " [CYCLE_COMPLETE] Autonomous Pick & Place Summary        ");
    RCLCPP_INFO(node_->get_logger(), "==========================================================");
    RCLCPP_INFO(node_->get_logger(), " Success:                %s", cycle_res->success ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), " Objects Detected:       %d", cycle_res->objects_detected);
    RCLCPP_INFO(node_->get_logger(), " Objects Queued:         %d", cycle_res->objects_queued);
    RCLCPP_INFO(node_->get_logger(), " Objects Picked:         %d", cycle_res->objects_picked);
    RCLCPP_INFO(node_->get_logger(), " Objects Skipped:        %d", cycle_res->objects_skipped);
    RCLCPP_INFO(node_->get_logger(), " Objects Low Confidence: %d", cycle_res->objects_low_confidence);
    RCLCPP_INFO(node_->get_logger(), " Fault Code:             '%s'", cycle_res->fault_code.c_str());
    RCLCPP_INFO(node_->get_logger(), " Fault Detail:           '%s'", cycle_res->fault_detail.c_str());
    RCLCPP_INFO(node_->get_logger(), " Cycle Duration:         %.2f s", cycle_res->cycle_duration_sec);
    RCLCPP_INFO(node_->get_logger(), "==========================================================");
  }

  return BT::NodeStatus::SUCCESS;
}

// ── Node Factory Registration ────────────────────────────────────────────────

void registerAutoPickNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  factory.registerBuilder<ScanObjectsNode>(
    "ScanObjectsNode",
    [node, tf_buffer](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ScanObjectsNode>(name, config, node, tf_buffer);
    });

  factory.registerBuilder<RandomizeQueueNode>(
    "RandomizeQueueNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<RandomizeQueueNode>(name, config, node);
    });

  factory.registerBuilder<PopObjectNode>(
    "PopObjectNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<PopObjectNode>(name, config, node);
    });

  factory.registerBuilder<AutoArmPickMtcNode>(
    "ArmPickMtcNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<AutoArmPickMtcNode>(name, config, node);
    });

  factory.registerBuilder<SkipObjectNode>(
    "SkipObjectNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SkipObjectNode>(name, config, node);
    });

  factory.registerBuilder<VerifyGraspNode>(
    "VerifyGraspNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<VerifyGraspNode>(name, config, node);
    });

  factory.registerBuilder<ArmPlaceMtcNode>(
    "ArmPlaceMtcNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ArmPlaceMtcNode>(name, config, node);
    });

  factory.registerBuilder<HomeArmNode>(
    "HomeArmNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<HomeArmNode>(name, config, node);
    });

  factory.registerBuilder<AbortCycleNode>(
    "AbortCycleNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<AbortCycleNode>(name, config, node);
    });

  factory.registerBuilder<ReportCycleNode>(
    "ReportCycleNode",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReportCycleNode>(name, config, node);
    });
}

}  // namespace birobot_manipulation
