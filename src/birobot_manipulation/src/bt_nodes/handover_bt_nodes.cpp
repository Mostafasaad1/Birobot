#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

#include <cmath>
#include <vector>
#include <thread>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

  // Retry TF lookup for up to 10 seconds to allow perception pipeline to start
  bool tf_found = false;
  std::string chosen_frame = "red_object_target";
  if (tf_buffer_) {
    const int max_polls = 100;   // 100 * 100ms = 10 seconds
    for (int poll = 0; poll < max_polls && !tf_found; ++poll) {
      try {
        if (tf_buffer_->canTransform("world", "red_object_target", tf2::TimePointZero, 100ms)) {
          chosen_frame = "red_object_target";
        } else if (tf_buffer_->canTransform("world", "grasp_target_1", tf2::TimePointZero, 100ms)) {
          chosen_frame = "grasp_target_1";
        } else {
          if (poll % 10 == 0) {
            RCLCPP_INFO(node_->get_logger(),
              "[BT:DetectObject] Waiting for perception TF 'red_object_target' or 'grasp_target_1' (%d/100)...", poll + 1);
          }
          std::this_thread::sleep_for(100ms);
          continue;
        }

        auto tf = tf_buffer_->lookupTransform("world", chosen_frame, tf2::TimePointZero);
        detected_pose.pose.position.x = tf.transform.translation.x;
        detected_pose.pose.position.y = tf.transform.translation.y;
        detected_pose.pose.position.z = tf.transform.translation.z;
        detected_pose.pose.orientation = tf.transform.rotation;
        tf_found = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "[BT:DetectObject] Resolved dynamic '%s' from 3D perception: (%.3f, %.3f, %.3f) after %d polls",
          chosen_frame.c_str(),
          detected_pose.pose.position.x, detected_pose.pose.position.y,
          detected_pose.pose.position.z, poll + 1);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(node_->get_logger(), "[BT:DetectObject] TF lookup attempt %d failed: %s", poll, ex.what());
        std::this_thread::sleep_for(100ms);
      }
    }
  }

  if (!tf_found) {
    // Fallback: Use canonical table surface position matching gazebo.launch.py spawn args:
    // -x 0.10 -y 0.05 -z 0.15 -Y 0.4
    // Table surface is at z=0.05, object sits from z=0.05 to z=0.25 (top surface at 0.250).
    detected_pose.pose.position.x = 0.10;
    detected_pose.pose.position.y = 0.05;
    detected_pose.pose.position.z = 0.250;
    // Width-aligned top-down orientation: object yaw=0.4 rad → grasp yaw = 0.4 + pi/2 = 1.9708 rad
    // q = (sin(pi/2)*cos(yaw/2), sin(pi/2)*sin(yaw/2), 0, 0) = (cos(yaw/2), sin(yaw/2), 0, 0)
    detected_pose.pose.orientation.x = 0.5525;
    detected_pose.pose.orientation.y = 0.8335;
    detected_pose.pose.orientation.z = 0.0;
    detected_pose.pose.orientation.w = 0.0;
    RCLCPP_WARN(
      node_->get_logger(),
      "[BT:DetectObject] *** PERCEPTION TIMEOUT: 3D detection did not find object after 10s. "
      "Using HARDCODED pose (%.3f, %.3f, %.3f). Check perception pipeline! ***",
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
  auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  arm1_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/detach", qos_tl);
  arm2_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/detach", qos_tl);
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

  // Handle Gazebo and MoveIt detach if opening gripper
  // NOTE: arm2 open at drop-off physically releases the object into the bin.
  //       arm1 open at handover releases grip after TransferOwnership.
  if (action == "open") {
    std_msgs::msg::Empty empty_msg;
    if (gripper == "arm2" || gripper == "arm2_gripper") {
      // First open fingers in hardware, THEN send physics detach
      // (detach fires AFTER the gripper motion starts below, so order matters —
      //  we send the detach burst first so Gazebo gets it right as fingers open)
      RCLCPP_INFO(node_->get_logger(), "[BT:GripperControl] Sending physics detach burst for arm2...");
      for (int i = 0; i < 30; ++i) {
        arm2_detach_pub_->publish(empty_msg);
        std::this_thread::sleep_for(30ms);
      }
      std::this_thread::sleep_for(200ms);
      if (!psi_) {
        psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
      }
      // Remove from attached scene and clear from world — object is deposited into bin
      moveit_msgs::msg::AttachedCollisionObject detach_obj;
      detach_obj.link_name = "arm2_gripper_tcp";
      detach_obj.object.id = "irregular_object_1";
      detach_obj.object.operation = detach_obj.object.REMOVE;
      psi_->applyAttachedCollisionObject(detach_obj);
      psi_->removeCollisionObjects({"irregular_object_1"});
      std::this_thread::sleep_for(150ms);
      RCLCPP_INFO(node_->get_logger(),
        "[BT:GripperControl] Released and cleared payload from arm2 in Gazebo Sim and MoveIt Planning Scene");
    } else {
      // Arm 1 open at handover — physics detach already done by TransferOwnership,
      // but send a cleanup burst anyway in case it was missed
      RCLCPP_INFO(node_->get_logger(), "[BT:GripperControl] Sending physics detach cleanup burst for arm1...");
      for (int i = 0; i < 10; ++i) {
        arm1_detach_pub_->publish(empty_msg);
        std::this_thread::sleep_for(30ms);
      }
    }
  }

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
  move_group->setPlanningTime(10.0);
  move_group->setNumPlanningAttempts(10);
  move_group->setMaxVelocityScalingFactor(0.5);
  move_group->setMaxAccelerationScalingFactor(0.5);

  if (action == "close" || action == "closed") {
    // Grip object gently without crushing: object width = 0.08m -> joint = 0.0225m
    std::vector<double> grip_joints = {0.0225, 0.0225};
    move_group->setJointValueTarget(grip_joints);
  } else {
    move_group->setNamedTarget("open");
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  for (int attempt = 1; attempt <= 3 && !success; ++attempt) {
    if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      success = true;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "[BT:GripperControl] Plan attempt %d for '%s' failed, retrying in 500ms...",
        attempt, group_name.c_str());
      std::this_thread::sleep_for(500ms);
    }
  }
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
  // TRANSIENT_LOCAL: messages are stored and replayed to late-joining subscribers
  // (ros_gz_bridge connects after this node; volatile QoS would silently drop messages)
  auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  arm1_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/attach", qos_tl);
  arm2_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/attach", qos_tl);
  planning_scene_diff_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("/clear_octomap");
  // Give ros_gz_bridge time to connect and receive the transient-local message
  std::this_thread::sleep_for(1000ms);
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

  // Ensure frame_id is set
  if (target_pose.header.frame_id.empty()) {
    target_pose.header.frame_id = "world";
  }

  std::string ik_frame = (arm_name == "arm_2") ? "arm2_gripper_tcp" : "arm1_gripper_tcp";

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:ArmPickMtc] Arm '%s' initiating staged pick on '%s' at (%.3f, %.3f, %.3f)",
    arm_name.c_str(), object_id.c_str(),
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

  // ── PRE-PICK: Register object in MoveIt world scene ──────────────────────
  // This MUST happen before any planning so MoveIt knows the object exists.
  // Without this, removeCollisionObjects() is a no-op and applyAttachedCollisionObject
  // creates a phantom object that only exists in the planning scene, not in physics.
  {
    moveit_msgs::msg::CollisionObject world_obj;
    world_obj.header.frame_id = "world";
    world_obj.id = object_id;
    world_obj.operation = world_obj.ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {0.15, 0.08, 0.20};
    world_obj.primitives.push_back(prim);

    // Derive object yaw dynamically from detected perception orientation
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
    // In irregular_object_pose_estimator.cpp, Col(1) (gripper Y) is aligned with object length
    double obj_yaw = std::atan2(rot[1][1], rot[0][1]);
    tf2::Quaternion q_box;
    q_box.setRPY(0.0, 0.0, obj_yaw);

    // Use the detected pose (world frame) for the collision object placement
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position.x = target_pose.pose.position.x;
    obj_pose.position.y = target_pose.pose.position.y;
    // Object top surface is target_pose.pose.position.z (~0.25m), height=0.20m -> centre at z - 0.100m
    obj_pose.position.z = target_pose.pose.position.z - 0.100;
    obj_pose.orientation.x = q_box.x();
    obj_pose.orientation.y = q_box.y();
    obj_pose.orientation.z = q_box.z();
    obj_pose.orientation.w = q_box.w();
    world_obj.primitive_poses.push_back(obj_pose);

    psi_->applyCollisionObjects({world_obj});
    RCLCPP_INFO(node_->get_logger(),
      "[BT:ArmPickMtc] Registered '%s' in MoveIt world scene at (%.3f, %.3f, %.3f) with dynamic yaw %.3f rad",
      object_id.c_str(), obj_pose.position.x, obj_pose.position.y, obj_pose.position.z, obj_yaw);
    std::this_thread::sleep_for(200ms);  // let planning scene sync
  }

  execution_future_ = std::async(
    std::launch::async,
    [this, arm_name, object_id, ik_frame, target_pose]() -> moveit::core::MoveItErrorCode
    {
      auto arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
      arm_group->setEndEffectorLink(ik_frame);
      arm_group->setPlanningTime(15.0);
      arm_group->setNumPlanningAttempts(20);
      arm_group->setGoalPositionTolerance(0.005);
      arm_group->setGoalOrientationTolerance(0.1);
      arm_group->setMaxVelocityScalingFactor(0.3);
      arm_group->setMaxAccelerationScalingFactor(0.3);

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

      // 2. Pre-grasp approach with candidate orientations aligned with object width (0.08m)
      // Dynamic candidate orientations derived from live 3D perception
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 2: Planning pre-grasp approach...");

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

      // 180° flip around TCP Z axis
      tf2::Quaternion q_z180(0.0, 0.0, 1.0, 0.0);
      tf2::Quaternion q_flip = (q_base * q_z180).normalized();

      // Small yaw perturbations (+/- 15 deg) around TCP Z for IK flexibility
      tf2::Quaternion q_p15; q_p15.setRPY(0.0, 0.0, 0.2618);
      tf2::Quaternion q_m15; q_m15.setRPY(0.0, 0.0, -0.2618);

      struct GraspCandidate {
        tf2::Quaternion q;
        const char* label;
      };
      std::vector<GraspCandidate> candidates = {
        { q_base, "perception 3D PCA orientation" },
        { q_flip, "perception orientation (180° flip)" },
        { (q_base * q_p15).normalized(), "perception orientation (+15° tolerance)" },
        { (q_base * q_m15).normalized(), "perception orientation (-15° tolerance)" },
        { (q_flip * q_p15).normalized(), "perception 180° flip (+15° tolerance)" },
        { (q_flip * q_m15).normalized(), "perception 180° flip (-15° tolerance)" },
      };

      moveit::core::MoveItErrorCode err = moveit::core::MoveItErrorCode::FAILURE;
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      geometry_msgs::msg::Pose chosen_pre_grasp_pose;

      for (const auto& cand : candidates) {
        pre_grasp.pose.orientation.x = cand.q.x();
        pre_grasp.pose.orientation.y = cand.q.y();
        pre_grasp.pose.orientation.z = cand.q.z();
        pre_grasp.pose.orientation.w = cand.q.w();

        RCLCPP_INFO(
          node_->get_logger(),
          "[BT:ArmPickMtc] Trying pre-grasp orientation: %s", cand.label);

        arm_group->setPoseTarget(pre_grasp, ik_frame);
        err = arm_group->plan(approach_plan);
        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
          chosen_pre_grasp_pose = pre_grasp.pose;
          RCLCPP_INFO(
            node_->get_logger(),
            "[BT:ArmPickMtc] Pre-grasp orientation succeeded: %s", cand.label);
          break;
        }
        arm_group->clearPoseTargets();
      }

      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] All pre-grasp orientations failed for '%s'", arm_name.c_str());
        return err;
      }

      err = arm_group->execute(approach_plan);
      if (err != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Pre-grasp execution failed");
        return err;
      }

      // Allow physical robot in Gazebo to settle at pre-grasp pose
      std::this_thread::sleep_for(300ms);
      arm_group->setStartStateToCurrentState();

      // 3. Descend to grasp pose using Cartesian path (keep same orientation)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 3: Descending to grasp pose...");
      geometry_msgs::msg::Pose grasp_pose = target_pose.pose;
      grasp_pose.orientation = chosen_pre_grasp_pose.orientation;
      // Dynamic grasp depth: grip 35mm below the perception-detected top surface.
      // Gripper finger length is 60mm and palm is at TCP - 50mm. Grasping at (top - 35mm) keeps the palm
      // safely 15mm above the object top with zero collision, while fingers firmly grip 45mm of the object.
      const double grasp_offset_from_top = 0.035;
      grasp_pose.position.z = target_pose.pose.position.z - grasp_offset_from_top;
      RCLCPP_INFO(node_->get_logger(),
        "[BT:ArmPickMtc] Dynamic grasp Z: perception top=%.3f, grasp_target_z=%.3f (offset -%.3f)",
        target_pose.pose.position.z, grasp_pose.position.z, grasp_offset_from_top);

      std::vector<geometry_msgs::msg::Pose> waypoints_down = { grasp_pose };
      moveit_msgs::msg::RobotTrajectory trajectory_down;
      double fraction_down = arm_group->computeCartesianPath(waypoints_down, 0.005, trajectory_down, false);
      if (fraction_down > 0.5) {
        RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Cartesian descent fraction: %.2f", fraction_down);
        err = arm_group->execute(trajectory_down);
      } else {
        RCLCPP_WARN(node_->get_logger(), "[BT:ArmPickMtc] Cartesian path fraction %.2f < 0.5, using joint planning", fraction_down);
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

      // 4. Attach in Gazebo Sim physics (DetachableJoint system) FIRST
      // At this moment, the arm is at the grasp pose with fingers open around the payload.
      // Attaching now pins the object to arm wrist link at the true grasp pose,
      // preventing the fingers from knocking or shooting the object across the table.
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 4: Binding object in Gazebo Sim physics...");
      std_msgs::msg::Empty empty_msg;
      for (int i = 0; i < 30; ++i) {
        if (arm_name == "arm_2") {
          arm2_attach_pub_->publish(empty_msg);
        } else {
          arm1_attach_pub_->publish(empty_msg);
        }
        std::this_thread::sleep_for(30ms);
      }
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 4: Physics attach burst complete (30 msgs × 30ms)");
      std::this_thread::sleep_for(200ms);

      // 5. Close gripper fingers to contact object (width = 0.08m -> joint position = 0.0225m)
      // Do NOT command "closed" (0.0m) which crushes the 80mm object into 35mm gap.
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 5: Closing gripper to touch payload...");
      std::vector<double> grip_joints = {0.0225, 0.0225};
      hand_group_interface->setJointValueTarget(grip_joints);
      moveit::planning_interface::MoveGroupInterface::Plan close_plan;
      if (hand_group_interface->plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        hand_group_interface->execute(close_plan);
      }
      std::this_thread::sleep_for(300ms);

      // 6. Attach in MoveIt Planning Scene with touch links
      //    IMPORTANT: first remove the standalone collision object so the planner
      //    does not see it as a free body colliding with the workcell.
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 6: Attaching object in MoveIt Planning Scene...");

      // 6a. Remove standalone object from the world
      psi_->removeCollisionObjects({object_id});
      std::this_thread::sleep_for(100ms);  // let the scene update propagate

      // 6b. Build and apply the attached collision object
      moveit_msgs::msg::AttachedCollisionObject attached_obj;
      attached_obj.link_name = ik_frame;
      attached_obj.object.id = object_id;
      attached_obj.object.header.frame_id = ik_frame;   // pose relative to TCP now
      attached_obj.object.operation = attached_obj.object.ADD;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.15, 0.08, 0.20};
      attached_obj.object.primitives.push_back(primitive);

      // Object sits relative to TCP:
      // TCP is at 35mm below top surface.
      // Object center is 100mm below top surface.
      // So object center is 65mm along tool axis (+Z in tcp frame)
      geometry_msgs::msg::Pose local_pose;
      local_pose.position.z = 0.065;
      local_pose.orientation.w = 1.0;
      attached_obj.object.primitive_poses.push_back(local_pose);

      // Allow collisions with every gripper link, octomap, and workcell surface link
      attached_obj.touch_links = {
        "<octomap>",
        "arm1_gripper_base_link",
        "arm1_gripper_left_finger",
        "arm1_gripper_right_finger",
        "arm1_gripper_tcp",
        "arm1_wrist_3_link",
        "arm1_tool0",
        "arm2_gripper_base_link",
        "arm2_gripper_left_finger",
        "arm2_gripper_right_finger",
        "arm2_gripper_tcp",
        "arm2_wrist_3_link",
        "arm2_tool0",
        "workcell_base_link",
        "table_link"
      };
      psi_->applyAttachedCollisionObject(attached_obj);
      std::this_thread::sleep_for(200ms);  // wait for planning scene sync

      // Flush stale octomap voxels from camera point cloud
      if (clear_octomap_client_) {
        if (clear_octomap_client_->wait_for_service(std::chrono::seconds(2))) {
          auto req = std::make_shared<std_srvs::srv::Empty::Request>();
          auto future = clear_octomap_client_->async_send_request(req);
          future.wait_for(std::chrono::milliseconds(500));
          RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] OctoMap cleared.");
        }
      }
      std::this_thread::sleep_for(150ms);

      // 7. Retreat / Lift up with payload (+15 cm Z)
      RCLCPP_INFO(node_->get_logger(), "[BT:ArmPickMtc] Step 7: Lifting payload to retreat pose...");
      std::vector<geometry_msgs::msg::Pose> waypoints_up = { chosen_pre_grasp_pose };
      moveit_msgs::msg::RobotTrajectory trajectory_up;
      double fraction_up = arm_group->computeCartesianPath(waypoints_up, 0.005, trajectory_up, false);
      if (fraction_up > 0.5) {
        err = arm_group->execute(trajectory_up);
      } else {
        arm_group->setPoseTarget(chosen_pre_grasp_pose, ik_frame);
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
  clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("/clear_octomap");
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

  // Flush residual octomap voxels from camera to ensure clean start state
  if (clear_octomap_client_) {
    if (clear_octomap_client_->wait_for_service(std::chrono::seconds(2))) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = clear_octomap_client_->async_send_request(req);
      future.wait_for(std::chrono::milliseconds(500));
      RCLCPP_INFO(node_->get_logger(), "[BT:MoveNamedPose] OctoMap cleared.");
    }
  }
  std::this_thread::sleep_for(150ms);

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
  move_group->setStartStateToCurrentState();
  move_group->setPlanningTime(10.0);
  move_group->setNumPlanningAttempts(15);
  move_group->setMaxVelocityScalingFactor(0.25);
  move_group->setMaxAccelerationScalingFactor(0.25);
  move_group->setNamedTarget(named_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  for (int attempt = 1; attempt <= 3 && !success; ++attempt) {
    if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      success = true;
      RCLCPP_INFO(node_->get_logger(),
        "[BT:MoveNamedPose] Trajectory to '%s' planned successfully for '%s' (attempt %d)",
        named_pose.c_str(), arm_name.c_str(), attempt);
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "[BT:MoveNamedPose] Plan attempt %d to '%s' failed for '%s', retrying in 500ms...",
        attempt, named_pose.c_str(), arm_name.c_str());
      std::this_thread::sleep_for(500ms);
    }
  }

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

// ── 5. CartesianRetractNode ──────────────────────────────────────────────────

CartesianRetractNode::CartesianRetractNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
}

BT::NodeStatus CartesianRetractNode::onStart()
{
  std::string arm_name = "arm_1";
  double dx = -0.130;
  double dy = 0.0;
  double dz = 0.0;
  getInput("arm", arm_name);
  getInput("dx", dx);
  getInput("dy", dy);
  getInput("dz", dz);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT:CartesianRetract] Retracting arm '%s' linearly by (dx=%.3f, dy=%.3f, dz=%.3f)",
    arm_name.c_str(), dx, dy, dz);

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_name);
  move_group->setMaxVelocityScalingFactor(0.20);
  move_group->setMaxAccelerationScalingFactor(0.20);

  // Compute target waypoint relative to current end-effector pose
  geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.x += dx;
  target_pose.position.y += dy;
  target_pose.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints = { target_pose };
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group->computeCartesianPath(waypoints, 0.005, trajectory, false);

  if (fraction > 0.5) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[BT:CartesianRetract] Cartesian path fraction: %.2f for '%s'. Executing...",
      fraction, arm_name.c_str());
    execution_future_ = std::async(std::launch::async, [move_group, trajectory]() {
      return move_group->execute(trajectory);
    });
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[BT:CartesianRetract] Cartesian path fraction %.2f < 0.5. Falling back to target pose/named pose...",
      fraction);
    if (arm_name == "arm_1") {
      move_group->setNamedTarget("handover_retract");
    } else {
      geometry_msgs::msg::PoseStamped target_stamped = current_pose;
      target_stamped.pose = target_pose;
      move_group->setPoseTarget(target_stamped);
    }
    moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
    if (move_group->plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      execution_future_ = std::async(std::launch::async, [move_group, fallback_plan]() {
        return move_group->execute(fallback_plan);
      });
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[BT:CartesianRetract] Fallback planning failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CartesianRetractNode::onRunning()
{
  if (execution_future_.wait_for(10ms) == std::future_status::ready) {
    auto res = execution_future_.get();
    if (res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "[BT:CartesianRetract] Retract motion completed successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[BT:CartesianRetract] Retract execution failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void CartesianRetractNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[BT:CartesianRetract] Action halted");
}

// ── 6. TransferOwnershipNode ─────────────────────────────────────────────────

TransferOwnershipNode::TransferOwnershipNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node),
  psi_(nullptr)
{
  auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  arm1_detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm1/detach", qos_tl);
  arm2_attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/arm2/attach", qos_tl);
  planning_scene_diff_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
  clear_octomap_client_ = node_->create_client<std_srvs::srv::Empty>("/clear_octomap");
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

  // 2. Gazebo Physics Handover: Detach Arm 1, Attach Arm 2 (burst with transient_local QoS)
  RCLCPP_INFO(node_->get_logger(), "[BT:TransferOwnership] Sending physics handover burst (arm1 detach + arm2 attach)...");
  std_msgs::msg::Empty empty_msg;
  for (int i = 0; i < 30; ++i) {
    arm1_detach_pub_->publish(empty_msg);
    arm2_attach_pub_->publish(empty_msg);
    std::this_thread::sleep_for(30ms);
  }
  RCLCPP_INFO(node_->get_logger(), "[BT:TransferOwnership] Physics handover burst complete (30 msgs × 30ms)");

  // Delay for physics stabilization and planning scene synchronization
  std::this_thread::sleep_for(500ms);

  // 3. MoveIt Attach to destination link with touch links
  moveit_msgs::msg::AttachedCollisionObject attach_obj;
  attach_obj.link_name = to_link;
  attach_obj.object.id = object_id;
  attach_obj.object.operation = attach_obj.object.ADD;
  attach_obj.touch_links = {
    "<octomap>",
    "arm1_gripper_base_link",
    "arm1_gripper_left_finger",
    "arm1_gripper_right_finger",
    "arm1_gripper_tcp",
    "arm1_wrist_3_link",
    "arm1_tool0",
    "arm2_gripper_base_link",
    "arm2_gripper_left_finger",
    "arm2_gripper_right_finger",
    "arm2_gripper_tcp",
    "arm2_wrist_3_link",
    "arm2_tool0",
    "workcell_base_link",
    "table_link"
  };
  psi_->applyAttachedCollisionObject(attach_obj);

  // Flush residual octomap voxels
  if (clear_octomap_client_) {
    if (clear_octomap_client_->wait_for_service(std::chrono::seconds(2))) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = clear_octomap_client_->async_send_request(req);
      future.wait_for(std::chrono::milliseconds(500));
      RCLCPP_INFO(node_->get_logger(), "[BT:TransferOwnership] OctoMap cleared.");
    }
  }
  std::this_thread::sleep_for(150ms);

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

  factory.registerBuilder<CartesianRetractNode>(
    "CartesianRetract",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<CartesianRetractNode>(name, config, node);
    });

  factory.registerBuilder<TransferOwnershipNode>(
    "TransferOwnership",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<TransferOwnershipNode>(name, config, node);
    });
}

}  // namespace birobot_manipulation
