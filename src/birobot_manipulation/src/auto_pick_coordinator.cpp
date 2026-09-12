#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "birobot_manipulation/auto_pick_bt_nodes.hpp"
#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"
#include "birobot_interfaces/action/auto_pick_and_place.hpp"

using namespace std::chrono_literals;

namespace birobot_manipulation {

using AutoPickAndPlace = birobot_interfaces::action::AutoPickAndPlace;
using GoalHandleAutoPickAndPlace = rclcpp_action::ServerGoalHandle<AutoPickAndPlace>;

class AutoPickCoordinator : public rclcpp::Node {
public:
  AutoPickCoordinator() : Node("auto_pick_coordinator") {
    RCLCPP_INFO(this->get_logger(), "Starting Auto Pick & Place Coordinator...");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string default_xml_path = "";
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("birobot_manipulation");
      default_xml_path = pkg_share + "/config/bt_trees/auto_pick_place.xml";
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not locate package share: %s", ex.what());
    }

    this->declare_parameter<std::string>("bt_xml_file", default_xml_path);
    this->get_parameter("bt_xml_file", bt_xml_file_);
    
    // Create action server
    action_server_ = rclcpp_action::create_server<AutoPickAndPlace>(
      this,
      "/auto_pick_and_place",
      std::bind(&AutoPickCoordinator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AutoPickCoordinator::handle_cancel, this, std::placeholders::_1),
      std::bind(&AutoPickCoordinator::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoPickAndPlace::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received goal request for AutoPickAndPlace");
    if (active_goal_) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected: System is already executing a cycle.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoPickAndPlace> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAutoPickAndPlace> goal_handle)
  {
    active_goal_ = goal_handle;
    std::thread{std::bind(&AutoPickCoordinator::execute, this)}.detach();
  }

  void execute()
  {
    RCLCPP_INFO(this->get_logger(), "Executing AutoPickAndPlace cycle...");
    auto result = std::make_shared<AutoPickAndPlace::Result>();
    
    BT::BehaviorTreeFactory factory;
    // Register common nodes (like ArmPickMtcNode) from handover_bt_nodes
    registerBirobotNodes(factory, shared_from_this(), tf_buffer_);
    // Register auto pick specific nodes
    registerAutoPickNodes(factory, shared_from_this(), tf_buffer_);

    BT::Tree tree;
    try {
      tree = factory.createTreeFromFile(bt_xml_file_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create Behavior Tree: %s", ex.what());
      result->success = false;
      result->fault_code = "BT_LOAD_ERROR";
      result->fault_detail = ex.what();
      active_goal_->abort(result);
      active_goal_ = nullptr;
      return;
    }

    // Set up Blackboard: set cycle_result and the fixed place_pose as typed objects
    // so BT.CPP never needs to call convertFromString<> on complex types.
    CycleResult cycle_result;
    cycle_result.start_time = this->now();
    tree.rootBlackboard()->set("cycle_result", cycle_result);

    // Fixed place pose — matches the value in auto_pick_place.xml comment.
    // Operators can edit x/y/z here and relaunch without recompiling.
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "world";
    place_pose.pose.position.x = 0.3;
    place_pose.pose.position.y = 0.3;
    place_pose.pose.position.z = 0.2;
    place_pose.pose.orientation.w = 1.0;
    tree.rootBlackboard()->set("place_pose", place_pose);

    // StdCout Logger for terminal visualization
    BT::StdCoutLogger logger(tree);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate loop_rate(10);
    
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !active_goal_->is_canceling()) {
      status = tree.tickOnce();
      
      // Publish feedback (best-effort: only publish when current_phase is set)
      auto feedback = std::make_shared<AutoPickAndPlace::Feedback>();
      bool phase_ready = tree.rootBlackboard()->get("current_phase", feedback->current_phase);
      if (phase_ready) {
        (void)tree.rootBlackboard()->get("queue_remaining", feedback->queue_remaining);
        (void)tree.rootBlackboard()->get("current_object_id", feedback->current_object_id);
        active_goal_->publish_feedback(feedback);
      }
      
      loop_rate.sleep();
    }

    (void)tree.rootBlackboard()->get("cycle_result", cycle_result);
    
    result->success = cycle_result.success;
    result->objects_detected = cycle_result.objects_detected;
    result->objects_queued = cycle_result.objects_queued;
    result->objects_picked = cycle_result.objects_picked;
    result->objects_skipped = cycle_result.objects_skipped;
    result->objects_low_confidence = cycle_result.objects_low_confidence;
    result->fault_code = cycle_result.fault_code;
    result->fault_detail = cycle_result.fault_detail;
    result->cycle_duration_sec = (this->now() - cycle_result.start_time).seconds();

    if (active_goal_->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled.");
      result->success = false;
      result->fault_code = "CANCELED";
      active_goal_->canceled(result);
    } else if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
      active_goal_->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Goal aborted with status %s", BT::toStr(status).c_str());
      active_goal_->abort(result);
    }
    
    active_goal_ = nullptr;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string bt_xml_file_;
  rclcpp_action::Server<AutoPickAndPlace>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleAutoPickAndPlace> active_goal_;
};

} // namespace birobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<birobot_manipulation::AutoPickCoordinator>();
  
  // Use a MultiThreadedExecutor to allow action server callbacks to run concurrently with execution thread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
