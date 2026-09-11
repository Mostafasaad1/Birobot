#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "birobot_interfaces/action/auto_pick_and_place.hpp"
#include "birobot_manipulation/auto_pick_bt_nodes.hpp"
#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

using namespace std::chrono_literals;

namespace birobot_manipulation
{

class AutoPickCoordinator : public rclcpp::Node
{
public:
  using AutoPickAndPlace = birobot_interfaces::action::AutoPickAndPlace;
  using GoalHandleAutoPick = rclcpp_action::ServerGoalHandle<AutoPickAndPlace>;

  explicit AutoPickCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("auto_pick_coordinator", options)
  {
  }

  void init()
  {
    RCLCPP_INFO(get_logger(), "Starting Birobot Auto Pick & Place Coordinator...");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Register Behavior Tree nodes
    registerBirobotNodes(factory_, shared_from_this(), tf_buffer_);
    registerAutoPickNodes(factory_, shared_from_this(), tf_buffer_);

    // Default BT XML path
    std::string default_xml = "";
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("birobot_manipulation");
      default_xml = pkg_share + "/config/bt_trees/auto_pick_place.xml";
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Could not locate package share: %s", ex.what());
    }

    declare_parameter<std::string>("bt_xml_file", default_xml);
    get_parameter("bt_xml_file", bt_xml_file_);

    RCLCPP_INFO(get_logger(), "Configured BT XML file: %s", bt_xml_file_.c_str());

    action_server_ = rclcpp_action::create_server<AutoPickAndPlace>(
      this,
      "/auto_pick_and_place",
      std::bind(&AutoPickCoordinator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AutoPickCoordinator::handle_cancel, this, std::placeholders::_1),
      std::bind(&AutoPickCoordinator::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Action server online: /auto_pick_and_place");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const AutoPickAndPlace::Goal> /*goal*/)
  {
    RCLCPP_INFO(get_logger(), "Received autonomous pick-and-place goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoPick> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAutoPick> goal_handle)
  {
    std::thread{std::bind(&AutoPickCoordinator::execute_cycle, this, goal_handle)}.detach();
  }

  void execute_cycle(const std::shared_ptr<GoalHandleAutoPick> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "==========================================================");
    RCLCPP_INFO(get_logger(), " TRIGGERED: Autonomous Vision-Guided Multi-Object Cycle   ");
    RCLCPP_INFO(get_logger(), "==========================================================");

    auto result = std::make_shared<AutoPickAndPlace::Result>();
    auto cycle_res = std::make_shared<CycleResult>();
    cycle_res->start_time = std::chrono::steady_clock::now();

    // Re-read bt_xml_file param in case updated
    get_parameter("bt_xml_file", bt_xml_file_);

    BT::Tree tree;
    try {
      tree = factory_.createTreeFromFile(bt_xml_file_);
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to instantiate Behavior Tree from %s: %s", bt_xml_file_.c_str(), ex.what());
      cycle_res->success = false;
      cycle_res->fault_code = "CONFIG_FAULT";
      cycle_res->fault_detail = std::string("Failed to load BT: ") + ex.what();
      result->success = false;
      result->fault_code = cycle_res->fault_code;
      result->fault_detail = cycle_res->fault_detail;
      goal_handle->abort(result);
      return;
    }

    // Connect shared cycle_result into root blackboard
    tree.rootBlackboard()->set("cycle_result", cycle_res);
    auto default_place = parsePoseString("x: 0.3, y: 0.3, z: 0.2, frame_id: world, qw: 1.0");
    tree.rootBlackboard()->set("place_pose", default_place);
    tree.rootBlackboard()->set("x: 0.3, y: 0.3, z: 0.2, frame_id: world, qw: 1.0", default_place);
    tree.rootBlackboard()->set("confidence_threshold", 0.70);

    // Provide feedback callback to blackboard
    FeedbackFn feedback_fn = [goal_handle](
      const std::string & current_phase,
      int32_t queue_remaining,
      const std::string & current_object_id)
    {
      if (goal_handle->is_active()) {
        auto feedback = std::make_shared<AutoPickAndPlace::Feedback>();
        feedback->current_phase = current_phase;
        feedback->queue_remaining = queue_remaining;
        feedback->current_object_id = current_object_id;
        goal_handle->publish_feedback(feedback);
      }
    };
    tree.rootBlackboard()->set("feedback_callback", feedback_fn);

    BT::StdCoutLogger logger(tree);
    RCLCPP_INFO(get_logger(), "Behavior Tree created. Starting execution loop...");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(get_logger(), "Goal canceled by client during execution.");
        cycle_res->success = false;
        cycle_res->fault_code = "CANCELED";
        cycle_res->fault_detail = "Operator canceled goal";
        break;
      }
      status = tree.tickOnce();
      std::this_thread::sleep_for(50ms);
    }

    // Check if fault subtree should run
    if (status != BT::NodeStatus::SUCCESS || !cycle_res->fault_code.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Behavior tree returned %s with fault_code '%s'. Executing FaultHandlerTree...",
        BT::toStr(status).c_str(), cycle_res->fault_code.c_str());

      try {
        BT::Tree fault_tree = factory_.createTree("FaultHandlerTree", tree.rootBlackboard());
        BT::NodeStatus fault_status = BT::NodeStatus::RUNNING;
        while (rclcpp::ok() && fault_status == BT::NodeStatus::RUNNING) {
          fault_status = fault_tree.tickOnce();
          std::this_thread::sleep_for(50ms);
        }
      } catch (const std::exception & ex) {
        RCLCPP_WARN(get_logger(), "Could not execute FaultHandlerTree: %s", ex.what());
      }
    }

    auto now = std::chrono::steady_clock::now();
    cycle_res->cycle_duration_sec = std::chrono::duration<double>(now - cycle_res->start_time).count();

    // Populate action result
    result->success = cycle_res->success;
    result->objects_detected = cycle_res->objects_detected;
    result->objects_queued = cycle_res->objects_queued;
    result->objects_picked = cycle_res->objects_picked;
    result->objects_skipped = cycle_res->objects_skipped;
    result->objects_low_confidence = cycle_res->objects_low_confidence;
    result->fault_code = cycle_res->fault_code;
    result->fault_detail = cycle_res->fault_detail;
    result->cycle_duration_sec = cycle_res->cycle_duration_sec;

    if (cycle_res->success) {
      RCLCPP_INFO(get_logger(), "Goal SUCCEEDED in %.2f seconds", result->cycle_duration_sec);
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Goal ABORTED: fault_code='%s', detail='%s' in %.2f seconds",
        result->fault_code.c_str(), result->fault_detail.c_str(), result->cycle_duration_sec);
      goal_handle->abort(result);
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  BT::BehaviorTreeFactory factory_;
  std::string bt_xml_file_;
  rclcpp_action::Server<AutoPickAndPlace>::SharedPtr action_server_;
};

}  // namespace birobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<birobot_manipulation::AutoPickCoordinator>();
  node->init();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
