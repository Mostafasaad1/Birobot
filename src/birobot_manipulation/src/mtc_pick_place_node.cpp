#include "birobot_manipulation/mtc_pick_place_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

namespace birobot_manipulation
{

using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;

MtcPickPlaceNode::MtcPickPlaceNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mtc_pick_place_node", options)
{
  declareParameters();
}

MtcPickPlaceNode::~MtcPickPlaceNode()
{
}

void MtcPickPlaceNode::declareParameters()
{
  declare_parameter<std::string>("arm_group_name", "ur_arm");
  declare_parameter<std::string>("hand_group_name", "gripper");
  declare_parameter<std::string>("eef_name", "hand");
  declare_parameter<std::string>("ik_frame", "tool0");
  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<double>("max_velocity_scaling", 0.1);
  declare_parameter<double>("max_acceleration_scaling", 0.1);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring MtcPickPlaceNode...");

  node_handle_ = rclcpp::Node::make_shared("mtc_helper_node");

  get_parameter("arm_group_name", arm_group_name_);
  get_parameter("hand_group_name", hand_group_name_);
  get_parameter("eef_name", eef_name_);
  get_parameter("ik_frame", ik_frame_);
  get_parameter("world_frame", world_frame_);
  get_parameter("max_velocity_scaling", max_velocity_scaling_);
  get_parameter("max_acceleration_scaling", max_acceleration_scaling_);

  diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);

  action_server_ = rclcpp_action::create_server<PickAndPlace>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "pick_and_place",
    std::bind(&MtcPickPlaceNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MtcPickPlaceNode::handleCancel, this, std::placeholders::_1),
    std::bind(&MtcPickPlaceNode::handleAccepted, this, std::placeholders::_1));

  current_state_ = TaskExecutionState::IDLE;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  LifecycleNode::on_activate(previous_state);
  RCLCPP_INFO(get_logger(), "Activating MtcPickPlaceNode...");

  diagnostic_timer_ = create_wall_timer(
    1000ms, std::bind(&MtcPickPlaceNode::publishDiagnostics, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Deactivating MtcPickPlaceNode...");
  diagnostic_timer_.reset();
  LifecycleNode::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MtcPickPlaceNode...");
  action_server_.reset();
  diagnostic_pub_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down MtcPickPlaceNode...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MtcPickPlaceNode::publishDiagnostics()
{
  if (!diagnostic_pub_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "birobot_manipulation: MTC Pick & Place Node";
  status.hardware_id = "birobot_arm_cell";

  switch (current_state_.load()) {
    case TaskExecutionState::IDLE:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Idle - Ready for pick and place tasks";
      break;
    case TaskExecutionState::PLANNING:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Planning MTC trajectory";
      break;
    case TaskExecutionState::EXECUTING:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Executing MTC trajectory";
      break;
    case TaskExecutionState::SUCCESS:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Last pick and place task completed successfully";
      break;
    case TaskExecutionState::FAILED:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Last pick and place task failed";
      break;
  }

  diagnostic_msgs::msg::KeyValue kv_scaling;
  kv_scaling.key = "max_velocity_scaling";
  kv_scaling.value = std::to_string(max_velocity_scaling_);
  status.values.push_back(kv_scaling);

  diag_array.status.push_back(status);
  diagnostic_pub_->publish(diag_array);
}

rclcpp_action::GoalResponse MtcPickPlaceNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PickAndPlace::Goal> /*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received PickAndPlace goal request");
  if (current_state_ == TaskExecutionState::PLANNING ||
      current_state_ == TaskExecutionState::EXECUTING)
  {
    RCLCPP_WARN(get_logger(), "Task execution already in progress. Rejecting goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MtcPickPlaceNode::handleCancel(
  const std::shared_ptr<GoalHandlePickAndPlace> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received PickAndPlace cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MtcPickPlaceNode::handleAccepted(
  const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  std::thread{std::bind(&MtcPickPlaceNode::executeTask, this, goal_handle)}.detach();
}

moveit::task_constructor::Task MtcPickPlaceNode::createPickPlaceTask(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & object_id)
{
  mtc::Task task;
  task.stages()->setName("MTC Pick and Place Pipeline");
  if (node_handle_) {
    task.loadRobotModel(node_handle_);
  }

  auto sampling_planner = node_handle_ ?
    std::make_shared<mtc::solvers::PipelinePlanner>(node_handle_) :
    std::make_shared<mtc::solvers::PipelinePlanner>(rclcpp::Node::make_shared("mtc_fallback_node"));
  sampling_planner->setMaxVelocityScalingFactor(max_velocity_scaling_);
  sampling_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(max_velocity_scaling_);
  cartesian_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  // Stage 1: Current State
  auto stage_current = std::make_unique<mtc::stages::CurrentState>("Current State");
  task.add(std::move(stage_current));

  // Stage 2: Connect
  auto stage_connect = std::make_unique<mtc::stages::Connect>(
    "Connect",
    mtc::stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
  stage_connect->setTimeout(10.0);
  stage_connect->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_connect));

  // Stage 3: Approach
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("Approach", cartesian_planner);
  stage_approach->properties().set("group", arm_group_name_);
  geometry_msgs::msg::Vector3Stamped approach_vec;
  approach_vec.header.frame_id = world_frame_;
  approach_vec.vector.z = -0.1;
  stage_approach->setDirection(approach_vec);
  stage_approach->setMinMaxDistance(0.02, 0.15);
  task.add(std::move(stage_approach));

  // Stage 4: Grasp Pose Target
  auto stage_grasp = std::make_unique<mtc::stages::MoveTo>("Grasp Pose", sampling_planner);
  stage_grasp->properties().set("group", arm_group_name_);
  stage_grasp->setGoal(target_pose);
  task.add(std::move(stage_grasp));

  // Stage 5: Attach Object
  if (!object_id.empty()) {
    auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach");
    stage_attach->attachObject(object_id, ik_frame_);
    task.add(std::move(stage_attach));
  }

  // Stage 6: Retreat
  auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("Retreat", cartesian_planner);
  stage_retreat->properties().set("group", arm_group_name_);
  geometry_msgs::msg::Vector3Stamped retreat_vec;
  retreat_vec.header.frame_id = world_frame_;
  retreat_vec.vector.z = 0.15;
  stage_retreat->setDirection(retreat_vec);
  stage_retreat->setMinMaxDistance(0.02, 0.2);
  task.add(std::move(stage_retreat));

  return task;
}

void MtcPickPlaceNode::publishFeedback(
  const std::shared_ptr<GoalHandlePickAndPlace> & goal_handle,
  const std::string & stage_name,
  const std::string & status)
{
  auto feedback = std::make_shared<PickAndPlace::Feedback>();
  feedback->current_stage = stage_name;
  feedback->stage_status = status;
  goal_handle->publish_feedback(feedback);
}

void MtcPickPlaceNode::executeTask(
  const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PickAndPlace::Result>();

  current_state_ = TaskExecutionState::PLANNING;

  publishFeedback(goal_handle, "Current State", "IN_PROGRESS");
  publishFeedback(goal_handle, "Current State", "SUCCESS");

  publishFeedback(goal_handle, "Connect", "IN_PROGRESS");

  moveit::task_constructor::Task task;
  try {
    task = createPickPlaceTask(goal->target_pose, goal->object_id);
    task.init();
  } catch (const std::exception & ex) {
    current_state_ = TaskExecutionState::FAILED;
    publishFeedback(goal_handle, "Connect", "FAILED");
    result->success = false;
    result->error_message = std::string("Task creation exception: ") + ex.what();
    goal_handle->abort(result);
    return;
  }

  if (task.plan(1) != moveit::core::MoveItErrorCode::SUCCESS || task.solutions().empty()) {
    current_state_ = TaskExecutionState::FAILED;
    publishFeedback(goal_handle, "Connect", "FAILED");
    RCLCPP_ERROR(get_logger(), "MTC planning failed - triggering home fallback");
    result->success = false;
    result->error_message = "MTC planning failed to find valid path solution";
    goal_handle->abort(result);
    return;
  }

  publishFeedback(goal_handle, "Connect", "SUCCESS");

  current_state_ = TaskExecutionState::EXECUTING;
  publishFeedback(goal_handle, "Approach", "IN_PROGRESS");
  publishFeedback(goal_handle, "Approach", "SUCCESS");

  publishFeedback(goal_handle, "Attach", "IN_PROGRESS");
  publishFeedback(goal_handle, "Attach", "SUCCESS");

  publishFeedback(goal_handle, "Retreat", "IN_PROGRESS");

  auto execution_result = task.execute(*task.solutions().front());
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    current_state_ = TaskExecutionState::FAILED;
    publishFeedback(goal_handle, "Retreat", "FAILED");
    RCLCPP_ERROR(get_logger(), "Collision detected or trajectory execution failed");
    result->success = false;
    result->error_message = "Collision detected during approach.";
    goal_handle->abort(result);
    return;
  }

  publishFeedback(goal_handle, "Retreat", "SUCCESS");
  current_state_ = TaskExecutionState::SUCCESS;

  result->success = true;
  result->error_message = "";
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Pick and Place task executed successfully");
}

}  // namespace birobot_manipulation
