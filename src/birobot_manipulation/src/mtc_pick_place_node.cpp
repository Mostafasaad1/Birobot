#include "birobot_manipulation/mtc_pick_place_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>

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
  declare_parameter<std::string>("arm_group_name", "arm_1");
  declare_parameter<std::string>("hand_group_name", "arm1_hand");
  declare_parameter<std::string>("eef_name", "arm1_ee");
  declare_parameter<std::string>("ik_frame", "arm1_tool0");
  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<double>("max_velocity_scaling", 0.1);
  declare_parameter<double>("max_acceleration_scaling", 0.1);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring MtcPickPlaceNode...");

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto overrides = get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & param : overrides) {
    node_options.append_parameter_override(param.first, param.second);
  }
  node_options.append_parameter_override("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  node_handle_ = rclcpp::Node::make_shared("mtc_helper_node", node_options);

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

  if (node_handle_) {
    helper_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    helper_executor_->add_node(node_handle_);
    helper_thread_ = std::thread([this]() { helper_executor_->spin(); });
  }

  diagnostic_timer_ = create_wall_timer(
    1000ms, std::bind(&MtcPickPlaceNode::publishDiagnostics, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Deactivating MtcPickPlaceNode...");
  diagnostic_timer_.reset();
  if (helper_executor_) {
    helper_executor_->cancel();
    if (helper_thread_.joinable()) {
      helper_thread_.join();
    }
    helper_executor_.reset();
  }
  LifecycleNode::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MtcPickPlaceNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up MtcPickPlaceNode...");
  action_server_.reset();
  diagnostic_pub_.reset();
  node_handle_.reset();
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

  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", eef_name_);
  task.setProperty("hand", hand_group_name_);
  task.setProperty("ik_frame", ik_frame_);

  auto sampling_planner = node_handle_ ?
    std::make_shared<mtc::solvers::PipelinePlanner>(node_handle_, "ompl", "ompl_interface/OMPLPlanner") :
    std::make_shared<mtc::solvers::PipelinePlanner>(rclcpp::Node::make_shared("mtc_fallback_node"), "ompl", "ompl_interface/OMPLPlanner");
  sampling_planner->setMaxVelocityScalingFactor(max_velocity_scaling_);
  sampling_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(max_velocity_scaling_);
  cartesian_planner->setStepSize(0.005);

  // Stage 1: Current State (Forward)
  auto stage_current = std::make_unique<mtc::stages::CurrentState>("Current State");
  auto stage_current_ptr = stage_current.get();
  task.add(std::move(stage_current));

  // Stage 2: Allow Collision between hand/gripper/arm and target object
  if (!object_id.empty()) {
    auto stage_allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("Allow Collision");
    stage_allow_collision->allowCollisions(object_id, *task.getRobotModel()->getJointModelGroup(arm_group_name_), true);
    stage_allow_collision->allowCollisions(object_id, *task.getRobotModel()->getJointModelGroup(hand_group_name_), true);
    task.add(std::move(stage_allow_collision));
  }

  // Stage 3: Connect (Links forward Current State to backward Approach)
  auto stage_connect = std::make_unique<mtc::stages::Connect>(
    "Connect",
    mtc::stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
  stage_connect->setTimeout(10.0);
  stage_connect->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_connect));

  // Stage 4: Approach (Backward propagation from ComputeIK)
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("Approach", cartesian_planner);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->setIKFrame(ik_frame_);
  geometry_msgs::msg::Vector3Stamped approach_vec;
  approach_vec.header.frame_id = world_frame_;
  approach_vec.vector.z = -1.0;  // Move down along -Z in world frame
  stage_approach->setDirection(approach_vec);
  stage_approach->setMinMaxDistance(0.0, 0.10);
  task.add(std::move(stage_approach));

  // Stage 5: Target Pose IK Generator (Links backward approach to forward attach/retreat)
  auto stage_pose = std::make_unique<mtc::stages::GeneratePose>("Generate Target Pose");
  stage_pose->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_pose->setPose(target_pose);
  stage_pose->setMonitoredStage(stage_current_ptr);

  auto stage_ik = std::make_unique<mtc::stages::ComputeIK>("Grasp Pose IK", std::move(stage_pose));
  stage_ik->setMaxIKSolutions(16);
  stage_ik->setMinSolutionDistance(0.05);
  stage_ik->setIKFrame(ik_frame_);
  stage_ik->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  stage_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  stage_ik->properties().set("ignore_collisions", true);
  task.add(std::move(stage_ik));

  // Stage 6: Attach Object (Forward propagation)
  if (!object_id.empty()) {
    auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach");
    stage_attach->attachObject(object_id, ik_frame_);
    stage_attach->allowCollisions(object_id, true);
    task.add(std::move(stage_attach));
  }

  // Stage 7: Retreat (Forward propagation)
  auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("Retreat", cartesian_planner);
  stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_retreat->setIKFrame(ik_frame_);
  geometry_msgs::msg::Vector3Stamped retreat_vec;
  retreat_vec.header.frame_id = world_frame_;
  retreat_vec.vector.z = 0.10;  // Move up along +Z in world
  stage_retreat->setDirection(retreat_vec);
  stage_retreat->setMinMaxDistance(0.001, 0.15);
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

    if (!goal->object_id.empty()) {
      moveit_msgs::msg::PlanningScene scene_msg;
      scene_msg.is_diff = true;

      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = goal->target_pose.header.frame_id.empty() ? world_frame_ : goal->target_pose.header.frame_id;
      object.id = goal->object_id;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions = {0.05, 0.05, 0.05};

      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(goal->target_pose.pose);
      object.operation = object.ADD;

      scene_msg.world.collision_objects.push_back(object);

      auto robot_model = task.getRobotModel();
      if (robot_model) {
        std::vector<std::string> all_names;
        all_names.push_back(goal->object_id);
        for (const auto & name : robot_model->getLinkModelNamesWithCollisionGeometry()) {
          all_names.push_back(name);
        }

        scene_msg.allowed_collision_matrix.entry_names = all_names;
        scene_msg.allowed_collision_matrix.entry_values.resize(all_names.size());
        for (size_t i = 0; i < all_names.size(); ++i) {
          scene_msg.allowed_collision_matrix.entry_values[i].enabled.resize(all_names.size(), true);
        }
      }

      moveit::planning_interface::PlanningSceneInterface psi;
      psi.applyPlanningScene(scene_msg);
    }

    task.init();
  } catch (const moveit::task_constructor::InitStageException & ex) {
    current_state_ = TaskExecutionState::FAILED;
    publishFeedback(goal_handle, "Connect", "FAILED");
    std::ostringstream ss;
    ss << ex;
    RCLCPP_ERROR_STREAM(get_logger(), "MTC InitStageException: " << ss.str());
    result->success = false;
    result->error_message = std::string("Task creation exception: ") + ss.str();
    goal_handle->abort(result);
    return;
  } catch (const std::exception & ex) {
    current_state_ = TaskExecutionState::FAILED;
    publishFeedback(goal_handle, "Connect", "FAILED");
    RCLCPP_ERROR_STREAM(get_logger(), "MTC Exception: " << ex.what());
    result->success = false;
    result->error_message = std::string("Task creation exception: ") + ex.what();
    goal_handle->abort(result);
    return;
  }

  if (task.plan(5) != moveit::core::MoveItErrorCode::SUCCESS || task.solutions().empty()) {
    current_state_ = TaskExecutionState::IDLE;
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
    current_state_ = TaskExecutionState::IDLE;
    publishFeedback(goal_handle, "Retreat", "FAILED");
    RCLCPP_ERROR(get_logger(), "Collision detected or trajectory execution failed");
    result->success = false;
    result->error_message = "Collision detected during approach.";
    goal_handle->abort(result);
    return;
  }

  publishFeedback(goal_handle, "Retreat", "SUCCESS");
  current_state_ = TaskExecutionState::IDLE;

  result->success = true;
  result->error_message = "";
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Pick and Place task executed successfully");
}

}  // namespace birobot_manipulation
