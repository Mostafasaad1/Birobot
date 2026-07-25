#ifndef BIROBOT_MANIPULATION__MTC_PICK_PLACE_NODE_HPP_
#define BIROBOT_MANIPULATION__MTC_PICK_PLACE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "birobot_interfaces/action/pick_and_place.hpp"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace birobot_manipulation
{

using PickAndPlace = birobot_interfaces::action::PickAndPlace;
using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

enum class TaskExecutionState
{
  IDLE,
  PLANNING,
  EXECUTING,
  SUCCESS,
  FAILED
};

class MtcPickPlaceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MtcPickPlaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MtcPickPlaceNode();

  // Lifecycle transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  // MTC Task builder
  moveit::task_constructor::Task createPickPlaceTask(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & object_id);

  // Diagnostic status getter
  TaskExecutionState getExecutionState() const { return current_state_; }

private:
  // Action Handlers
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAndPlace::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

  void handleAccepted(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

  void executeTask(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);

  void publishFeedback(
    const std::shared_ptr<GoalHandlePickAndPlace> & goal_handle,
    const std::string & stage_name,
    const std::string & status);

  void publishDiagnostics();
  void declareParameters();

  // Parameters
  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string eef_name_;
  std::string ik_frame_;
  std::string world_frame_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;

  // State & ROS Infrastructure
  rclcpp::Node::SharedPtr node_handle_;
  std::atomic<TaskExecutionState> current_state_{TaskExecutionState::IDLE};
  rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace birobot_manipulation

#endif  // BIROBOT_MANIPULATION__MTC_PICK_PLACE_NODE_HPP_
