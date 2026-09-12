// Copyright 2026 Birobot Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <random>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"
#include "birobot_interfaces/srv/randomize_object.hpp"

using namespace std::chrono_literals;

namespace birobot_manipulation {

class BirobotBtCoordinatorNode : public rclcpp::Node
{
public:
  BirobotBtCoordinatorNode()
  : Node("birobot_bt_coordinator"),
    mission_running_(false),
    cancel_requested_(false)
  {
    RCLCPP_INFO(get_logger(), "Initializing Birobot BehaviorTree Mission Coordinator...");

    // TF2 Buffer & Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Default XML path
    std::string default_xml_path = "";
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("birobot_manipulation");
      default_xml_path = pkg_share + "/config/bt_trees/collaborative_handover.xml";
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Could not locate package share: %s", ex.what());
    }

    declare_parameter<std::string>("bt_xml_file", default_xml_path);
    declare_parameter<bool>("auto_start", false);

    get_parameter("bt_xml_file", bt_xml_file_);
    get_parameter("auto_start", auto_start_);

    // Status publisher
    pub_mission_status_ = create_publisher<std_msgs::msg::String>("/birobot/mission_status", 10);

    // Services
    srv_trigger_handover_ = create_service<std_srvs::srv::Trigger>(
      "/birobot/trigger_handover",
      std::bind(&BirobotBtCoordinatorNode::handle_trigger_handover, this, std::placeholders::_1, std::placeholders::_2));

    srv_reset_mission_ = create_service<std_srvs::srv::Trigger>(
      "/birobot/reset_mission",
      std::bind(&BirobotBtCoordinatorNode::handle_reset_mission, this, std::placeholders::_1, std::placeholders::_2));

    srv_randomize_object_ = create_service<birobot_interfaces::srv::RandomizeObject>(
      "/birobot/randomize_object",
      std::bind(&BirobotBtCoordinatorNode::handle_randomize_object, this, std::placeholders::_1, std::placeholders::_2));

    publish_status("IDLE: Ready for mission trigger");
    RCLCPP_INFO(get_logger(), "Coordinator services registered:");
    RCLCPP_INFO(get_logger(), "  - /birobot/trigger_handover (std_srvs/srv/Trigger)");
    RCLCPP_INFO(get_logger(), "  - /birobot/randomize_object (birobot_interfaces/srv/RandomizeObject)");
    RCLCPP_INFO(get_logger(), "  - /birobot/reset_mission (std_srvs/srv/Trigger)");
  }

  void initialize_bt_and_controllers()
  {
    RCLCPP_INFO(get_logger(), "Loading Behavior Tree XML: %s", bt_xml_file_.c_str());

    auto raw_node = shared_from_this();
    birobot_manipulation::registerBirobotNodes(factory_, raw_node, tf_buffer_);

    try {
      tree_ = factory_.createTreeFromFile(bt_xml_file_);
      logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
      RCLCPP_INFO(get_logger(), "Behavior Tree loaded successfully.");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to create Behavior Tree: %s", ex.what());
    }

    // Wait for all arm trajectory and gripper action servers in background
    std::thread([this]() {
      std::vector<std::string> action_servers = {
        "/arm1_joint_trajectory_controller/follow_joint_trajectory",
        "/arm2_joint_trajectory_controller/follow_joint_trajectory",
        "/arm1_gripper_controller/follow_joint_trajectory",
        "/arm2_gripper_controller/follow_joint_trajectory"
      };

      for (const auto & action_name : action_servers) {
        auto client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          shared_from_this(), action_name);
        RCLCPP_INFO(get_logger(), "Waiting for action server '%s'...", action_name.c_str());
        while (rclcpp::ok() && !client->wait_for_action_server(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) return;
        }
        RCLCPP_INFO(get_logger(), "Action server online: '%s'", action_name.c_str());
      }

      RCLCPP_INFO(get_logger(), "Synchronizing MoveGroup controller handles...");
      std::this_thread::sleep_for(3000ms);
      RCLCPP_INFO(get_logger(), "All controller interfaces confirmed ready.");

      if (auto_start_) {
        RCLCPP_INFO(get_logger(), "auto_start is true. Initiating immediate handover mission...");
        start_mission_async();
      }
    }).detach();
  }

  ~BirobotBtCoordinatorNode() override
  {
    cancel_requested_ = true;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void publish_status(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    pub_mission_status_->publish(msg);
  }

  void handle_trigger_handover(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mission_running_.load()) {
      response->success = false;
      response->message = "Mission already in progress.";
      RCLCPP_WARN(get_logger(), "Trigger received but mission is already RUNNING.");
      return;
    }

    bool started = start_mission_async();
    if (started) {
      response->success = true;
      response->message = "Handover mission started successfully.";
    } else {
      response->success = false;
      response->message = "Failed to launch mission worker.";
    }
  }

  void handle_reset_mission(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mission_running_.load()) {
      cancel_requested_ = true;
      response->success = true;
      response->message = "Mission cancellation requested.";
      publish_status("ABORTED: Mission canceled by user");
    } else {
      response->success = true;
      response->message = "System is already IDLE.";
      publish_status("IDLE: Ready for mission trigger");
    }
  }

  void handle_randomize_object(
    const std::shared_ptr<birobot_interfaces::srv::RandomizeObject::Request> request,
    std::shared_ptr<birobot_interfaces::srv::RandomizeObject::Response> response)
  {
    double target_x = 0.0;
    double target_y = 0.0;
    double target_z = 0.10;
    double target_yaw = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::string zone = request->zone;
    if (zone.empty()) {
      zone = "other_side";
    }

    if (request->custom_pose) {
      target_x = request->custom_x;
      target_y = request->custom_y;
      target_yaw = request->custom_yaw;
      bool is_on_table = (-0.80 <= target_x && target_x <= 0.80 && -0.40 <= target_y && target_y <= 0.40);
      if (std::abs(request->custom_z) > 1e-4) {
        target_z = request->custom_z;
      } else {
        target_z = is_on_table ? 0.15 : 0.10;
      }
    } else {
      std::uniform_real_distribution<double> yaw_dist(-1.5708, 1.5708);
      target_yaw = std::round(yaw_dist(gen) * 1000.0) / 1000.0;

      while (true) {
        if (zone == "other_side" || zone == "otherside" || zone == "rear" || zone == "back") {
          std::uniform_real_distribution<double> x_dist(-1.080, -0.900);
          std::uniform_real_distribution<double> y_dist(-0.250, 0.250);
          target_x = x_dist(gen);
          target_y = y_dist(gen);
        } else if (zone == "front" || zone == "infront") {
          std::uniform_real_distribution<double> x_dist(-0.480, -0.150);
          std::uniform_real_distribution<double> y_dist(-0.280, 0.280);
          target_x = x_dist(gen);
          target_y = y_dist(gen);
        } else { // "all"
          std::uniform_real_distribution<double> subzone_choice(0.0, 1.0);
          if (subzone_choice(gen) < 0.70) {
            std::uniform_real_distribution<double> x_dist(-1.080, -0.900);
            std::uniform_real_distribution<double> y_dist(-0.250, 0.250);
            target_x = x_dist(gen);
            target_y = y_dist(gen);
          } else {
            std::uniform_real_distribution<double> x_dist(-0.480, -0.150);
            std::uniform_real_distribution<double> y_dist(-0.280, 0.280);
            target_x = x_dist(gen);
            target_y = y_dist(gen);
          }
        }
        double dist_base = std::hypot(target_x - (-0.60), target_y);
        if (dist_base >= 0.24 && dist_base <= 0.52) {
          break;
        }
      }
      target_x = std::round(target_x * 1000.0) / 1000.0;
      target_y = std::round(target_y * 1000.0) / 1000.0;

      bool is_on_table = (-0.80 <= target_x && target_x <= 0.80 && -0.40 <= target_y && target_y <= 0.40);
      target_z = is_on_table ? 0.15 : 0.10;
    }

    bool is_on_table = (-0.80 <= target_x && target_x <= 0.80 && -0.40 <= target_y && target_y <= 0.40);
    std::string location_desc = is_on_table ? "ON TABLE" : "GROUND (PAST TABLE BORDER)";

    // Set Gazebo pose via gz service
    double cy = std::cos(target_yaw * 0.5);
    double sy = std::sin(target_yaw * 0.5);
    double qx = 0.0;
    double qy = 0.0;
    double qz = sy;
    double qw = cy;

    std::ostringstream cmd_oss;
    cmd_oss << "gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req '"
            << "name: \"irregular_object_1\", position: {x: " << std::fixed << std::setprecision(4) << target_x
            << ", y: " << target_y << ", z: " << target_z << "}, orientation: {x: " << qx << ", y: " << qy
            << ", z: " << qz << ", w: " << qw << "}'";

    RCLCPP_INFO(get_logger(), "Relocating irregular_object_1 to (%.3f, %.3f, %.3f) [%s]...",
      target_x, target_y, target_z, location_desc.c_str());

    int ret = std::system(cmd_oss.str().c_str());
    bool gz_ok = (ret == 0);

    response->success = gz_ok;
    response->x = target_x;
    response->y = target_y;
    response->z = target_z;
    response->yaw = target_yaw;
    response->location_desc = location_desc;
    response->message = gz_ok ? "Object successfully relocated" : "Failed to execute gz service (is Gazebo running?)";
  }

  bool start_mission_async()
  {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    cancel_requested_ = false;
    mission_running_ = true;

    worker_thread_ = std::thread([this]() {
      RCLCPP_INFO(get_logger(), "==========================================================");
      RCLCPP_INFO(get_logger(), " STARTING COLLABORATIVE HANDOVER MISSION ");
      RCLCPP_INFO(get_logger(), "==========================================================");

      publish_status("RUNNING: Starting Behavior Tree execution");

      // Re-create tree fresh for every run
      try {
        tree_ = factory_.createTreeFromFile(bt_xml_file_);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Failed to create Behavior Tree: %s", ex.what());
        publish_status("FAILED: Could not create tree from file");
        mission_running_ = false;
        return;
      }

      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      int tick_count = 0;

      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !cancel_requested_.load()) {
        status = tree_.tickOnce();
        tick_count++;

        if (tick_count % 10 == 0) {
          publish_status("RUNNING: Collaborative Handover in progress...");
        }
        std::this_thread::sleep_for(100ms);
      }

      if (cancel_requested_.load()) {
        tree_.haltTree();
        RCLCPP_WARN(get_logger(), "Mission was CANCELLED by user.");
        publish_status("ABORTED: Mission canceled");
      } else if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(get_logger(), "==========================================================");
        RCLCPP_INFO(get_logger(), " MISSION COMPLETE: Collaborative Handover SUCCESS! ");
        RCLCPP_INFO(get_logger(), "==========================================================");
        publish_status("SUCCESS: Collaborative Handover Complete!");
      } else {
        RCLCPP_ERROR(get_logger(), "==========================================================");
        RCLCPP_ERROR(get_logger(), " MISSION ABORTED: Tree status %s", BT::toStr(status).c_str());
        RCLCPP_ERROR(get_logger(), "==========================================================");
        publish_status("FAILED: Behavior Tree execution failed");
      }

      mission_running_ = false;
    });

    return true;
  }

  std::string bt_xml_file_;
  bool auto_start_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::StdCoutLogger> logger_;

  std::atomic<bool> mission_running_;
  std::atomic<bool> cancel_requested_;
  std::thread worker_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mission_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trigger_handover_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_mission_;
  rclcpp::Service<birobot_interfaces::srv::RandomizeObject>::SharedPtr srv_randomize_object_;
};

} // namespace birobot_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<birobot_manipulation::BirobotBtCoordinatorNode>();
  node->initialize_bt_and_controllers();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
