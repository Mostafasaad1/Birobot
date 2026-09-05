#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "birobot_manipulation/bt_nodes/handover_bt_nodes.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("birobot_bt_coordinator");

  RCLCPP_INFO(node->get_logger(), "Starting Birobot BehaviorTree.CPP v4 Mission Coordinator...");

  // TF2 Buffer & Listener
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Parameter for BT XML path
  std::string default_xml_path = "";
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("birobot_manipulation");
    default_xml_path = pkg_share + "/config/bt_trees/collaborative_handover.xml";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node->get_logger(), "Could not locate package share: %s", ex.what());
  }

  node->declare_parameter<std::string>("bt_xml_file", default_xml_path);
  std::string bt_xml_file;
  node->get_parameter("bt_xml_file", bt_xml_file);

  RCLCPP_INFO(node->get_logger(), "Loading Behavior Tree XML: %s", bt_xml_file.c_str());

  // Factory and Node Registration
  BT::BehaviorTreeFactory factory;
  birobot_manipulation::registerBirobotNodes(factory, node, tf_buffer);

  // Instantiate Behavior Tree
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_file);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Failed to create Behavior Tree: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // StdCout Logger for terminal visualization
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "Behavior Tree loaded successfully. Starting mission execution...");

  // Spinner thread for ROS 2 callbacks
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() {
    executor->spin();
  });

  // Small delay to allow MoveGroup & TF discovery
  std::this_thread::sleep_for(2000ms);

  // Main Tree Execution Loop
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(100ms);
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "==========================================================");
    RCLCPP_INFO(node->get_logger(), " MISSION COMPLETE: Collaborative Handover SUCCESS! ");
    RCLCPP_INFO(node->get_logger(), "==========================================================");
  } else {
    RCLCPP_ERROR(node->get_logger(), "==========================================================");
    RCLCPP_ERROR(node->get_logger(), " MISSION ABORTED: Behavior Tree returned status %s", BT::toStr(status).c_str());
    RCLCPP_ERROR(node->get_logger(), "==========================================================");
  }

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
