#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "birobot_perception/irregular_object_pose_estimator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<birobot_perception::IrregularObjectPoseEstimator>();

  // Auto-configure and auto-activate for seamless launch execution
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
