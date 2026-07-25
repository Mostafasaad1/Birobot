#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "birobot_manipulation/mtc_pick_place_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<birobot_manipulation::MtcPickPlaceNode>(options);

  // Transition through lifecycle: configure -> activate
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
