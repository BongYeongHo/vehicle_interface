#include "vehicle_interface/can_message_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vehicle_interface::CanMessageReceiverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}