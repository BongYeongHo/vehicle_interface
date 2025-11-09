#ifndef VEHICLE_INTERFACE__CAN_MESSAGE_SENDER_NODE_HPP_
#define VEHICLE_INTERFACE__CAN_MESSAGE_SENDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <algorithm>

// Longitudinal_Control 
#define SPEEDCMD2SIG 255.0

// Lateral Controller
#define STEERCMD2SIG 255.0
#define MAX_STEER_RAD 0.523

namespace vehicle_interface
{
class CanMessageSenderNode : public rclcpp::Node
{
public:
  explicit CanMessageSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageSenderNode();

private:
  // Node components
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_throttle_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_brake_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_steer_cmd_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mcu_can_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Variables to store command values
  float TC_throttle_output_cmd_ = 0.0;
  float TC_brake_output_cmd_ = 0.0;
  float TC_steer_output_cmd_ = 0.0;
  bool gear_can_ = 0;

  // Methods
  void TCthrottle_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void TCsteer_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void TimerCallback();

};

}  // namespace vehicle_interface

#endif  // VEHICLE_INTERFACE__CAN_MESSAGE_SENDER_NODE_HPP_

