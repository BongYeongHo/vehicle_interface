#ifndef VEHICLE_INTERFACE__CAN_MESSAGE_RECEIVER_NODE_HPP_
#define VEHICLE_INTERFACE__CAN_MESSAGE_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

// vehicle SCALE_FACTOR  
#define VELOCITY_SCALE_FACTOR 0.01

// steer cal
#define DEG2RAD 0.0174533
#define STEER_ANGLE_SCALE 0.01071

// motor revolution
#define MY_PI 3.141592
#define WHEEL_DIAMETER 0.51 // m
#define GEAR_RATIO 4.5
#define MOTOR_OFFSET 32000

// vehicle status
#define MANULAL 1
#define AUTONOMOUS 3

// gear status


namespace vehicle_interface
{

class CanMessageReceiverNode : public rclcpp::Node
{
public:
  explicit CanMessageReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageReceiverNode();

private:
  // Node components
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr mcu_can_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr motor_can_sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_RPM_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vehicle_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr operation_mode_switch_status_pub_; //JHH_251016

  // vehicle_state
  int gear_status_ = 0;
  bool use_motor_rpm_ = false;
  bool operation_mode_switch_status_ = false;

  // Methods
  void vcu_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg);
  void motor_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg);
};

}  // namespace vehicle_interface

#endif  // VEHICLE_INTERFACE__CAN_MESSAGE_RECEIVER_NODE_HPP_
