#include "vehicle_interface/can_message_sender_node.hpp"

namespace vehicle_interface
{

using namespace std::chrono_literals;

CanMessageSenderNode::CanMessageSenderNode(const rclcpp::NodeOptions & options) : Node("can_message_sender", options)
{
  TC_throttle_cmd_ = this->create_subscription<std_msgs::msg::Float64>("/twist_controller/output/throttle_cmd", 1, std::bind(&CanMessageSenderNode::TCthrottle_callback, this, std::placeholders::_1));
  TC_brake_cmd_ = this->create_subscription<std_msgs::msg::Float64>("/twist_controller/output/brake_cmd", 1, std::bind(&CanMessageSenderNode::TCbrake_callback, this, std::placeholders::_1));
  TC_steer_cmd_ = this->create_subscription<std_msgs::msg::Float64>("/twist_controller/output/steer_cmd", 1, std::bind(&CanMessageSenderNode::TCsteer_callback, this, std::placeholders::_1));

  mcu_can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/HANA_VCU/to_can_bus", 1);
  
  timer_ = this->create_wall_timer(10ms,std::bind(&CanMessageSenderNode::TimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode initialized");
}

CanMessageSenderNode::~CanMessageSenderNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode destroyed");
}

void CanMessageSenderNode::TCthrottle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    TC_throttle_output_cmd_ = msg->data;
}
void CanMessageSenderNode::TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    TC_brake_output_cmd_ = msg->data;
}
void CanMessageSenderNode::TCsteer_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    TC_steer_output_cmd_ = msg->data;
}

void CanMessageSenderNode::TimerCallback()
{
    // Throttle/Brake/Gear 
    uint8_t throttle_can = static_cast<uint8_t>(std::clamp(TC_throttle_output_cmd_ * SPEEDCMD2SIG, 0.0, 255.0));
    uint8_t brake_can    = static_cast<uint8_t>(std::clamp(TC_brake_output_cmd_    * SPEEDCMD2SIG, 0.0, 255.0));
   
    can_msgs::msg::Frame can_msg_drive;
    can_msg_drive.id = 320;
    can_msg_drive.dlc = 5;
    can_msg_drive.data[0] = throttle_can;
    can_msg_drive.data[1] = 0;
    can_msg_drive.data[2] = brake_can;
    can_msg_drive.data[3] = 1;
    can_msg_drive.data[4] = gear_can_;
    mcu_can_pub_->publish(can_msg_drive);

    // Steering
    int16_t steer_can = static_cast<int16_t>((TC_steer_output_cmd_ / MAX_STEER_RAD) * STEERCMD2SIG);
    uint8_t steer_can_upper = (uint8_t)((steer_can & 0xFF00) >> 8);
    uint8_t steer_can_lower = (uint8_t)( steer_can & 0x00FF);

    can_msgs::msg::Frame can_msg_steer;
    can_msg_steer.id = 666;
    can_msg_steer.dlc = 2;
    can_msg_steer.data[0] = steer_can_upper;
    can_msg_steer.data[1] = steer_can_lower;
    mcu_can_pub_->publish(can_msg_steer);
}

} // namespace vehicle_interface