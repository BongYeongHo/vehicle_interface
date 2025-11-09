#include "vehicle_interface/can_message_receiver_node.hpp"

namespace vehicle_interface
{

CanMessageReceiverNode::CanMessageReceiverNode(const rclcpp::NodeOptions & options) : Node("interface_receiver", options)
{
  use_motor_rpm_ = this->declare_parameter("use_wheel_enco", false);    

  mcu_can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/HANA_VCU/from_can_bus", 1, std::bind(&CanMessageReceiverNode::vcu_can_data_callback, this, std::placeholders::_1));
  motor_can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/motor_rpm/from_can_bus", 1, std::bind(&CanMessageReceiverNode::motor_can_data_callback, this, std::placeholders::_1));

  /* To AutowareInterface, TwistController */
  velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/velocity_status", 10);

  motor_RPM_status_pub_ = this->create_publisher<std_msgs::msg::Float64>( 
    "/can_message_receiver/velocity_status", 10);

  steer_status_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/steering_status", 10);
    
  gear_status_pub_=this->create_publisher<std_msgs::msg::Bool>(
    "/can_message_receiver/gear_status", 10);

  vehicle_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/can_message_receiver/vehicle_mode_status", 10);
  
  // JHH
  operation_mode_switch_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/can_message_receiver/operation_mode_switch", 10);

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode initialized");
}

CanMessageReceiverNode::~CanMessageReceiverNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode destroyed");
}

void CanMessageReceiverNode::vcu_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 513) // 201
    {
        int16_t velocity_raw_data = msg->data[5] << 8 | msg->data[4];
        float speed_data = velocity_raw_data * VELOCITY_SCALE_FACTOR; // [mps]

        std_msgs::msg::Float64 vehicle_speed_msg;
        vehicle_speed_msg.data = speed_data;

        this->get_parameter("Vehicle_motor", use_motor_rpm_);
        if(!use_motor_rpm_)
        {
            velocity_status_pub_->publish(vehicle_speed_msg);
        }
    }

    if(msg->id == 273) // 111
    {
        uint16_t steer_raw_data = msg->data[0] + (msg->data[1] << 8); 

        float steer_data = steer_raw_data;
        steer_data -= 5200.f;
        steer_data *= STEER_ANGLE_SCALE;
        steer_data *= DEG2RAD; 
        
        std_msgs::msg::Float64 steering_status_msg;
        steering_status_msg.data = steer_data;
        steer_status_pub_->publish(steering_status_msg);
    }

    if (msg->id == 321) // 141 vehicle_mode (8bit)
    {
        uint8_t vehicle_status = msg->data[4];
        bool vehicle_mode = 0;

        gear_status_ = msg->data[5];
        if(gear_status_ == 0)
        {
            gear_status_ = 2;
        }
        else if(gear_status_ == 1)
        {
            gear_status_ = 20;
        }
        std_msgs::msg::Bool gear_status_msg;
        gear_status_msg.data = gear_status_;
        gear_status_pub_ ->publish(gear_status_msg);

        if (vehicle_status == MANULAL)
        {
            vehicle_mode = false;
        }
        else if (vehicle_status == AUTONOMOUS)
        {
            vehicle_mode = true;
        }

        std_msgs::msg::Bool vehicle_status_msg;
        vehicle_status_msg.data = vehicle_mode;
        vehicle_status_pub_->publish(vehicle_status_msg);
    }

    if(msg->id == 528) // 210
    {
        uint8_t operation_mode_switch_status_data = msg->data[3];

        if(operation_mode_switch_status_data == 1)
        {
          operation_mode_switch_status_ = false;
        }
        else if(operation_mode_switch_status_data == 0)
        {
          operation_mode_switch_status_ = true;
        }
        
        std_msgs::msg::Bool operation_mode_switch_status_msg;
        operation_mode_switch_status_msg.data = operation_mode_switch_status_;
        operation_mode_switch_status_pub_->publish(operation_mode_switch_status_msg);
    }
}

void CanMessageReceiverNode::motor_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 402724847) //  Motor_control id 180117EF
    {
        double motor_rpm_raw = msg->data[7] << 8 | msg->data[6];
        double motor_rpm = motor_rpm_raw - MOTOR_OFFSET;
        
        double vehicle_speed_data = motor_rpm * MY_PI * WHEEL_DIAMETER / GEAR_RATIO;
        vehicle_speed_data /= 60; // [mps]

        std_msgs::msg::Float64 vehicle_motor_rpm_msg;
        vehicle_motor_rpm_msg.data = vehicle_speed_data;

        this->get_parameter("Vehicle_motor", use_motor_rpm_);
        if(!use_motor_rpm_)
        {
            motor_RPM_status_pub_->publish(vehicle_motor_rpm_msg);
        }
    }
}

} // namespace vehicle_interface