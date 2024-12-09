#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav_interfaces/msg/wheel_speed.hpp"
#include <serial/serial.h>
#include "diff_control/msg/hardware_data.hpp" // 引入 HardwareData 消息

uint8_t tdata;
uint8_t data[24];
class SerialBridge : public rclcpp::Node
{
    public:
        SerialBridge() : Node("serial_bridge_node")
        {
            // 初始化发布者
            publisher_ = this->create_publisher<nav_interfaces::msg::WheelSpeed>("/serial/wheel_speed_out", 1);
            publisher_in_ = this->create_publisher<nav_interfaces::msg::WheelSpeed>("/serial/wheel_speed_in", 1);

        expect_speed_pub_ = this->create_publisher<robot_hardware::msg::HardwareData>("/serial/hardware_expect_speed_msg", 10);

        real_speed_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
            "/serial/hardware_real_speed_msg", 10, std::bind(&HardwareManager::receive_real_speed_callback, this, std::placeholders::_1));
        }

        ~SerialBridge()
        {
            ser.close();
        }

        nav_interfaces::msg::WheelSpeed wheel_speed_out = nav_interfaces::msg::WheelSpeed();
        // nav_interfaces::msg::WheelSpeed wheel_speed_in = nav_interfaces::msg::WheelSpeed();

        void read_encoder_values(int &val_1, int &val_2)
        {

               
        }

        void set_motor_values(int val_1, int val_2)
        {
           
        }


    private:
        void pub_timer_callback()
        {
            auto message = this->wheel_speed_out;
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f'", message.speed_l, message.speed_r);
            publisher_->publish(message);
        }

        // void sub_topic_callback(nav_interfaces::msg::WheelSpeed::SharedPtr msg) 
        // {
        //     this->wheel_speed_in.speed_l = msg->speed_l;
        //     this->wheel_speed_in.speed_r = msg->speed_r;
        //     // RCLCPP_INFO(this->get_logger(), "Received: '%f' '%f'", msg->speed_l, msg->speed_r);
        // }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_interfaces::msg::WheelSpeed>::SharedPtr publisher_;
        rclcpp::Publisher<nav_interfaces::msg::WheelSpeed>::SharedPtr publisher_in_;
};
