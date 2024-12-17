#ifndef HARDWARE_MANAGER_HPP
#define HARDWARE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include "robot_hardware/msg/hardware_data.hpp" // 引入 HardwareData 消息类型
#include <sensor_msgs/msg/image.hpp>           // 引入 ROS 传感器消息类型
#include <opencv2/opencv.hpp>                  // 引入 OpenCV
#include <cv_bridge/cv_bridge.h>               // 引入 cv_bridge

class HardwareManager : public rclcpp::Node {
public:
    HardwareManager(); // 构造函数

private:
    // 成员变量
    robot_hardware::msg::HardwareData hardware_data_;  // 硬件数据
    rclcpp::Publisher<robot_hardware::msg::HardwareData>::SharedPtr expect_speed_pub_; // 发布期望速度
    rclcpp::Subscription<robot_hardware::msg::HardwareData>::SharedPtr real_speed_sub_; // 订阅实际速度
    rclcpp::Subscription<robot_hardware::msg::HardwareData>::SharedPtr keyboard_cmd_sub_; // 订阅键盘命令
    rclcpp::TimerBase::SharedPtr feedback_timer_; // 定时器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // 图像订阅器

    // 回调函数
    void receive_real_speed_callback(const robot_hardware::msg::HardwareData::SharedPtr msg); // 实际速度回调
    void receive_keyboard_command_callback(const robot_hardware::msg::HardwareData::SharedPtr msg); // 键盘命令回调
    void publish_expect_speed(); // 定期发布期望速度
    void usb_cam_image_callback(const sensor_msgs::msg::Image::SharedPtr msg); // 图像数据回调
};

#endif // HARDWARE_MANAGER_HPP
