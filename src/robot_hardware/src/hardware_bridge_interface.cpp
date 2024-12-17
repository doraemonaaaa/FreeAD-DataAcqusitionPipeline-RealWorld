#include "hardware_manager.hpp"

HardwareManager::HardwareManager()
    : Node("hardware_manager"), hardware_data_(), left_wheel_cmd_(0.0), right_wheel_cmd_(0.0) {
    // 发布期望速度
    expect_speed_pub_ = this->create_publisher<robot_hardware::msg::HardwareData>("hardware_expect_speed_msg", 10);

    // 订阅串口节点发布的实际速度
    real_speed_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
        "hardware_real_speed_msg", 10, std::bind(&HardwareManager::receive_real_speed_callback, this, std::placeholders::_1));

    // 订阅键盘输入的命令消息
    keyboard_cmd_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
        "keyboard_cmd", 10, std::bind(&HardwareManager::receive_keyboard_command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hardware Manager initialized.");
}

void HardwareManager::receive_real_speed_callback(const robot_hardware::msg::HardwareData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    hardware_data_.real_left_speed = msg->real_left_speed;
    hardware_data_.real_right_speed = msg->real_right_speed;
    hardware_data_.real_left_neg_flag = msg->real_left_neg_flag;
    hardware_data_.real_right_neg_flag = msg->real_right_neg_flag;
}

void HardwareManager::receive_keyboard_command_callback(const robot_hardware::msg::HardwareData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    left_wheel_cmd_ = msg->expect_left_speed;
    right_wheel_cmd_ = msg->expect_right_speed;

    RCLCPP_INFO(this->get_logger(), "Keyboard Command Received: Left=%f, Right=%f",
                left_wheel_cmd_, right_wheel_cmd_);
}

std::pair<double, double> HardwareManager::get_wheel_speeds() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {
        hardware_data_.real_left_neg_flag ? -hardware_data_.real_left_speed : hardware_data_.real_left_speed,
        hardware_data_.real_right_neg_flag ? -hardware_data_.real_right_speed : hardware_data_.real_right_speed
    };
}

std::pair<double, double> HardwareManager::get_wheel_commands() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {left_wheel_cmd_, right_wheel_cmd_};
}

void HardwareManager::set_wheel_speeds(double left_speed, double right_speed) {
    auto msg = robot_hardware::msg::HardwareData();
    msg.expect_left_speed = static_cast<int16_t>(left_speed);
    msg.expect_right_speed = static_cast<int16_t>(right_speed);
    msg.expect_left_neg_flag = left_speed < 0 ? 1 : 0;
    msg.expect_right_neg_flag = right_speed < 0 ? 1 : 0;

    expect_speed_pub_->publish(msg);
}
