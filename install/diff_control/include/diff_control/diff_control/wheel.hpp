#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <cstdint>
#include <string>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

class Wheel {
public:
    // 轮子名称
    std::string name_;
    // 当前轮子位置（单位：米）
    double pos_;
    // 当前线速度（单位：米/秒）
    double vel_;
    // 期望线速度（单位：米/秒）
    double ex_vel_;
    // 轮子半径（单位：米）
    double wheel_radius_;

    // 默认构造函数
    Wheel()
        : name_("default"),
          pos_(0.0),
          vel_(0.0),
          ex_vel_(0.0),
          wheel_radius_(0.0),  // wheel_radius放在最后初始化
          rpm(0.0),
          ex_rpm(0.0),
          last_update_time(rclcpp::Clock().now()) {}

    // 构造函数，初始化轮子名称和参数
    Wheel(const std::string& name, double wheel_radius)
        : name_(name),
          pos_(0.0),
          vel_(0.0),
          ex_vel_(0.0),
          wheel_radius_(wheel_radius),  // wheel_radius放在最后初始化
          rpm(0.0),
          ex_rpm(0.0),
          last_update_time(rclcpp::Clock().now()) {
        if (wheel_radius_ <= 0) {
            throw std::invalid_argument("Wheel radius must be greater than zero.");
        }
    }

    // 设置函数，用于初始化轮子名称和半径
    void setup(const std::string& name_, double wheel_radius_) {
        this->name_ = name_;
        this->wheel_radius_ = wheel_radius_;
        last_update_time = rclcpp::Clock().now();  // 修正赋值
    }

    // 更新当前转速并计算相应的线速度和位置
    void update_rpm_and_pos(std::int16_t rpm) {
        auto current_time = rclcpp::Clock().now();
        double dt = (current_time - last_update_time).seconds();  // 使用ROS时间计算时间差
        set_current_rpm(rpm);
        update_position(dt);
        last_update_time = current_time;
    }

    // 更新当前线速度并计算相应的转速和位置
    void update_velocity_and_pos(double velocity_m_per_s) {
        auto current_time = rclcpp::Clock().now();
        double dt = (current_time - last_update_time).seconds();  // 使用ROS时间计算时间差
        set_current_velocity(velocity_m_per_s);
        update_position(dt);
        last_update_time = current_time;
        // rclcpp::get_logger("logger_name"); // 获取logger对象
        // RCLCPP_INFO(rclcpp::get_logger("logger_name"), "速度：%f,更新位置: %f, 时差:%f, 轮子半径: %f", velocity_m_per_s, pos_, dt, wheel_radius_);

    }

    // 设置期望速度（线速度，单位：米/秒），同时更新期望转速
    void set_expect_velocity(double velocity_m_per_s) {
        ex_vel_ = velocity_m_per_s;
        ex_rpm = velocity_to_rpm(velocity_m_per_s);
    }

    // 设置期望转速（转速，单位：RPM），同时更新期望线速度
    void set_expect_rpm(std::int16_t rpm) {
        ex_rpm = rpm;
        ex_vel_ = rpm_to_velocity(rpm);
    }

    // 将 RPM 转换为线速度（米/秒）
    double rpm_to_velocity(std::int16_t rpm) const {
        return (rpm * 2.0 * M_PI * wheel_radius_) / 60.0;
    }

    // 将线速度（米/秒）转换为 RPM
    std::int16_t velocity_to_rpm(double velocity_m_per_s) const {
        return static_cast<std::int16_t>((velocity_m_per_s * 60.0) / (2.0 * M_PI * wheel_radius_));
    }

private:
    // 当前转速（单位：RPM）
    std::int16_t rpm;
    // 期望转速（单位：RPM）
    std::int16_t ex_rpm;
    // 上一次更新时间
    rclcpp::Time last_update_time;

    // 设置当前转速，并更新当前线速度
    void set_current_rpm(std::int16_t rpm) {
        this->rpm = rpm;
        vel_ = rpm_to_velocity(rpm);
    }

    // 设置当前线速度，并更新当前转速
    void set_current_velocity(double velocity_m_per_s) {
        vel_ = velocity_m_per_s;
        rpm = velocity_to_rpm(velocity_m_per_s);
    }

    // 更新轮子位置
    void update_position(double dt) {
        // 使用线速度直接更新位置
        pos_ += vel_ * dt;
    }
};

#endif // WHEEL_HPP
