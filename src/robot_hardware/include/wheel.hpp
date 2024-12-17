#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <cstdint>
#include <string>
#include <cmath>
#include <stdexcept>

class Wheel {
public:
    // 默认构造函数
    Wheel()
        : name_("default"),
          wheel_radius_(0.0),
          position_rad_(0.0),
          cur_rpm_(0.0),
          ex_rpm_(0.0),
          cur_vel_(0.0),
          ex_vel_(0.0) {}

    // 构造函数，初始化轮子名称和参数
    Wheel(const std::string& name, double wheel_radius)
        : name_(name),
          wheel_radius_(wheel_radius),
          position_rad_(0.0),
          cur_rpm_(0.0),
          ex_rpm_(0.0),
          cur_vel_(0.0),
          ex_vel_(0.0) {
        if (wheel_radius <= 0) {
            throw std::invalid_argument("Wheel radius must be greater than zero.");
        }
    }

    // 更新当前转速并计算相应的线速度和位置
    void update_rpm(double rpm, double dt) {
        set_current_rpm(rpm);
        update_position(dt);
    }

    // 更新当前线速度并计算相应的转速和位置
    void update_velocity(double velocity_m_per_s, double dt) {
        set_current_velocity(velocity_m_per_s);
        update_position(dt);
    }

    // 设置期望速度（线速度，单位：米/秒），同时更新期望转速
    void set_expect_velocity(double velocity_m_per_s) {
        ex_vel_ = velocity_m_per_s;
        ex_rpm_ = velocity_to_rpm(velocity_m_per_s);
    }

    // 设置期望转速（转速，单位：RPM），同时更新期望线速度
    void set_expect_rpm(double rpm) {
        ex_rpm_ = rpm;
        ex_vel_ = rpm_to_velocity(rpm);
    }

    // 获取期望速度（线速度，单位：米/秒）
    double get_expect_velocity() const {
        return ex_vel_;
    }

    // 获取期望转速（转速，单位：RPM）
    double get_expect_rpm() const {
        return ex_rpm_;
    }

    // 获取当前速度（单位：米/秒）
    double get_velocity() const {
        return cur_vel_;
    }

    // 获取当前转速（单位：RPM）
    double get_rpm() const {
        return cur_rpm_;
    }

    // 获取当前位置（单位：米）
    double get_position() const {
        return position_rad_ * wheel_radius_;
    }

    // 获取轮子名称
    const std::string& get_name() const {
        return name_;
    }

private:
    // 轮子名称
    std::string name_;

    // 轮子半径（单位：米）
    double wheel_radius_;

    // 当前轮子位置（单位：弧度）
    double position_rad_;

    // 当前转速（单位：RPM）
    double cur_rpm_;

    // 期望转速（单位：RPM）
    double ex_rpm_;

    // 当前线速度（单位：米/秒）
    double cur_vel_;

    // 期望线速度（单位：米/秒）
    double ex_vel_;

    // 将 RPM 转换为线速度（米/秒）
    double rpm_to_velocity(double rpm) const {
        return (rpm * 2.0 * M_PI * wheel_radius_) / 60.0;
    }

    // 将线速度（米/秒）转换为 RPM
    double velocity_to_rpm(double velocity_m_per_s) const {
        return (velocity_m_per_s * 60.0) / (2.0 * M_PI * wheel_radius_);
    }

    // 设置当前转速，并更新当前线速度
    void set_current_rpm(double rpm) {
        cur_rpm_ = rpm;
        cur_vel_ = rpm_to_velocity(rpm);
    }

    // 设置当前线速度，并更新当前转速
    void set_current_velocity(double velocity_m_per_s) {
        cur_vel_ = velocity_m_per_s;
        cur_rpm_ = velocity_to_rpm(velocity_m_per_s);
    }

    // 更新轮子位置
    void update_position(double dt) {
        double angular_velocity_rad_per_s = (cur_rpm_ * 2.0 * M_PI) / 60.0;
        position_rad_ += angular_velocity_rad_per_s * dt;
        // 保持位置在 [0, 2π) 范围内
        position_rad_ = std::fmod(position_rad_, 2.0 * M_PI);
        if (position_rad_ < 0.0) {
            position_rad_ += 2.0 * M_PI;
        }
    }
};

#endif // WHEEL_HPP
