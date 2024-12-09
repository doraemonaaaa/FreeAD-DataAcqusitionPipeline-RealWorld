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

class Wheel {
public:
    // 轮子名称
    std::string name;
    // 当前轮子位置（单位：米）
    double pos;
    // 当前线速度（单位：米/秒）
    double vel;
    // 期望线速度（单位：米/秒）
    double ex_vel;
    // 轮子半径（单位：米）
    double wheel_radius;

    // 默认构造函数
    Wheel()
        : name("default"),
          pos(0.0),
          vel(0.0),
          ex_vel(0.0),
          wheel_radius(0.0),  // wheel_radius放在最后初始化
          rpm(0.0),
          ex_rpm(0.0),
          last_update_time(std::chrono::steady_clock::now()) {}

    // 构造函数，初始化轮子名称和参数
    Wheel(const std::string& name, double wheel_radius)
        : name(name),
          pos(0.0),
          vel(0.0),
          ex_vel(0.0),
          wheel_radius(wheel_radius),  // wheel_radius放在最后初始化
          rpm(0.0),
          ex_rpm(0.0),
          last_update_time(std::chrono::steady_clock::now()) {
        if (wheel_radius <= 0) {
            throw std::invalid_argument("Wheel radius must be greater than zero.");
        }
    }

    // 设置函数，用于初始化轮子名称和半径
    void setup(const std::string& name, double wheel_radius){
        this->name = name;
        this->wheel_radius = wheel_radius;
        last_update_time = std::chrono::steady_clock::now();
    }

    // 更新当前转速并计算相应的线速度和位置
    void update_rpm_and_pos(std::int16_t rpm) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_update_time;
        double dt = elapsed.count();
        set_current_rpm(rpm);
        update_position(dt);
        last_update_time = current_time;
    }

    // 更新当前线速度并计算相应的转速和位置
    void update_velocity_and_pos(double velocity_m_per_s) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_update_time;
        double dt = elapsed.count();
        set_current_velocity(velocity_m_per_s);
        update_position(dt);
        last_update_time = current_time;
    }

    // 设置期望速度（线速度，单位：米/秒），同时更新期望转速
    void set_expect_velocity(double velocity_m_per_s) {
        ex_vel = velocity_m_per_s;
        ex_rpm = velocity_to_rpm(velocity_m_per_s);
    }

    // 设置期望转速（转速，单位：RPM），同时更新期望线速度
    void set_expect_rpm(std::int16_t rpm) {
        ex_rpm = rpm;
        ex_vel = rpm_to_velocity(rpm);
    }

    // 获取期望转速（转速，单位：RPM）
    double get_expect_rpm() const {
        return ex_rpm;
    }

    // 获取当前转速（单位：RPM）
    double get_rpm() const {
        return rpm;
    }

    // 获取当前位置（单位：米）
    double get_position() const {
        return pos;
    }

    // 获取轮子名称
    const std::string& get_name() const {
        return name;
    }

    // 将 RPM 转换为线速度（米/秒）
    double rpm_to_velocity(std::int16_t rpm) const {
        return (rpm * 2.0 * M_PI * wheel_radius) / 60.0;
    }

    // 将线速度（米/秒）转换为 RPM
    std::int16_t velocity_to_rpm(double velocity_m_per_s) const {
        return static_cast<std::int16_t>((velocity_m_per_s * 60.0) / (2.0 * M_PI * wheel_radius));
    }

private:
    // 当前转速（单位：RPM）
    std::int16_t rpm;
    // 期望转速（单位：RPM）
    std::int16_t ex_rpm;
    // 上一次更新时间
    std::chrono::steady_clock::time_point last_update_time;

    // 设置当前转速，并更新当前线速度
    void set_current_rpm(std::int16_t rpm) {
        this->rpm = rpm;
        vel = rpm_to_velocity(rpm);
    }

    // 设置当前线速度，并更新当前转速
    void set_current_velocity(double velocity_m_per_s) {
        vel = velocity_m_per_s;
        rpm = velocity_to_rpm(velocity_m_per_s);
    }

    // 更新轮子位置
    void update_position(double dt) {
        // 使用线速度直接更新位置
        pos += vel * dt;
        // 如果需要限制位置范围，可以添加相应逻辑
    }
};

#endif // WHEEL_HPP
