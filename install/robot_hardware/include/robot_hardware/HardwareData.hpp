#ifndef ROBOT_HARDWARE_HARDWARE_DATA_HPP
#define ROBOT_HARDWARE_HARDWARE_DATA_HPP

#include <cstdint>

namespace robot_hardware {

// HardwareData 结构体，用于管理机器人的实际速度和期望速度数据
struct HardwareData {
    // 实际速度和标志位
    int16_t real_left_speed;     // 左轮实际速度
    int16_t real_right_speed;    // 右轮实际速度
    uint8_t real_left_neg_flag;  // 左轮负向标志
    uint8_t real_right_neg_flag; // 右轮负向标志

    // 期望速度和标志位
    int16_t expect_left_speed;   // 左轮期望速度
    int16_t expect_right_speed;  // 右轮期望速度
    uint8_t expect_left_neg_flag;// 左轮负向标志
    uint8_t expect_right_neg_flag;// 右轮负向标志

    // 构造函数，将变量初始化为默认值
    HardwareData()
        : real_left_speed(0),
          real_right_speed(0),
          real_left_neg_flag(0),
          real_right_neg_flag(0),
          expect_left_speed(1),
          expect_right_speed(1),
          expect_left_neg_flag(1),
          expect_right_neg_flag(1) {}


    // 重置函数，将所有成员变量重置为默认值
    void reset() {
        real_left_speed = 0;
        real_right_speed = 0;
        real_left_neg_flag = 0;
        real_right_neg_flag = 0;
        expect_left_speed = 1;
        expect_right_speed = 1;
        expect_left_neg_flag = 1;
        expect_right_neg_flag = 1;
    }
};

} // namespace robot_hardware

#endif // ROBOT_HARDWARE_HARDWARE_DATA_HPP
