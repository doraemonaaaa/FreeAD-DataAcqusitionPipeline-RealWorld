#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>

// 定义Wheel类，用于表示差速驱动机器人的轮子
class Wheel
{
    public:

    // 轮子的名称
    std::string name = ""; 
    
    // 编码器计数值
    int enc = 0; 
    
    // 轮子接收到的命令速度（单位可能为弧度/秒或其他）
    double cmd = 0; 
    
    // 当前的轮子位置（可能是累计的旋转角度）
    double pos = 0; 
    
    // 当前的轮子速度（单位可能是弧度/秒）
    double vel = 0; 
    
    // 每个编码器计数对应的弧度值
    double rads_per_count = 0; 

    // 默认构造函数
    Wheel() = default;

    // 带参数的构造函数，初始化轮子的名称和编码器每圈计数
    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    // 设置轮子的名称和每圈编码器计数
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name; // 设置轮子名称
      rads_per_count = (2*M_PI)/counts_per_rev; // 计算每计数对应的弧度值
    }

    // 根据编码器计数计算轮子的累计旋转角度（单位：弧度）
    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

};

#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
