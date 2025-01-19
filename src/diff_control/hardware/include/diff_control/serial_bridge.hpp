#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_hardware/msg/motor_data.hpp" // 引入 MotorData 消息

#include <serial/serial.h>

class SerialBridge : public rclcpp::Node
{
public:
    SerialBridge() : Node("serial_bridge_node")
    {
        // 设置串口参数
        serial_port_.setPort("/dev/motor");  // 替换为你的串口设备路径
        serial_port_.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);  // 创建 Timeout 对象,每次串口读写操作的最大等待时间。如果在此时间内没有成功读取或写入数据，操作就会超时。
        serial_port_.setTimeout(timeout);  // 将其传给 setTimeout
    }

    ~SerialBridge() {}

    bool setup(double wheel_radius)
    {
        wheel_radius_ = wheel_radius;
        try
        {
            serial_port_.open();
            if (serial_port_.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
                return false;
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while opening serial port: %s", e.what());
            return false;
        }
    }

    std::tuple<double, double> read_motor_vel()
    {
        double vel_l = 0, vel_r = 0;

        // 确保串口有足够的数据可读取（8字节）
        if (serial_port_.available() >= 8)
        {
            // 读取8个字节数据
            std::vector<uint8_t> incoming_data(8);
            size_t bytes_read = serial_port_.read(incoming_data.data(), 8); // 直接读取到缓冲区

            // 如果读取到的数据符合预期
            if (bytes_read == 8 && incoming_data[0] == 0xFF && incoming_data[7] == 0xAA)
            {
                // 逐字节解析数据
                int16_t speed_l = (incoming_data[1] << 8) | incoming_data[2];  // left wheel speed
                int16_t speed_r = (incoming_data[3] << 8) | incoming_data[4];  // right wheel speed
                uint8_t neg_flag_l = incoming_data[5];  // left wheel direction
                uint8_t neg_flag_r = incoming_data[6];  // right wheel direction

                // 限制最大速度为 10000 RPM
                speed_l = std::min(speed_l, static_cast<int16_t>(10000));
                speed_r = std::min(speed_r, static_cast<int16_t>(10000));

                // 将 RPM 转换为线速度
                vel_l = rpm_to_velocity(speed_l);
                vel_r = rpm_to_velocity(speed_r);

                // 处理方向标志
                vel_l = (neg_flag_l == 1) ? vel_l : -vel_l;
                vel_r = (neg_flag_r == 1) ? vel_r : -vel_r;

                // 调试输出
                //RCLCPP_INFO(this->get_logger(), "Received speeds: left: %.2f m/s, right: %.2f m/s", vel_l, vel_r);
            }
        }

        return std::make_tuple(vel_l, vel_r);
    }


    void set_motor_vel(double vel_l, double vel_r)
    {
        hardware_data_.expect_left_speed_rpm = velocity_to_rpm(std::abs(vel_l));
        hardware_data_.expect_right_speed_rpm = velocity_to_rpm(std::abs(vel_r));
        hardware_data_.expect_left_neg_flag = vel_l >= 0 ? 1 : 0;
        hardware_data_.expect_right_neg_flag = vel_r >= 0 ? 1 : 0;

        std::vector<uint8_t> data;
        data.clear();
        
        data.push_back(0xFF);
        // 将 left_speed_expect 和 right_speed_expect 按高字节和低字节存储
        data.push_back((hardware_data_.expect_left_speed_rpm >> 8) & 0xFF); // 左轮速度高字节
        data.push_back(hardware_data_.expect_left_speed_rpm & 0xFF);        // 左轮速度低字节

        data.push_back((hardware_data_.expect_right_speed_rpm  >> 8) & 0xFF); // 右轮速度高字节
        data.push_back(hardware_data_.expect_right_speed_rpm  & 0xFF);        // 右轮速度低字节

        // 存储方向标志
        data.push_back(hardware_data_.expect_left_neg_flag );  // 左轮负向标志
        data.push_back(hardware_data_.expect_right_neg_flag); // 右轮负向标志
        data.push_back(0xAA);

        ser_write(data);
    }



private:
    serial::Serial serial_port_;  // 串口对象
    robot_hardware::msg::MotorData hardware_data_;  // 硬件数据
    double wheel_radius_;

    // 将 RPM 转换为线速度（米/秒）
    double rpm_to_velocity(std::int16_t rpm) const
    {
        return (rpm * 2.0 * M_PI * wheel_radius_) / 60.0;
    }

    // 将线速度（米/秒）转换为 RPM
    std::int16_t velocity_to_rpm(double velocity_m_per_s) const
    {
        return static_cast<std::int16_t>((velocity_m_per_s * 60.0) / (2.0 * M_PI * wheel_radius_));
    }

    // 写方法，封装数据写入串口
    void ser_write(const std::vector<uint8_t>& data)
    {
        try
        {
            serial_port_.write(data);  // 向串口写入数据
            //RCLCPP_INFO(this->get_logger(), "Data written to serial port: %s", byte_vector_to_hex(data).c_str());
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
        }
    }

    // 辅助方法：将字节数组转换为十六进制字符串
    std::string byte_vector_to_hex(const std::vector<uint8_t>& data)
    {
        std::stringstream ss;
        for (auto byte : data)
        {
            ss << std::hex << static_cast<int>(byte) << " ";
        }
        return ss.str();
    }
};
