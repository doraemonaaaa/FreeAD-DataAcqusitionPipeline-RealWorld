#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "robot_hardware/msg/hardware_data.hpp" // 引入 HardwareData 消息

/*
Note: 
    test:read OK
         write 
*/
class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        // 设置串口参数
        serial_port_.setPort("/dev/motor");  // 替换为你的串口设备路径
        serial_port_.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);  // 创建 Timeout 对象,每次串口读写操作的最大等待时间。如果在此时间内没有成功读取或写入数据，操作就会超时。
        serial_port_.setTimeout(timeout);  // 将其传给 setTimeout

        try
        {
            serial_port_.open();
            if (serial_port_.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while opening serial port: %s", e.what());
            return;
        }

        // 创建发布者反馈数据到hardware_manager
        publisher_ = this->create_publisher<robot_hardware::msg::HardwareData>("/serial/hardware_real_speed_msg", 10);

        // 创建一个订阅者用于接收速度数据
        expect_speed_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
            "/serial/hardware_expect_speed_msg", 10,
            std::bind(&SerialNode::expect_speed_sub_callback, this, std::placeholders::_1)
        );

        // 创建定时器来定期读取串口数据
        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 接受频率比32多一点，40hz -> 50hz
            std::bind(&SerialNode::timer_ser_read_callback, this));

        // 设置定时器，使用 lambda 来传递数据
        watch_dog_write_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                ser_write(std::vector<uint8_t>{0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC});  // 定期feed dog
            }
        );

    } 

private:
    serial::Serial serial_port_;  // 串口对象
    rclcpp::Publisher<robot_hardware::msg::HardwareData>::SharedPtr publisher_;  // 发布者
    rclcpp::Subscription<robot_hardware::msg::HardwareData>::SharedPtr expect_speed_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;  // 定时器，用于定期读取串口数据
    rclcpp::TimerBase::SharedPtr watch_dog_write_timer_;  // 定时器，用于定期发送看门狗信息

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

    void expect_speed_sub_callback(const robot_hardware::msg::HardwareData::SharedPtr msg){
        std::vector<uint8_t> data;
        data.clear();
        
        data.push_back(0xFF);
        // 将 left_speed_expect 和 right_speed_expect 按高字节和低字节存储
        data.push_back((msg->expect_left_speed_rpm >> 8) & 0xFF); // 左轮速度高字节
        data.push_back(msg->expect_left_speed_rpm & 0xFF);        // 左轮速度低字节

        data.push_back((msg->expect_right_speed_rpm >> 8) & 0xFF); // 右轮速度高字节
        data.push_back(msg->expect_right_speed_rpm & 0xFF);        // 右轮速度低字节

        // 存储方向标志
        data.push_back(msg->expect_left_neg_flag);  // 左轮负向标志
        data.push_back(msg->expect_right_neg_flag); // 右轮负向标志
        data.push_back(0xAA);

        ser_write(data);
    }
    
    void timer_ser_read_callback()
    {
        if (serial_port_.available())
        {
            // 读取1字节, 准备获取输入
            std::string incoming_data_str = serial_port_.read(1);  // 读取1字节数据
            std::vector<uint8_t> incoming_data(incoming_data_str.begin(), incoming_data_str.end());  // 转换为std::vector<uint8_t>
            
            if (incoming_data.size() == 1 && incoming_data[0] == 0xAA)
            {
                // 读取24个字节数据
                incoming_data_str = serial_port_.read(24);
                incoming_data = std::vector<uint8_t>(incoming_data_str.begin(), incoming_data_str.end());  // 转换为std::vector<uint8_t>
                
                //RCLCPP_INFO(this->get_logger(), "Received data: %s", byte_vector_to_hex(incoming_data).c_str());

                if (incoming_data.size() == 24 && incoming_data[0] == 0xFF && incoming_data[23] == 0xAA)
                {
                    // 逐字节解析数据
                    uint8_t byte1 = incoming_data[0];
                    uint8_t byte2 = incoming_data[1];
                    uint8_t byte3 = incoming_data[2];
                    uint8_t byte4 = incoming_data[3];
                    uint8_t byte5 = incoming_data[4];
                    uint8_t byte6 = incoming_data[5];
                    uint8_t byte7 = incoming_data[6];
                    uint8_t byte8 = incoming_data[7];
                    uint8_t byte9 = incoming_data[8];
                    uint8_t byte10 = incoming_data[9];
                    uint8_t byte11 = incoming_data[10];
                    uint8_t byte12 = incoming_data[11];
                    uint8_t byte13 = incoming_data[12];
                    uint8_t byte14 = incoming_data[13];
                    uint8_t byte15 = incoming_data[14];
                    uint8_t byte16 = incoming_data[15];
                    uint8_t byte17 = incoming_data[16];
                    uint8_t byte18 = incoming_data[17];
                    uint8_t byte19 = incoming_data[18];
                    uint8_t byte20 = incoming_data[19];
                    uint8_t byte21 = incoming_data[20];
                    uint8_t byte22 = incoming_data[21];
                    uint8_t byte23 = incoming_data[22];
                    uint8_t byte24 = incoming_data[23];

                    // 处理速度和其他传感器数据
                    int16_t speed1 = (byte2 << 8) | byte3;  // left wheel speed
                    int16_t speed2 = (byte4 << 8) | byte5;  // right wheel speed
                    uint8_t neg_flag1 = byte22;  // lef wheel direction
                    uint8_t neg_flag2 = byte23;  // right wheel direction

                    // int16_t joystick_x = (byte6 << 8) | byte7;
                    // int16_t joystick_y = (byte8 << 8) | byte9;
                    // int16_t sensor_1 = (byte10 << 8) | byte11;
                    // int16_t sensor_2 = (byte12 << 8) | byte13;
                    // int16_t sensor_3 = (byte14 << 8) | byte15;
                    // int16_t sensor_4 = (byte16 << 8) | byte17;
                    // int16_t return_speed1 = (byte18 << 8) | byte19;
                    // int16_t return_speed2 = (byte20 << 8) | byte21;


                    // 调整数值范围
                    // if (joystick_x > 1900 && joystick_x < 2100)
                    // {
                    //     joystick_x = 2000;
                    // }
                    // if (joystick_y > 1900 && joystick_y < 2100)
                    // {
                    //     joystick_y = 2000;
                    // }

                    if (speed1 > 10000)
                    {
                        //RCLCPP_INFO(this->get_logger(), "Speed1 more than 10000 (original): %d", speed1);
                        speed1 = (65535 - speed1);
                    }
                    else if (speed1 > 0)
                    {
                        //RCLCPP_INFO(this->get_logger(), "Speed1 more than 0 (original): %d", speed1);
                        speed1 = -speed1;
                    }
                    else
                    {
                        //RCLCPP_INFO(this->get_logger(), "Speed1 less than 0: %d", speed1);
                    }

                    if (speed2 > 10000)
                    {
                        speed2 = -(65535 - speed2);
                    }

                    // 发布数据
                    //auto msg = nav_interfaces::msg::WheelSpeed();
                    //msg.speed_l = static_cast<float>(speed1);
                    //msg.speed_r = static_cast<float>(speed2);
                    // 在代码中定义和使用 HardwareData 结构体
                    auto msg = robot_hardware::msg::HardwareData();
                    msg.real_left_speed_rpm = speed1;
                    msg.real_right_speed_rpm = speed2;
                    msg.real_left_neg_flag = neg_flag1;
                    msg.real_right_neg_flag = neg_flag2;

                    publisher_->publish(msg);


                    //RCLCPP_INFO(this->get_logger(), "Published speeds: left: %d, right: %d", speed1, speed2);
                }
            }
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}
