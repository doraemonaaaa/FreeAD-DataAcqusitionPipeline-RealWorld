#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/msg/hardware_data.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

using std::placeholders::_1;

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode() : Node("keyboard_control_node") {
        speed_ = 400;
        // 初始化发布者
        publisher_ = this->create_publisher<robot_hardware::msg::HardwareData>("keyboard_cmd", 10);
        RCLCPP_INFO(this->get_logger(), "Keyboard control node started. Use W/S to move forward/backward, A/D to turn left/right.");

        // 初始化键盘输入
        initKeyboard();
    }

    ~KeyboardControlNode() {
        restoreKeyboard();
    }

    void run() {
        while (rclcpp::ok()) {
            int key = getchar();

            // 设置左右轮期望速度和方向标志
            int16_t left_speed_rpm = 0;
            int16_t right_speed_rpm = 0;
            uint8_t left_neg_flag = 0;
            uint8_t right_neg_flag = 0;

            // 按键控制逻辑
            switch (key) {
                case 'w':  // 前进
                    left_speed_rpm = speed_;
                    right_speed_rpm = speed_;
                    left_neg_flag = 1;
                    right_neg_flag = 1;
                    break;
                case 's':  // 后退
                    left_speed_rpm = speed_;
                    right_speed_rpm = speed_;
                    left_neg_flag = 0;
                    right_neg_flag = 0;
                    break;
                case 'a':  // 左转
                    left_speed_rpm = speed_;
                    right_speed_rpm = speed_;
                    left_neg_flag = 0;
                    right_neg_flag = 1;
                    break;
                case 'd':  // 右转
                    left_speed_rpm = speed_;
                    right_speed_rpm = speed_;
                    left_neg_flag = 1;
                    right_neg_flag = 0;
                    break;
                case 'q':  // 停止
                    left_speed_rpm = 0;
                    right_speed_rpm = 0;
                    break;
                default:
                    continue;
            }

            // 发送控制消息
            auto msg = robot_hardware::msg::HardwareData();
            msg.expect_left_speed_rpm = left_speed_rpm;
            msg.expect_right_speed_rpm = right_speed_rpm;
            msg.expect_left_neg_flag = left_neg_flag;
            msg.expect_right_neg_flag = right_neg_flag;

            publisher_->publish(msg);
        }
    }

private:
    rclcpp::Publisher<robot_hardware::msg::HardwareData>::SharedPtr publisher_;
    int speed_;

    // 键盘设置
    struct termios old_tio, new_tio;

    void initKeyboard() {
        tcgetattr(STDIN_FILENO, &old_tio); // 保存旧的键盘设置
        new_tio = old_tio; // 基于旧设置创建新的设置
        new_tio.c_lflag &= (~ICANON & ~ECHO); // 设置非规范模式且不回显
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio); // 应用新设置
    }

    void restoreKeyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); // 恢复旧的键盘设置
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
