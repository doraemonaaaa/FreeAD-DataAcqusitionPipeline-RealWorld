#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

// 获取键盘按键输入
char get_key()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);  // 获取当前终端设置
    newt = oldt;
    newt.c_lflag &= ~ICANON;  // 禁用缓冲模式
    newt.c_lflag &= ~ECHO;    // 禁用回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();  // 获取键盘输入
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复终端设置
    return ch;
}

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        // 创建发布者，发布 Twist 消息到 /cmd_vel 话题
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 创建定时器，每 0.1 秒检查一次键盘输入
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&KeyboardControl::publish_twist, this));
        
        // 打印控制信息
        RCLCPP_INFO(this->get_logger(), "Use 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right.");
    }

private:
    void publish_twist()
    {
        // 默认速度为 0
        twist_.linear.x = 0.0;
        twist_.angular.z = 0.0;

        // 检查键盘输入并设置线速度
        char key = get_key();
        if (key == 's') {  // 按 's' 键时往后退
            twist_.linear.x = -0.2;  // 负值表示往后退
            RCLCPP_INFO(this->get_logger(), "Moving backward");
        }
        else if (key == 'w') {  // 按 'w' 键时往前进
            twist_.linear.x = 0.2;
            RCLCPP_INFO(this->get_logger(), "Moving forward");
        }
        else if (key == 'a') {  // 按 'a' 键时向左转
            twist_.angular.z = 0.2;
            RCLCPP_INFO(this->get_logger(), "Turning left");
        }
        else if (key == 'd') {  // 按 'd' 键时向右转
            twist_.angular.z = -0.2;
            RCLCPP_INFO(this->get_logger(), "Turning right");
        }

        // 发布 Twist 消息
        publisher_->publish(twist_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControl>());
    rclcpp::shutdown();
    return 0;
}
