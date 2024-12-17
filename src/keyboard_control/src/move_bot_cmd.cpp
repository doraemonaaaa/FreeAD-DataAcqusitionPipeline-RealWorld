#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveBot : public rclcpp::Node
{
public:
    MoveBot() : Node("move_bot")
    {
        // 创建发布器，发布到 /cmd_vel 话题，消息类型是 Twist
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 定时器每秒调用一次 publish_velocity 方法
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MoveBot::publish_velocity, this));
    }

private:
    // 定时发布消息
    void publish_velocity()
    {
        // 创建 Twist 消息
        geometry_msgs::msg::Twist twist_msg;

        // 设置线速度，向前移动
        twist_msg.linear.x = 0.5;
        // 设置角速度，不旋转
        twist_msg.angular.z = 0.0;

        // 发布消息
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f", 
                    twist_msg.linear.x, twist_msg.angular.z);
    }

    // 发布器和定时器成员
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建并启动节点
    rclcpp::spin(std::make_shared<MoveBot>());

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
