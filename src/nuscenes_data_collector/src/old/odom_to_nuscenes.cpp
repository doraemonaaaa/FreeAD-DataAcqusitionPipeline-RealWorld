#include "rclcpp/rclcpp.hpp"
#include "odom_to_nuscenes.hpp"

int main(int argc, char **argv)
{
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);

    // 创建 OdomToNuScenes 节点实例，并传入话题名称
    auto odom_node = std::make_shared<OdomToNuScenes>();

    // 启动 ROS 2 节点并开始接收消息
    rclcpp::spin(odom_node);

    // 当节点停止时，关闭 ROS 2 系统
    rclcpp::shutdown();
    return 0;
}
