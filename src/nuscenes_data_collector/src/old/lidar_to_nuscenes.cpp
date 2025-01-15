#include "rclcpp/rclcpp.hpp"
#include "lidar_to_nuscenes.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建两个节点实例
    auto lidar_node = std::make_shared<LidarToNuScenes>();  // 修改为 lidar_sub_name

    // 启动 ROS 2 节点
    rclcpp::spin(lidar_node);  // 先启动 lidar_node

    rclcpp::shutdown();
    return 0;
}
