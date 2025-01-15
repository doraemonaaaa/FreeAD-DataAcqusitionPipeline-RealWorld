#include "rclcpp/rclcpp.hpp"
#include "imu_to_nuscenes.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto imu_node = std::make_shared<ImuToNuScenes>();  // 修改为 imu_sub_name

    // 启动 ROS 2 节点
    rclcpp::spin(imu_node);

    rclcpp::shutdown();
    return 0;
}
