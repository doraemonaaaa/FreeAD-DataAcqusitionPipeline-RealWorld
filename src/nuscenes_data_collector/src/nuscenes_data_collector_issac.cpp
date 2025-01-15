#include "nuscenes_data_collector_issac.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the node
    auto node = std::make_shared<NuScenesDataCollectorIssac>();

    // Spin the node to keep it running
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}