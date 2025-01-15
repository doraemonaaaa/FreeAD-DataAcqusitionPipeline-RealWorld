#include "usb_camera/usb_camera.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UsbCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
