#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class UsbCameraNode : public rclcpp::Node
{
public:
    UsbCameraNode() : Node("usb_camera_node")
    {
        // 根据 camera_labels_ 中的 int 创建摄像头实例
        for (const auto& label : camera_labels_)
        {
            int camera_idx = label.first;  // 获取摄像头编号
            std::string camera_name = label.second;  // 获取标签

            cv::VideoCapture cap(camera_idx); // 使用摄像头编号打开摄像头
            if (!cap.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera %d", camera_idx);
                rclcpp::shutdown();
                return;
            }

            // 将摄像头标签和对应的 VideoCapture 对象存入 camera_caps_ 中
            camera_caps_[camera_name] = cap;
        }

        // 创建发布者，用于发送图像
        img_show_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // 创建定时器，定期调用 publish_image 函数
        img_show_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                 std::bind(&UsbCameraNode::publish_image, this)); // 10Hz 频率
    }

private:
    std::map<std::string, cv::VideoCapture> camera_caps_;  // 存储标签和对应的 VideoCapture 对象
    std::map<int, std::string> camera_labels_ = {
        {0, "front"}, // 摄像头编号 0 对应前置摄像头
        {1, "left"},  // 摄像头编号 1 对应左侧摄像头
        {2, "right"}, // 摄像头编号 2 对应右侧摄像头
        {3, "rear"}   // 摄像头编号 3 对应后置摄像头
        // 你可以继续为其他摄像头设置编号和标签
    };

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_show_publisher_; // 发布者
    rclcpp::TimerBase::SharedPtr img_show_timer_; // 定时器

    void publish_image()
    {
        // 假设我们选择发布 "left" 摄像头图像
        std::string selected_camera_label = "left";  // 你可以根据需要选择不同的标签

        // 查找对应标签的摄像头
        if (camera_caps_.find(selected_camera_label) != camera_caps_.end())
        {
            auto& cap = camera_caps_[selected_camera_label]; // 获取 VideoCapture 对象
            cv::Mat frame;
            cap >> frame; // 捕获指定摄像头的帧

            if (frame.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Captured empty frame from camera %s", selected_camera_label.c_str());
                return;
            }

            // 将 OpenCV 图像转换为 ROS 图像消息
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_show_publisher_->publish(*msg); // 发布图像
            RCLCPP_INFO(this->get_logger(), "Published image from camera %s", selected_camera_label.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid camera label: %s", selected_camera_label.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UsbCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
