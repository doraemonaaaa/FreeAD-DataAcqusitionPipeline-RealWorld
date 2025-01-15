#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <cstring>
#include <sstream>

class UsbCameraNode : public rclcpp::Node
{
public:
    UsbCameraNode() : Node("usb_camera_node")
    {
        // 扫描 /dev 目录，查找以 "CAM_" 开头且不包含 "CAM_COMPUTER" 的设备文件
        if (!scan_and_open_cameras())
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras could be opened. Shutting down the node.");
            rclcpp::shutdown();
            return;
        }

        // 创建定时器，定期捕获图像帧（10Hz）
        img_show_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                  std::bind(&UsbCameraNode::capture_frames, this));

        // 创建超时检测定时器，每 1 秒检查一次摄像头超时
        timeout_check_timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                                       std::bind(&UsbCameraNode::check_camera_timeout, this));
    }

private:
    std::map<std::string, cv::VideoCapture> camera_caps_;  // 存储设备名和对应的 VideoCapture 对象
    std::map<std::string, rclcpp::Time> last_frame_time_; // 存储每个摄像头的最后帧时间
    const rclcpp::Duration timeout_ = rclcpp::Duration(2, 0); // 超时时间为2秒
    rclcpp::TimerBase::SharedPtr img_show_timer_;           // 捕获帧定时器
    rclcpp::TimerBase::SharedPtr timeout_check_timer_;     // 超时检测定时器

    // 扫描 /dev 目录，查找以 "CAM_" 开头且不包含 "CAM_COMPUTER" 的设备文件，并尝试打开它们
    bool scan_and_open_cameras()
    {
        DIR *dir = opendir("/dev");
        if (dir == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev directory.");
            return false;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            // 只处理以 "CAM_" 开头且不包含 "CAM_COMPUTER" 的设备文件
            if (std::strncmp(entry->d_name, "CAM_", 4) == 0 && std::strstr(entry->d_name, "CAM_COMPUTER") == nullptr)
            {
                std::string device_path = "/dev/" + std::string(entry->d_name);
                std::string camera_name = entry->d_name; // 使用设备名称作为摄像头的名称

                if (!open_camera(device_path, camera_name))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open camera %s", device_path.c_str());
                    continue; // 如果打开失败，继续尝试下一个设备
                }
            }
        }
        closedir(dir);  // 关闭目录

        return !camera_caps_.empty();  // 如果有摄像头被打开，则返回 true
    }

    // 尝试打开指定路径的摄像头并将其存储
    bool open_camera(const std::string &device_path, const std::string &camera_name)
    {
        cv::VideoCapture cap(device_path);
        if (!cap.isOpened())
        {
            return false; // 如果无法打开摄像头，返回 false
        }

        // 将设备文件和对应的 VideoCapture 对象存入 camera_caps_ 中
        camera_caps_[camera_name] = cap;
        last_frame_time_[camera_name] = this->now();  // 初始化最后的帧时间戳
        RCLCPP_INFO(this->get_logger(), "Camera %s opened successfully.", device_path.c_str());
        return true;
    }

    // 捕获当前帧
    void capture_frames()
    {
        for (auto &camera : camera_caps_)
        {
            std::string camera_name = camera.first;
            auto &cap = camera.second;

            if (!cap.isOpened())
            {
                RCLCPP_WARN(this->get_logger(), "Camera %s is not opened. Skipping frame capture.", camera_name.c_str());
                continue;
            }

            cv::Mat frame;
            cap >> frame; // 捕获帧

            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Captured an empty frame from camera %s. Check the camera connection or settings.", camera_name.c_str());
                continue;
            }

            // 更新摄像头最后捕获的时间戳
            last_frame_time_[camera_name] = this->now();
            RCLCPP_INFO(this->get_logger(), "Successfully captured a frame from camera %s.", camera_name.c_str());
        }
    }

    // 检查每个摄像头的超时状态
    void check_camera_timeout()
    {
        for (auto &camera : camera_caps_)
        {
            std::string camera_name = camera.first;

            // 获取摄像头的最后捕获时间
            auto last_time = last_frame_time_[camera_name];

            // 如果超时，记录警告日志
            if (this->now() - last_time > timeout_)
            {
                RCLCPP_WARN(this->get_logger(), "Camera %s has not provided a frame for more than 2 seconds.", camera_name.c_str());
            }
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
