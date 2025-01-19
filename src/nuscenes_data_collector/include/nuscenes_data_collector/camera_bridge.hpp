#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <cstring>
#include <sstream>
#include <map>
#include <mutex>

class UsbCameraBridge : public rclcpp::Node
{
public:
    UsbCameraBridge() : Node("usb_camera_node")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("frame_rate", 10);
        this->declare_parameter<int>("image_width", 1920);
        this->declare_parameter<int>("image_height", 1080);
        this->declare_parameter<int>("post_processing_img_width", 1600);
        this->declare_parameter<int>("post_processing_img_height", 900);

        // Retrieve parameters from the parameter server
        frame_rate_ = this->get_parameter("frame_rate").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        post_processing_img_width_ = this->get_parameter("post_processing_img_width").as_int();
        post_processing_img_height_ = this->get_parameter("post_processing_img_height").as_int();

        // Scan and open cameras
        if (!scan_and_open_cameras())
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras could be opened. Shutting down the node.");
            rclcpp::shutdown();
            return;
        }
    }

    // API: Capture a frame from a specific camera and return the image (cv::Mat) and timestamp (rclcpp::Time)
    bool capture_frame(const std::string &camera_name, std::pair<cv::Mat, rclcpp::Time> &out_frame_data)
    {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        
        auto &cap = camera_caps_[camera_name];
        
        if (!cap.isOpened())
        {
            RCLCPP_WARN(this->get_logger(), "Camera %s is not opened. Skipping frame capture.", camera_name.c_str());
            return false;
        }

        cv::Mat frame;
        cap >> frame;  // Capture a frame

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Captured an empty frame from camera %s.", camera_name.c_str());
            return false;
        }

        // Resize image if needed
        if (frame.cols != post_processing_img_width_ || frame.rows != post_processing_img_height_)
        {
            cv::resize(frame, frame, cv::Size(post_processing_img_width_, post_processing_img_height_));
        }

        // Get the current time as timestamp
        rclcpp::Time timestamp = this->now();

        // Return the frame and timestamp as a pair
        out_frame_data = std::make_pair(frame, timestamp);
        return true;
    }

private:
    int frame_rate_;
    int image_width_;
    int image_height_;
    int post_processing_img_width_;
    int post_processing_img_height_;
    bool flip_x_, flip_y_, is_img_show;

    std::map<std::string, cv::VideoCapture> camera_caps_;  // Store device names and VideoCapture objects
    std::mutex capture_mutex_;  // Used for synchronizing capture operations

    // Scan the /dev directory to find camera devices
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
            // Only process devices that start with "CAM_" and exclude "CAM_COMPUTER"
            if (std::strncmp(entry->d_name, "CAM_", 4) == 0 && std::strstr(entry->d_name, "CAM_COMPUTER") == nullptr)
            {
                std::string device_path = "/dev/" + std::string(entry->d_name);
                std::string camera_name = entry->d_name; // Use the device name as the camera name

                if (!open_camera(device_path, camera_name))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open camera %s", device_path.c_str());
                    continue; // If opening the camera fails, try the next one
                }
            }
        }
        closedir(dir);  // Close the directory

        return !camera_caps_.empty();  // Return true if cameras were successfully opened
    }

    // Try to open the specified camera device and store it
    bool open_camera(const std::string &device_path, const std::string &camera_name)
    {
        cv::VideoCapture cap(device_path);
        if (!cap.isOpened())
        {
            return false; // If unable to open the camera, return false
        }

        // Set image size
        cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

        // Store the device file and corresponding VideoCapture object
        camera_caps_[camera_name] = cap;
        RCLCPP_INFO(this->get_logger(), "Camera %s opened successfully.", device_path.c_str());
        return true;
    }
};
