#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <cstring>
#include <sstream>
#include <nlohmann/json.hpp>  // For JSON data handling
#include <fstream>
#include <algorithm>  // For std::transform

class UsbCameraNode : public rclcpp::Node
{
public:
    UsbCameraNode() : Node("usb_camera_node")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("frame_rate", 30);
        this->declare_parameter<int>("image_width", 1920);
        this->declare_parameter<int>("image_height", 1080);
        this->declare_parameter<std::string>("img_type", "PNG");
        this->declare_parameter<int>("post_processing_img_width", 1600);
        this->declare_parameter<int>("post_processing_img_height", 900);
        this->declare_parameter<bool>("flip_x", false);  // Default is no flip (false)
        this->declare_parameter<bool>("flip_y", false);  // Default is no flip (false)

        // Retrieve parameters from the parameter server
        frame_rate_ = this->get_parameter("frame_rate").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        img_type_ = this->get_parameter("img_type").as_string();
        post_processing_img_width_ = this->get_parameter("post_processing_img_width").as_int();
        post_processing_img_height_ = this->get_parameter("post_processing_img_height").as_int();
        flip_x_ = this->get_parameter("flip_x").as_bool();
        flip_y_ = this->get_parameter("flip_y").as_bool();

        // Scan and open cameras
        if (!scan_and_open_cameras())
        {
            RCLCPP_ERROR(this->get_logger(), "No cameras could be opened. Shutting down the node.");
            rclcpp::shutdown();
            return;
        }

        // Create timer to capture frames periodically based on frame rate
        img_captrue_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / frame_rate_), 
                                                  std::bind(&UsbCameraNode::capture_frames, this));

        // Create timer to check for camera timeouts
        timeout_check_timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                                       std::bind(&UsbCameraNode::check_camera_timeout, this));
    }

private:
    int frame_rate_;
    int image_width_;
    int image_height_;
    std::string img_type_;
    int post_processing_img_width_;
    int post_processing_img_height_;
    bool flip_x_, flip_y_;

    std::map<std::string, cv::VideoCapture> camera_caps_;  // Store device names and VideoCapture objects
    std::map<std::string, rclcpp::Time> last_frame_time_; // Store the last frame time for each camera
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_; // Store publishers for each camera
    const rclcpp::Duration timeout_ = rclcpp::Duration(2, 0); // Timeout duration of 2 seconds
    rclcpp::TimerBase::SharedPtr img_captrue_timer_;           // Frame capture timer
    rclcpp::TimerBase::SharedPtr timeout_check_timer_;     // Timeout check timer

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

                // Create a publisher for each camera, with a topic based on the camera name
                image_publishers_[camera_name] = this->create_publisher<sensor_msgs::msg::Image>(device_path, 100);
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
        last_frame_time_[camera_name] = this->now();  // Initialize the last frame timestamp
        RCLCPP_INFO(this->get_logger(), "Camera %s opened successfully.", device_path.c_str());
        return true;
    }

    // Capture the current frame and publish as an image message
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
            cap >> frame; // Capture the frame

            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Captured an empty frame from camera %s. Check the camera connection or settings.", camera_name.c_str());
                continue;
            }

            // Resize image if needed
            if (frame.cols != post_processing_img_width_ || frame.rows != post_processing_img_height_)
            {
                cv::resize(frame, frame, cv::Size(post_processing_img_width_, post_processing_img_height_));
            }

            // Apply axis flipping if required
            if (flip_x_)
            {
                cv::flip(frame, frame, 1);  // Flip horizontally
            }
            if (flip_y_)
            {
                cv::flip(frame, frame, 0);  // Flip vertically
            }

            // Display the frame in a window using OpenCV
            cv::imshow(camera_name, frame);  // Show in a window named after the camera
            cv::waitKey(1);  // Wait for 1 ms to process events and refresh the window

            // Convert frame to a sensor_msgs::msg::Image
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            // Publish the captured image to the camera's specific topic
            image_publishers_[camera_name]->publish(*msg);

            // Update the last frame timestamp
            last_frame_time_[camera_name] = this->now();
            RCLCPP_INFO(this->get_logger(), "Successfully captured and published a frame from camera %s.", camera_name.c_str());
        }
    }

    // Check for camera timeouts
    void check_camera_timeout()
    {
        for (auto &camera : camera_caps_)
        {
            std::string camera_name = camera.first;

            // Get the last capture time for the camera
            auto last_time = last_frame_time_[camera_name];

            // If the camera has timed out, log a warning
            if (this->now() - last_time > timeout_)
            {
                RCLCPP_ERROR(this->get_logger(), "Camera %s has not provided a frame for more than 2 seconds.", camera_name.c_str());
            }
        }
    }
};

