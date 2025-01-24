#ifndef NUSCENES_DATA_COLLECTOR_HPP
#define NUSCENES_DATA_COLLECTOR_HPP

#include "rclcpp/rclcpp.hpp" 
#include "rclcpp/timer.hpp" 
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <mutex> // For thread safety

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "sensor_msgs/image_encodings.hpp" // For image encoding constants
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rosgraph_msgs/msg/clock.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// message sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <future>

#include "nuscenes_data_collector/utils.hpp"
#include "nuscenes_data_collector/nuscenes_json_writer.hpp"
#include "nuscenes_data_collector/token_rule.hpp"
#include "nuscenes_data_collector/camera_bridge.hpp"


class NuScenesDataCollector : public rclcpp::Node {
public:
    NuScenesDataCollector()
    : Node("nuscenes_data_collector"),              // 初始化 Node 名称
      root_path_(this->declare_parameter<std::string>("root_path_", "/home/pyh/Documents/ros2_ws/RobotAD_Issac_ws/NuscenesData")), // 从参数中获取 root_path_
      json_paths_({ // json init
          root_path_ + "/ego_pose.json", 
          root_path_ + "/sample_data.json", 
          root_path_ + "/sample_annotation.json",
          root_path_ + "/sample.json",
          root_path_ + "/scene.json",
          root_path_ + "/log.json",
          root_path_ + "/instance.json",
          root_path_ + "/map.json",
          root_path_ + "/can_bus/can_bus.json"
      }),
      json_writer_(json_paths_)                     // 将 logger 传递给 json_writer_
    {
        // Declare parameters with JSON strings as default values
        this->declare_parameter<std::string>("sensor_token_map_", R"({
            "CAM_FRONT": "SENSOR_CAM_FRONT",
            "CAM_BACK": "SENSOR_CAM_BACK",
            "CAM_BACK_LEFT": "SENSOR_CAM_BACK_LEFT",
            "CAM_FRONT_LEFT": "SENSOR_CAM_FRONT_LEFT",
            "CAM_FRONT_RIGHT": "SENSOR_CAM_FRONT_RIGHT",
            "CAM_BACK_RIGHT": "SENSOR_CAM_BACK_RIGHT",
            "LIDAR_TOP": "SENSOR_LIDAR_TOP"
        })");
        
        this->declare_parameter<std::string>("calibrated_sensor_token_map_", R"({
            "CAM_FRONT": "CALIBRATION_RW1_CAM_FRONT",
            "CAM_BACK": "CALIBRATION_RW1_CAM_BACK",
            "CAM_BACK_LEFT": "CALIBRATION_RW1_CAM_BACK_LEFT",
            "CAM_FRONT_LEFT": "CALIBRATION_RW1_CAM_FRONT_LEFT",
            "CAM_FRONT_RIGHT": "CALIBRATION_RW1_CAM_FRONT_RIGHT",
            "CAM_BACK_RIGHT": "CALIBRATION_RW1_CAM_BACK_RIGHT",
            "LIDAR_TOP": "CALIBRATION_RW1_LIDAR_TOP"
        })");

        // Declare other parameters
        this->declare_parameter<int64_t>("target_sample_frame_", 20);
        this->declare_parameter<int>("scene_frame_", 20);
        this->declare_parameter<std::string>("sample_scene_recoder_name_", "LIDAR_TOP");
        this->declare_parameter<std::string>("scene_mode_", "train");
        this->declare_parameter<std::string>("robot_model_", "diff_bot");
        this->declare_parameter<std::string>("map_name_", "office_issac");
        this->declare_parameter<float>("sample_duration_", 0.5);
        this->declare_parameter<std::vector<std::string>>("sample_sensors_", {
            "CAM_FRONT", "CAM_BACK", "CAM_BACK_LEFT",
            "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT", "LIDAR_TOP"
        });

        // Retrieve parameters from the parameter server
        this->get_parameter("scene_mode_", scene_mode_);
        this->get_parameter("robot_model_", robot_model_);
        this->get_parameter("map_name_", map_name_);
        this->get_parameter("root_path_", root_path_);
        this->get_parameter("sample_duration_", sample_duration_);
        this->get_parameter("sample_sensors_", sample_sensors_);

        // Retrieve JSON string parameters
        std::string sensor_token_map_json;
        this->get_parameter("sensor_token_map_", sensor_token_map_json);
        std::string calibrated_sensor_token_map_json;
        this->get_parameter("calibrated_sensor_token_map_", calibrated_sensor_token_map_json);

        // Parse JSON strings into unordered_maps
        try {
            auto sensor_token_map_json_parsed = nlohmann::json::parse(sensor_token_map_json);
            for (auto it = sensor_token_map_json_parsed.begin(); it != sensor_token_map_json_parsed.end(); ++it) {
                sensor_token_map_[it.key()] = it.value();
            }

            auto calibrated_sensor_token_map_json_parsed = nlohmann::json::parse(calibrated_sensor_token_map_json);
            for (auto it = calibrated_sensor_token_map_json_parsed.begin(); it != calibrated_sensor_token_map_json_parsed.end(); ++it) {
                calibrated_sensor_token_map_[it.key()] = it.value();
            }
        }
        catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
        }

        // Retrieve additional parameters
        this->get_parameter("target_sample_frame_", target_sample_frame_);
        this->get_parameter("scene_frame_", scene_frame_);
        this->get_parameter("sample_scene_recoder_name_", sample_scene_recoder_name_);

        // 创建一个大日志，包含所有相关信息，并加入清晰的分隔符
        std::stringstream log_message;
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                     [Scene Info]                        \n";
        log_message << "----------------------------------------------------------\n";
        log_message << "Scene Mode: " << scene_mode_ << std::endl;
        log_message << "Robot Model: " << robot_model_ << std::endl;
        log_message << "Map Name: " << map_name_ << std::endl;
        log_message << "Root Path: " << root_path_ << std::endl;
        log_message << "Sample Frequency: " << sample_duration_ << std::endl;
        log_message << "Target Sample Frame: " << target_sample_frame_ << std::endl;
        log_message << "Scene Frame: " << scene_frame_ << std::endl;
        log_message << "Sample Scene Recorder Name: " << sample_scene_recoder_name_ << std::endl;
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                   [Sample Sensors]                      \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& sensor : sample_sensors_) {
            log_message << "  - " << sensor << std::endl;
        }
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                [Sensor Token Map]                        \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& [key, value] : sensor_token_map_) {
            log_message << "  - " << key << ": " << value << std::endl;
        }
        log_message << "\n----------------------------------------------------------\n";
        log_message << "     [Calibrated Sensor Token Map]                        \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& [key, value] : calibrated_sensor_token_map_) {
            log_message << "  - " << key << ": " << value << std::endl;
        }
        // 输出到日志
        RCLCPP_INFO(this->get_logger(), "%s", log_message.str().c_str());


        // 检查 JSON 加载是否有错误
        if (json_writer_.isLoadError()) {
            RCLCPP_ERROR(this->get_logger(), "Json Load Error. Shutting down the node.");
            rclcpp::shutdown();  // 停止 ROS 节点
            return;  // 退出函数，确保不继续执行后续代码
        }

        // 初始化其他参数和订阅
        declare_parameters();
        
        // 传感器集初始化
        for(auto c: camera_names_) add_sample_sensors(c);
        add_sample_sensors("LIDAR_TOP");
        for(auto device : sample_sensors_){
            devices_frame_count_[device] = 0;
            sample_triggers_[device] = true;
            sample_done_sign_[device] = false;
        }
        start_system_time_token_ = get_cur_system_time();
        scene_sample_frame_count_ = 0;
        std::string cur_system_time_string = get_cur_system_time();
        sample_frame_ = 0;
        cur_scene_num_ = 0;

        // write log
        log_token_ = map_name_ + start_system_time_token_;
        json log_json;
        log_json["token"] = log_token_;
        log_json["logfile"] = "None";  // TODO: waitting for log method
        log_json["vehicle"] = robot_model_;
        log_json["date_captured"] = start_system_time_token_.substr(0, 10);  // 从字符串中提取前10个字符，即 YYYY-MM-DD
        log_json["location"] = map_name_;
        json_writer_.addKeyValuePairToJson("log", log_json, true); 
        // write to map
        map_write(log_json, root_path_+"/map.json", map_name_);

        last_sample_time_ = this->now();
        uint64_t timestamp_ns = static_cast<uint64_t>(last_sample_time_.nanoseconds());
        std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
        std::string next_sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
        sample_recorder_cache_.first = timestamp_ns;
        sample_recorder_cache_.second.first = sample_token;
        sample_recorder_cache_.second.second = next_sample_token;
        // init the scene
        scene_init(sample_token);
        // 创建定时器，调用clock_callback to sample
        clock_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(90)), // 毫秒触发一次
            std::bind(&NuScenesDataCollector::clock_callback, this)
        );

        // Create timer for image capture
        capture_image_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(133), 
            std::bind(&NuScenesDataCollector::capture_image_callback, this)
        );

        // init the sensor workflow
        // 创建一个 UsbCameraBridge 实例
        usb_cameras_ = std::make_shared<UsbCameraBridge>();
        image_for_driving_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dev/CAM_FRONT", 10);
        // warm up the camera
        for(int i=0; i < 20; i++){
            for(auto& camera : camera_names_){
                std::pair<cv::Mat, rclcpp::Time> frame_data;
                usb_cameras_->capture_frame(camera, frame_data);
            }
        }

        rclcpp::QoS qos_profile(1);  // 队列大小为1，确保只处理最新的消息
        qos_profile.reliable();  // 设置可靠传输
        qos_profile.durability_volatile();  // 设置为volatile，防止缓存消息
        // Odometry 订阅
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos_profile, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_callback(msg, "odom");
            }
        );
        // IMU 订阅
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,        // 替换为您的 IMU 主题名称，例如 "/sensor/imu"
            qos_profile,       // 使用之前定义的 QoS 配置
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->imu_callback(msg, "imu");  // 定义 IMU 回调函数以处理接收到的消息
            }
        );

        rclcpp::QoS lidar_qos_profile(5);
        qos_profile.reliable();  // 设置可靠传输
        // lidar 订阅
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, lidar_qos_profile, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->lidar_callback(msg, "LIDAR_TOP");
            }
        );
    }

private:


    void add_sample_sensors(std::string device_name){
        sample_sensors_.emplace_back(device_name);
    }

    void declare_parameters() {
        // Initialize camera names and declare related parameters
        camera_names_ = {"CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"};
        // Get odom_topic and lidar_topic parameter
        odom_topic_ = "/diffbot_base_controller/odom";
        RCLCPP_INFO(this->get_logger(), "Retrieved odom_topic: %s", odom_topic_.c_str());
        lidar_topic_ = "/scanner/cloud";
        RCLCPP_INFO(this->get_logger(), "Retrieved lidar_topic: %s", lidar_topic_.c_str());
        imu_topic_ = "/demo/imu";
        RCLCPP_INFO(this->get_logger(), "Retrieved imu_topic: %s", imu_topic_.c_str());
    }

    // Callback to capture an image from the camera
    void capture_image_callback()
    {
        if ((!latest_ego_pose_.empty()) && (!latest_imu_.empty())) {
            
            std::vector<std::future<bool>> futures;  // 用于存储每个摄像头的捕获结果
            std::unordered_map<std::string, std::pair<cv::Mat, rclcpp::Time>> captured_data;  // 存储每个摄像头的图像和时间戳
            std::mutex data_mutex;  // 保护 captured_data 的 mutex，防止多线程并发写入冲突

            // 并发地捕获图像
            for (const auto &camera_name : camera_names_)
            {
                futures.push_back(std::async(std::launch::async, [this, &captured_data, &data_mutex, camera_name]() {
                    std::pair<cv::Mat, rclcpp::Time> frame_data;
                    bool success = usb_cameras_->capture_frame(camera_name, frame_data);  // 异步调用 capture_frame
                    
                    if (success)
                    {
                        // 如果捕获成功，将图像数据存储到线程安全的容器中
                        std::lock_guard<std::mutex> lock(data_mutex);
                        captured_data[camera_name] = frame_data;
                    }
                    return success;
                }));
            }

            // 等待所有的捕获线程完成，并处理每个摄像头的图像
            for (auto &fut : futures)
            {
                fut.get();  // 确保每个摄像头的图像捕获完成
            }

            // 顺序处理每个摄像头的图像
            for (const auto &camera_name : camera_names_)
            {
                if (captured_data.find(camera_name) != captured_data.end())
                {
                    std::pair<cv::Mat, rclcpp::Time> frame_data = captured_data[camera_name];
                    // Handle the captured image and timestamp
                    cv::Mat image = frame_data.first;
                    rclcpp::Time timestamp = frame_data.second;

                    // Determine the data type ('samples' or 'sweeps')
                    std::string data_type = (sample_triggers_[camera_name] == true) ? "samples" : "sweeps";
                    bool is_save = (data_type == "samples") ? false : true; // Cache for 'samples', save for 'sweeps'

                    // Generate directory path and ensure it exists
                    std::string folder_path = root_path_ + "/" + data_type + "/" + camera_name;
                    ensure_directory_exists(folder_path);

                    // Generate file path for saving images
                    std::stringstream ss;
                    ss << folder_path << '/' << start_system_time_token_ << "__" 
                       << camera_name << "__" 
                       << std::to_string(timestamp.seconds()) // Ensure no scientific notation
                       << ".jpg";
                    std::string image_save_file = ss.str();

                    // Encode the image to JPEG
                    std::vector<uchar> encoded_img;
                    bool encode_success = cv::imencode(".jpg", image, encoded_img);
                    if (!encode_success)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to encode image to JPEG.");
                        continue; // Skip this image and move to the next one
                    }

                    // Save or cache the image
                    if (is_save)
                    {
                        // Save to disk as JPEG
                        std::ofstream ofs(image_save_file, std::ios::binary);
                        if (ofs.is_open())
                        {
                            ofs.write(reinterpret_cast<const char*>(encoded_img.data()), encoded_img.size());
                            ofs.close();
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to open image file for writing: %s", image_save_file.c_str());
                        }
                    }
                    else
                    {
                        // Cache image data (for 'samples')
                        sample_imgs_cache_[camera_name] = {image_save_file, encoded_img};
                        // set the sign that camera sample done in this epoch
                        sample_triggers_[camera_name] = false;
                        sample_done_sign_[camera_name] = true;
                    }

                    // Now, prepare the JSON data based on the format you provided
                    nlohmann::json sample_data_json;
                    // formula: sample_data_token = harsh(frame, "samples_data_" + device_name, start_system_time_token_)
                    std::string sample_data_token_tmp = "samples_data_" + camera_name;
                    std::string sample_data_token = generate_unique_token(devices_frame_count_[camera_name], camera_name, start_system_time_token_);
                    // write ego_pose
                    latest_ego_pose_["token"] = sample_data_token;  // set as sample_data_token
                    json_writer_.addKeyValuePairToJson("ego_pose", latest_ego_pose_, true);
                    bool is_key_frame = data_type == "samples" ? true : false;
                    // record the sample scene and log
                    std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
                    // write can_bus, 检查 latest_scene_ 是否包含 "name" 键，并且其值是否非空
                    if (latest_scene_.contains("name") && !latest_scene_["name"].get<std::string>().empty()) {
                        std::string scene_name = latest_scene_["name"];
                        json_writer_.addDataUnderKey("can_bus", scene_name, latest_imu_, true);
                    } 
                    // Extract the path from the image path, keeping only the part after 'sweeps' or 'samples'
                    std::string relative_path = extractRelativePath(image_save_file);
                    // get calibrated sensor token 
                    std::string calibrated_sensor_token = get_calibration_token(calibrated_sensor_token_map_ ,camera_name);
                    if(calibrated_sensor_token == "")
                        RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", camera_name.c_str());
                    // Prepare the JSON fields
                    sample_data_json["token"] = sample_data_token;
                    sample_data_json["sample_token"] = sample_token;  // 指向sample_data所关联的sample
                    sample_data_json["ego_pose_token"] = sample_data_token;
                    sample_data_json["calibrated_sensor_token"] = calibrated_sensor_token;  
                    // 将秒数转换为纳秒
                    int64_t seconds_in_nanoseconds = timestamp.seconds() * 1e9;
                    // 获取纳秒部分
                    int32_t nanoseconds_part = timestamp.nanoseconds();
                    sample_data_json["timestamp"] = seconds_in_nanoseconds + nanoseconds_part; 
                    sample_data_json["fileformat"] = "jpg";  // File format is JPEG
                    sample_data_json["is_key_frame"] = is_key_frame;  // Example, this could be dynamic based on your application
                    sample_data_json["height"] = image.rows;
                    sample_data_json["width"] = image.cols;
                    sample_data_json["filename"] = relative_path;
                    sample_data_json["prev"] = "";  // Example, replace with actual previous token if needed
                    sample_data_json["next"] = "";  // Example, replace with actual next token if needed
                    // generate sample_data's prev and next
                    generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
                    sample_data_json, 
                    "sample_data_" + camera_name, 
                    json_writer_, 
                    "sample_data");
                    
                    devices_frame_count_[camera_name]++;

                    // Publish image for "CAM_FRONT" camera
                    if (camera_name == "CAM_FRONT")
                    {
                        // Create a ROS image message
                        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(
                            std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                        // Set the timestamp for the message
                        image_msg->header.stamp = timestamp;
                        // Publish the image
                        image_for_driving_pub_->publish(*image_msg);
                        //RCLCPP_INFO(this->get_logger(), "Published image from %s at timestamp %f", camera_name.c_str(), timestamp.seconds());
                    }

                    RCLCPP_INFO(this->get_logger(), "Captured image from %s", camera_name.c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to capture image from %s", camera_name.c_str());
                }
            }
        }
    }

    // lidar data callback
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &device_name) {
        try {
            RCLCPP_INFO(this->get_logger(), "Received lidar_top data");
            save_data(msg, device_name);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception occurred in lidar save_data: %s", e.what());
        }
    }

    void clock_callback()
    {
        // 获取当前时间
        auto cur_time = this->now();
        // 计算时间差
        rclcpp::Duration time_diff = cur_time - last_sample_time_;

        // 检查是否达到采样帧
        if (sample_frame_ >= target_sample_frame_) {
            node_shutdown_callback();
            return;
        }

        // Check whether the sample time has been reached and the last sample is done
        if (time_diff.seconds() >= sample_duration_) {
            check_sample_done();
            last_sample_time_ = cur_time;

            // a new sample should be take
            // Calculate timestamp (in nanoseconds)
            uint64_t timestamp_ns = static_cast<uint64_t>(cur_time.nanoseconds());
            std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            std::string next_sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            sample_recorder_cache_.first = timestamp_ns;
            sample_recorder_cache_.second.first = sample_token;
            sample_recorder_cache_.second.second = next_sample_token;
            
            // trigger the camera
            capture_image_callback();
        }
    }

    // Odometry data callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &device_name) {
        //RCLCPP_INFO(this->get_logger(), "received Odometry data");
        save_data(msg, device_name);
    }

    // Odometry data callback
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg, const std::string &device_name) {
        //RCLCPP_INFO(this->get_logger(), "received IMU data");
        save_data(msg, device_name);
    }
    
    /// Template function to save data to sweeps directory
    template <typename T>
        void save_data(const T &msg, const std::string &device_name) {
        try {
            // Check if the message type is Odometry
            if constexpr (std::is_same_v<T, std::shared_ptr<nav_msgs::msg::Odometry>>) {
                save_odom_data(msg);
            }
            // Check if the message type is IMU and we have valid ego_pose
            else if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::Imu>>) {
                if (!latest_ego_pose_.empty()) {
                    save_imu_data(msg);
                } 
            }
            // Ensure we must have an ego pose and imu before processing other types
            if ((!latest_ego_pose_.empty()) && (!latest_imu_.empty())) {
                // Handle PointCloud2 (Lidar) messages
                if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::PointCloud2>>) {
                    save_lidar_data(msg, device_name);
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception caught while saving data: %s", e.what());
            // Optionally return or continue depending on the behavior you want
        }
    }

    // write odom data in nuscenes ego_pose format and save to ego_pose.json
    void save_odom_data(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        json ego_pose_data;
        // The timestamp can be a combination of seconds and nanoseconds
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        ego_pose_data["timestamp"] = timestamp;
        ego_pose_data["rotation"] = {
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        };
        ego_pose_data["translation"] = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };
        latest_ego_pose_ = ego_pose_data;
    }

    void save_imu_data(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
        // 提取线性加速度（linear_accel）
        auto linear_accel = msg->linear_acceleration;
        // 提取四元数（q）
        auto orientation = msg->orientation;
        // 提取角速度（rotation_rate）
        auto angular_velocity = msg->angular_velocity;

        // 获取消息的时间戳并转换为微秒（utime）
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

        // 将geometry_msgs::msg::Quaternion转为JSON数组
        nlohmann::json orientation_json = {orientation.x, orientation.y, orientation.z, orientation.w};

        // 将geometry_msgs::msg::Vector3转为JSON数组
        nlohmann::json angular_velocity_json = {angular_velocity.x, angular_velocity.y, angular_velocity.z};
        nlohmann::json linear_accel_json = {linear_accel.x, linear_accel.y, linear_accel.z};

        // 保存数据到latest_imu_字典
        latest_imu_["linear_accel"] = linear_accel_json;  // 线性加速度
        latest_imu_["q"] = orientation_json;              // 四元数
        latest_imu_["rotation_rate"] = angular_velocity_json;  // 角速度
        latest_imu_["pos"] = latest_ego_pose_["translation"];  // 位置
        latest_imu_["utime"] = timestamp;  // 保存计算的utime
    }


    /// @brief this contain the real intensity from data
    /// @param msg 
    /// @param device_name 
    void save_lidar_data(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const std::string &device_name) {
        // Determine whether to save to 'sweeps' or 'samples'
        std::string data_type = (sample_triggers_[device_name] == true) ? "samples" : "sweeps";
        bool is_save = data_type == "samples" ? false : true; // samples save to the tmp at first
        // Create the directory path for saving point cloud
        std::string folder_path = root_path_ + "/" + data_type + "/" + device_name;
        ensure_directory_exists(folder_path);
        
        // Generate a file path with timestamp for the point cloud
        std::stringstream ss;
        ss << folder_path << '/' << start_system_time_token_ << "__" << device_name << "__" 
        << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".bin"; // 修改为 NuScenes 格式扩展名
        std::string cloud_save_file = ss.str();

        // Convert PointCloud2 to pcl::PointCloud and save to binary file
        pcl::PointCloud<pcl::PointXYZI> cloud;  // 使用 PointXYZI 支持强度值
        
        // Step 1: Convert sensor_msgs::msg::PointCloud2 to pcl::PCLPointCloud2
        pcl::PCLPointCloud2 pcl_cloud2;
        pcl_conversions::toPCL(*msg, pcl_cloud2);  // 使用 pcl_conversions::toPCL
        
        // Step 2: Convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZI>
        pcl::fromPCLPointCloud2(pcl_cloud2, cloud);  // 使用 pcl::fromPCLPointCloud2

        // Ensure the cloud is not empty
        if (cloud.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud is empty: %s", cloud_save_file.c_str());
            return;
        }

        // Save point cloud to binary file in NuScenes format
        if(is_save){
            std::ofstream ofs(cloud_save_file, std::ios::binary);
            if (ofs.is_open()) {
                // Write the point cloud data to file in [x, y, z, intensity] format
                for (const auto &point : cloud.points) {
                    ofs.write(reinterpret_cast<const char*>(&point.x), sizeof(float)); // x
                    ofs.write(reinterpret_cast<const char*>(&point.y), sizeof(float)); // y
                    ofs.write(reinterpret_cast<const char*>(&point.z), sizeof(float)); // z
                    ofs.write(reinterpret_cast<const char*>(&point.intensity), sizeof(float)); // intensity
                }
                ofs.close();
                //RCLCPP_INFO(this->get_logger(), "PointCloud2 saved as BIN: %s", cloud_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving point cloud: %s", cloud_save_file.c_str());
            }
        }
        else{
            // cache the lidar sample
            std::vector<uint8_t> point_cloud_data;
            for (const auto &point : cloud.points) {
                point_cloud_data.insert(point_cloud_data.end(), reinterpret_cast<const uint8_t*>(&point.x), reinterpret_cast<const uint8_t*>(&point.x) + sizeof(float));
                point_cloud_data.insert(point_cloud_data.end(), reinterpret_cast<const uint8_t*>(&point.y), reinterpret_cast<const uint8_t*>(&point.y) + sizeof(float));
                point_cloud_data.insert(point_cloud_data.end(), reinterpret_cast<const uint8_t*>(&point.z), reinterpret_cast<const uint8_t*>(&point.z) + sizeof(float));
                point_cloud_data.insert(point_cloud_data.end(), reinterpret_cast<const uint8_t*>(&point.intensity), reinterpret_cast<const uint8_t*>(&point.intensity) + sizeof(float));
            }

            // Overwrite the existing cache
            sample_lidar_cache_ = {cloud_save_file, point_cloud_data};
            // set the sign shows this epoch sample is done
            sample_triggers_[device_name] = false;
            sample_done_sign_[device_name] = true;
            RCLCPP_INFO(this->get_logger(), "Point cloud data cached: %s", cloud_save_file.c_str());
        }
        
        // Prepare the JSON data for sample_data
        nlohmann::json sample_data_json;
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        bool is_key_frame = (data_type == "samples");  // Example of setting key frame flag
        std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
        std::string sample_data_token = generate_unique_token(devices_frame_count_[device_name], device_name, start_system_time_token_);
        // write ego_pose
        latest_ego_pose_["token"] = sample_data_token;  // set as sample_data_token
        json_writer_.addKeyValuePairToJson("ego_pose", latest_ego_pose_, true);

        // write can_bus, 检查 latest_scene_ 是否包含 "name" 键，并且其值是否非空
        if (latest_scene_.contains("name") && !latest_scene_["name"].get<std::string>().empty()) {
                std::string scene_name = latest_scene_["name"];
                json_writer_.addDataUnderKey("can_bus", scene_name, latest_imu_, true);
        } 

        std::string calibrated_sensor_token = get_calibration_token(calibrated_sensor_token_map_, device_name);
        if (calibrated_sensor_token.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
        }

        std::string relative_path = extractRelativePath(cloud_save_file);

        sample_data_json["token"] = sample_data_token;
        sample_data_json["sample_token"] = sample_token;
        sample_data_json["ego_pose_token"] = sample_data_token;
        sample_data_json["calibrated_sensor_token"] = calibrated_sensor_token;
        sample_data_json["timestamp"] = timestamp;
        sample_data_json["fileformat"] = "bin";  // 修改文件格式为 bin
        sample_data_json["is_key_frame"] = is_key_frame;
        sample_data_json["height"] = "";
        sample_data_json["width"] = "";
        sample_data_json["filename"] = relative_path;
        sample_data_json["prev"] = "";
        sample_data_json["next"] = "";

        generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
            sample_data_json, 
            "sample_data_" + device_name, 
            json_writer_, 
            "sample_data");

        devices_frame_count_[device_name]++;
    }

    void sample_recorder(uint64_t timestamp, std::string sample_token, std::string next_sample_token){
        // ********************generate sample_token, and write to sample.json and scene.json********************
        // adding samples to scene, wont be full
        if(scene_sample_frame_count_ < scene_frame_ - 1)
        {
            scene_processing(sample_token);

            json sample_json;
            sample_json["token"] = sample_token;
            sample_json["timestamp"] = timestamp;
            sample_json["prev"] = "";
            sample_json["next"] = "";
            sample_json["scene_token"] = cur_scene_token_;
            // generate sample's prev and next
            generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
            sample_json, 
            "sample_", 
            json_writer_, 
            "sample");
            // scene 
        }
        // the scene will be full, start a new scene
        else{
            json sample_json;
            sample_json["token"] = sample_token;
            sample_json["timestamp"] = timestamp;
            sample_json["prev"] = "";
            sample_json["next"] = "";
            sample_json["scene_token"] = cur_scene_token_;
            // generate sample's prev and next
            generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
            sample_json, 
            "sample_", 
            json_writer_, 
            "sample");
            // scene
            scene_done(sample_token, next_sample_token);
        }
    }

    void scene_init(std::string sample_token){
        if(scene_sample_frame_count_ == 0){
            // update_scene_token and reset scene_frame__count
            std::string cur_system_time_string = get_cur_system_time();
            cur_scene_token_ = "scene_" + map_name_ + "_" + cur_system_time_string; 
            std::string scene_name = scene_mode_ + "_" + map_name_ + "_" + std::to_string(cur_scene_num_) + "_" + start_system_time_token_; // Attention: nuscenes对每个场景有严格的划分nuscnes.splits函数可以划分那些事train、val还是test
            std::string description = "";
            latest_scene_["token"] = cur_scene_token_;
            latest_scene_["log_token"] = log_token_;
            latest_scene_["nbr_samples"] = scene_sample_frame_count_;
            latest_scene_["first_sample_token"] = sample_token;
            latest_scene_["last_sample_token"] = sample_token;
            latest_scene_["name"] = scene_name;
            latest_scene_["description"] = description;
        }
    }

    void scene_processing(std::string sample_token){
        // update count
        scene_sample_frame_count_++;

        latest_scene_["last_sample_token"] = sample_token;
        latest_scene_["nbr_samples"] = scene_sample_frame_count_;
    }

    void scene_done(std::string sample_token, std::string next_sample_token){
        // update count
        scene_sample_frame_count_++;
        // the scene is full, so write it
        latest_scene_["last_sample_token"] = sample_token;
        latest_scene_["nbr_samples"] = scene_sample_frame_count_;
        json_writer_.addKeyValuePairToJson("scene", latest_scene_, true); 
    
        latest_scene_.clear(); // clear cache
        scene_sample_frame_count_ = 0;
        cur_scene_num_++;
        scene_init(next_sample_token);
    }

    void scene_cache_write(){
        // 检查 latest_scene_ 是否为空
        if (!latest_scene_.empty() && latest_scene_["nbr_samples"] != 0) {
            json_writer_.addKeyValuePairToJson("scene", latest_scene_, true); 
            latest_scene_.clear(); // 清空缓存
        }
        // 重置帧计数器
        scene_sample_frame_count_ = 0;
    }

    void map_write(const json& log_json, const std::string& map_path, const std::string& map_name) {
        auto logger = this->get_logger();
        bool error_occurred = false; // 标记是否发生了错误

        try {
            // 打开并读取现有的 map.json 文件
            std::ifstream file(map_path);
            if (!file.is_open()) {
                RCLCPP_ERROR(logger, "Failed to open %s for reading.", map_path.c_str());
                error_occurred = true;
            } else {
                json map_data;
                try {
                    file >> map_data;
                } catch (const json::parse_error& e) {
                    RCLCPP_ERROR(logger, "JSON parse error in %s: %s", map_path.c_str(), e.what());
                    error_occurred = true;
                }
                file.close();
                RCLCPP_INFO(logger, "Successfully read map.json");

                if (!error_occurred) {
                    // 检查 map_data 是否为数组
                    if (!map_data.is_array()) {
                        RCLCPP_ERROR(logger, "map.json is not an array.");
                        error_occurred = true;
                    }
                }

                if (!error_occurred) {
                    // 检查 log_json 是否包含 "token" 字段
                    if (!log_json.contains("token") || log_json["token"].is_null() || log_json["token"].get<std::string>().empty()) {
                        RCLCPP_ERROR(logger, "Provided log_json does not contain a valid 'token' field.");
                        error_occurred = true;
                    }
                }

                if (!error_occurred) {
                    // 获取要添加的 token
                    std::string new_log_token = log_json["token"].get<std::string>();

                    // 查找正确的 map token
                    bool token_found = false;
                    for (auto& item : map_data) {
                        if (item.contains("token") && item["token"] == map_name) {
                            // 确保 "log_tokens" 是一个数组
                            if (!item.contains("log_tokens") || !item["log_tokens"].is_array()) {
                                item["log_tokens"] = json::array();
                            }

                            // 检查是否已经存在该 token，避免重复
                            bool already_exists = false;
                            for (const auto& existing_log_token : item["log_tokens"]) {
                                if (existing_log_token == new_log_token) {
                                    already_exists = true;
                                    break;
                                }
                            }

                            if (!already_exists) {
                                item["log_tokens"].push_back(new_log_token);
                                RCLCPP_INFO(logger, "Added log token '%s' to map token '%s'.", new_log_token.c_str(), map_name.c_str());
                            } else {
                                RCLCPP_INFO(logger, "Log token '%s' already exists for map token '%s'.", new_log_token.c_str(), map_name.c_str());
                            }

                            token_found = true;
                            break;
                        }
                    }

                    if (!token_found) {
                        RCLCPP_ERROR(logger, "Token '%s' not found in %s.", map_name.c_str(), map_path.c_str());
                        error_occurred = true;
                    }

                    if (!error_occurred) {
                        // 写回 map.json 文件
                        std::ofstream out_file(map_path, std::ios::trunc); // 使用 trunc 模式覆盖文件
                        if (!out_file.is_open()) {
                            RCLCPP_ERROR(logger, "Failed to open %s for writing.", map_path.c_str());
                            error_occurred = true;
                        } else {
                            out_file << map_data.dump(4); // 美化输出
                            out_file.close();
                            RCLCPP_INFO(logger, "Log token added successfully to %s.", map_path.c_str());
                        }
                    }
                }
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "Exception occurred: %s", e.what());
            error_occurred = true;
        }

        // 如果发生了错误，调用 shutdown 回调
        if (error_occurred) {
            this->node_shutdown_callback();
        }
    }

    bool check_sample_done(){
        // check sample done
        for(auto& sign_pair : sample_done_sign_){
            if(sign_pair.second == false){
                // clear sign, restart a epoch of sample
                for(auto& sign_pair : sample_done_sign_){
                    sign_pair.second = false;
                }
                RCLCPP_INFO(this->get_logger(), "Failed this sample epoch");
                return false;
            }
        }

        // check every sample is not null, if there is a null, return false
        bool all_samples_done = true;
        // Check image data cache
        for (auto& sample_img_cache : sample_imgs_cache_) {
            if (sample_img_cache.second.second.empty()) {
                all_samples_done = false; // Image data is null
                RCLCPP_ERROR(this->get_logger(), "Image data for device %s is empty", sample_img_cache.first.c_str());
            }
        }
        // Check lidar data cache
        if (sample_lidar_cache_.second.empty()) {
            all_samples_done = false; // Lidar data is null
            RCLCPP_ERROR(this->get_logger(), "Lidar data is empty");
        }
        // If any sample data is null, abort and return false
        if (!all_samples_done) {
            RCLCPP_ERROR(this->get_logger(), "Some sample data is missing, aborting this epoch.");
            return false;
        }

        // save sample
        for (auto& sample_img_cache : sample_imgs_cache_) {
            // 只在图像数据非空时进行保存
            if (!sample_img_cache.second.second.empty()) {
                std::string save_path = sample_img_cache.second.first;  // 获取图像文件保存路径
                std::vector<uint8_t> data = sample_img_cache.second.second;  // 获取缓存的图像数据
                // 保存缓存的图像数据
                std::ofstream ofs(save_path, std::ios::binary);
                if (ofs.is_open()) {
                    ofs.write(reinterpret_cast<const char*>(data.data()), data.size());  // 写入缓存数据到文件
                    ofs.close();
                    RCLCPP_INFO(this->get_logger(), "Image data saved from cache: %s", save_path.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving cached image: %s", save_path.c_str());
                }
                // 清空图像数据缓存（second 部分）
                sample_img_cache.second.second.clear();  // 清空图像数据部分
                RCLCPP_INFO(this->get_logger(), "Cache cleared after saving.");
            }
        }
        if (!sample_lidar_cache_.second.empty()) {
             // Retrieve cached data
            std::string save_path = sample_lidar_cache_.first;    // File path to save the point cloud
            std::vector<uint8_t> data = sample_lidar_cache_.second; // Cached lidar data
            // Save cached data to the file
            std::ofstream ofs(save_path, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(data.data()), data.size());  // Write cached data
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "Point cloud data saved from cache: %s", save_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving cached point cloud: %s", save_path.c_str());
            }
            // Clear the cache after saving
            sample_lidar_cache_.first.clear();
            sample_lidar_cache_.second.clear();
            RCLCPP_INFO(this->get_logger(), "Cache cleared after saving.");
        }
        
        sample_recorder(sample_recorder_cache_.first, sample_recorder_cache_.second.first, sample_recorder_cache_.second.second);

        if (sample_frame_ + 1 > target_sample_frame_) {
            node_shutdown_callback();
            return false;
        }
        else{
            sample_frame_++;
        }
        // Set all triggers
        for (auto& trigger : sample_triggers_) {
            trigger.second = true;
        }
        // clear sign
        for(auto& sign_pair : sample_done_sign_){
            sign_pair.second = false;
        }
        return true;
    } 

    void node_shutdown_callback(){
        // write last json in slider which have prev and next
        for (auto& slider : prev_next_sliders_) {
            if(slider.second.size() == 1){
                // prefix sample_data_
                if(slider.first.rfind("sample_data_", 0) == 0){
                    json_writer_.addKeyValuePairToJson("sample_data", slider.second[0], true);
                }
                // prefix sample_
                else if(slider.first.rfind("sample_", 0) == 0){
                    json_writer_.addKeyValuePairToJson("sample", slider.second[0], true);
                }
                else{
                    // 将last sample_annotation 写入 JSON 文件
                     json_writer_.addKeyValuePairToJson("sample_annotation", slider.second[0], true);
                }
            }
        }

        // write the scene which is not full
        scene_cache_write();

        RCLCPP_INFO(this->get_logger(), "*****************************sample finished*****************************");
        rclcpp::shutdown();  // 停止 ROS 节点
        return;  // 退出函数，确保不继续执行后续代码
    }

    std::string extractRelativePath(const std::string& data_path) {
        // 查找 "samples" 或 "sweeps" 的位置
        size_t pos = data_path.find("samples");
        if (pos == std::string::npos) {
            pos = data_path.find("sweeps");
        }

        // 如果找到了 "samples" 或 "sweeps"，返回从该位置到路径结尾的相对路径
        if (pos != std::string::npos) {
            return data_path.substr(pos);
        }

        // 如果都没有找到，返回空字符串
        return "";
    }

    std::string get_cur_system_time() {
        // 获取当前的系统时间
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    // Ensure a specific directory exists
    void ensure_directory_exists(const std::string &path) {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path); // 使用 C++17 的文件系统库来创建目录
            RCLCPP_INFO(this->get_logger(), "Directory created: %s", path.c_str());
        }
    }

    std::string root_path_;
    std::vector<std::string> json_paths_;
    
    std::string scene_mode_;
    std::string robot_model_; // specify the robot used for capturing data
    std::string map_name_;
    float sample_duration_; // sample every 0.5s
    std::unordered_map<std::string, std::string> sensor_token_map_;  // Initialize the unordered_map with device name (channel) -> token mapping

    std::unordered_map<std::string, std::string> calibrated_sensor_token_map_;
    int64_t target_sample_frame_;
    int scene_frame_; // the frame each scene should have
    std::string sample_scene_recoder_name_;
    std::string cur_scene_token_;
    double latest_time_;
    rclcpp::Time last_sample_time_;
    int64_t sample_frame_;
    std::unordered_map<std::string, bool> sample_triggers_;  // make devices to sample
    std::unordered_map<std::string, bool> sample_done_sign_;  // indicate sample done
    std::unordered_map<std::string, unsigned long long> devices_frame_count_;  // each device a frame count
    int scene_sample_frame_count_;
    int cur_scene_num_;
    json latest_scene_;  // the newest scene we record
    std::string log_token_;
    std::vector<std::string> camera_names_;
    std::vector<std::string> sample_sensors_;
    json latest_ego_pose_;
    json latest_imu_;
    std::string start_system_time_token_;
    std::unordered_map<std::string, std::vector<json>> prev_next_sliders_;  // this is used for all prev_next

    NuscenesJsonWriter json_writer_;

    std::string odom_topic_;
    std::string lidar_topic_;
    std::string imu_topic_;

    // samples全局缓存
    std::pair<std::string, std::vector<uint8_t>> sample_lidar_cache_;
    std::unordered_map<std::string, std::pair<std::string, std::vector<uint8_t>>> sample_imgs_cache_;
    std::pair<uint64_t, std::pair<std::string, std::string>> sample_recorder_cache_;


    rclcpp::TimerBase::SharedPtr clock_timer_;
    rclcpp::TimerBase::SharedPtr capture_image_timer_;
    std::shared_ptr<UsbCameraBridge> usb_cameras_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_for_driving_pub_;  // pub the CAM_FRONT for Driving
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

#endif  // NUSCENES_DATA_COLLECTOR_HPP


