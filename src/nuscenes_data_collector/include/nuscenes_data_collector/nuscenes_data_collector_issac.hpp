#ifndef NUSCENES_DATA_COLLECTOR_ISSAC_HPP
#define NUSCENES_DATA_COLLECTOR_ISSAC_HPP

#include "rclcpp/rclcpp.hpp"
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
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <future>

#include "nuscenes_data_collector/utils.hpp"
#include "nuscenes_data_collector/nuscenes_json_writer.hpp"
#include "nuscenes_data_collector/token_rule.hpp"

using json = nlohmann::json;

// 定义同步策略，仅包含两个消息类型
using ImgBbox3dSyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    vision_msgs::msg::Detection3DArray
>;

// issac use simulation time
class NuScenesDataCollectorIssac : public rclcpp::Node {
public:
    NuScenesDataCollectorIssac()
    : Node("nuscenes_data_collector"),
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
            "CAM_FRONT": "CALIBRATION_CAM_FRONT",
            "CAM_BACK": "CALIBRATION_CAM_BACK",
            "CAM_BACK_LEFT": "CALIBRATION_CAM_BACK_LEFT",
            "CAM_FRONT_LEFT": "CALIBRATION_CAM_FRONT_LEFT",
            "CAM_FRONT_RIGHT": "CALIBRATION_CAM_FRONT_RIGHT",
            "CAM_BACK_RIGHT": "CALIBRATION_CAM_BACK_RIGHT",
            "LIDAR_TOP": "CALIBRATION_LIDAR_TOP"
        })");

        // Declare other parameters
        this->declare_parameter<int64_t>("target_sample_frame_", 40);
        this->declare_parameter<int>("scene_frame_", 20);
        this->declare_parameter<std::string>("scene_mode_", "train");
        this->declare_parameter<std::string>("robot_model_", "diff_bot");
        this->declare_parameter<std::string>("map_name_", "office_issac");
        this->declare_parameter<int>("issac_per_sec_frame_", 60);
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
        this->get_parameter("issac_per_sec_frame_", issac_per_sec_frame_);
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

        // 创建一个大的日志，包含所有相关信息，并加入清晰的分隔符
        std::stringstream log_message;
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                     [Scene Info]                        \n";
        log_message << "----------------------------------------------------------\n";
        log_message << "Scene Mode: " << scene_mode_ << std::endl;
        log_message << "Robot Model: " << robot_model_ << std::endl;
        log_message << "Map Name: " << map_name_ << std::endl;
        log_message << "Root Path: " << root_path_ << std::endl;
        log_message << "Isaac Per Sec Frame: " << issac_per_sec_frame_ << std::endl;
        log_message << "Sample Frequency: " << sample_duration_ << std::endl;
        log_message << "Target Sample Frame: " << target_sample_frame_ << std::endl;
        log_message << "Scene Frame: " << scene_frame_ << std::endl;
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                    [Sample Sensors]                     \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& sensor : sample_sensors_) {
            log_message << "  - " << sensor << std::endl;
        }
        log_message << "\n----------------------------------------------------------\n";
        log_message << "                 [Sensor Token Map]                       \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& [key, value] : sensor_token_map_) {
            log_message << "  - " << key << ": " << value << std::endl;
        }
        log_message << "\n----------------------------------------------------------\n";
        log_message << "            [Calibrated Sensor Token Map]                  \n";
        log_message << "----------------------------------------------------------\n";
        for (const auto& [key, value] : calibrated_sensor_token_map_) {
            log_message << "  - " << key << ": " << value << std::endl;
        }
        // 输出到日志
        RCLCPP_INFO(this->get_logger(), "%s", log_message.str().c_str());


        if (json_writer_.isLoadError()) {
            RCLCPP_ERROR(this->get_logger(), "Json Load Error. Shutting down the node.");
            rclcpp::shutdown();
            return;
        }

        // 初始化其他成员变量
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
        last_sample_time_ = rclcpp::Time(0);
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

        uint64_t timestamp_ns = static_cast<uint64_t>(last_sample_time_.nanoseconds());
        std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
        std::string next_sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
        sample_recorder_cache_.first = timestamp_ns;
        sample_recorder_cache_.second.first = sample_token;
        sample_recorder_cache_.second.second = next_sample_token;
        // init the scene
        scene_init(sample_token);

        // init the sensor workflow
        initialize_sync_camera_bbox3d_subscriptions();

        rclcpp::QoS qos_profile(1);  // 队列大小为1，确保只处理最新的消息
        qos_profile.reliable();  // 设置可靠传输
        qos_profile.durability_volatile();  // 设置为volatile，防止缓存消息
        // 创建订阅者，订阅 /clock 主题，队列大小为10
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/issac/clock",
            qos_profile,
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                this->clock_callback(msg);
            }
        );
        // Odometry 订阅
        // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     odom_topic_, qos_profile, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        //         this->odom_callback(msg, "odom");
        //     }
        // );
        // IMU 订阅
        // imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //     imu_topic_,        // 替换为您的 IMU 主题名称，例如 "/sensor/imu"
        //     qos_profile,       // 使用之前定义的 QoS 配置
        //     [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        //         this->imu_callback(msg, "imu");  // 定义 IMU 回调函数以处理接收到的消息
        //     }
        // );

        // main character tf 订阅
        main_character_tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/issac/nuscenes/main_character", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                this->main_character_tf_callback(msg, "main_character_tf");  // 这里使用 Lambda 传递设备名称
            }
        );


        rclcpp::QoS lidar_qos_profile(50);
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
        // Declare odom_topic parameter
        this->declare_parameter<std::string>("odom_topic", "/issac/odom");
        RCLCPP_INFO(this->get_logger(), "Declared odom_topic with default: '/issac/odom'");

        // Initialize camera names and declare related parameters
        camera_names_ = {"CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"};
        for (const auto &camera : camera_names_) {
            camera_topics_[camera] = "/issac/" + camera;
            RCLCPP_INFO(this->get_logger(), "Declared camera topic parameter for %s: %s", camera.c_str(), camera_topics_[camera].c_str());

            bbox3d_topics_[camera] = "/issac/" + camera + "/bbox3d";
            RCLCPP_INFO(this->get_logger(), "Declared bbox3d topic parameter for %s: %s", camera.c_str(), bbox3d_topics_[camera].c_str());
        }
        // Get odom_topic and lidar_topic parameter
        //odom_topic_ = "/issac/nuscenes/odom";
        odom_topic_ = "/issac/nuscenes/main_character";
        RCLCPP_INFO(this->get_logger(), "Retrieved odom_topic: %s", odom_topic_.c_str());
        lidar_topic_ = "/issac/nuscenes/point_cloud";
        RCLCPP_INFO(this->get_logger(), "Retrieved lidar_topic: %s", lidar_topic_.c_str());
        //imu_topic_ = "/issac/nuscenes/imu";
        //RCLCPP_INFO(this->get_logger(), "Retrieved imu_topic: %s", imu_topic_.c_str());
    }

    // Initialize camera subscriptions dynamically
    // void initialize_camera_subscriptions() {
    //     for (const auto &camera : camera_names_) {
    //         rclcpp::QoS qos_profile(100);
    //         qos_profile.reliable();  // 设置可靠传输
    //         // Camera image subscription
    //         camera_subscriptions_[camera] = this->create_subscription<sensor_msgs::msg::Image>(
    //             camera_topics_[camera], qos_profile, [this, camera](const sensor_msgs::msg::Image::SharedPtr msg) {
    //                 std::async(std::launch::async, [this, msg, camera]() {
    //                     this->camera_callback(msg, camera);
    //                 }); 
    //             }
    //         );

    //         // Camera bbox3d subscription
    //         bbox3d_subscriptions_[camera] = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    //             bbox3d_topics_[camera], qos_profile, [this, camera](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    //                 std::async(std::launch::async, [this, msg, camera]() {
    //                     this->bbox3d_callback(msg, camera);
    //                 });
    //             }
    //         );
    //     }
    // }

    void initialize_sync_camera_bbox3d_subscriptions() {
        for (const auto &camera : camera_names_) {
            // 创建 Image 和 bbox3d 的订阅者并存储到成员变量中
            auto image_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                this, camera_topics_[camera], rmw_qos_profile_sensor_data);
            auto bbox3d_sub = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection3DArray>>(
                this, bbox3d_topics_[camera], rmw_qos_profile_sensor_data);

            image_subs_.push_back(image_sub);
            bbox3d_subs_.push_back(bbox3d_sub);

            // 创建同步器并存储到成员变量中
            auto sync = std::make_shared<message_filters::Synchronizer<ImgBbox3dSyncPolicy>>(
                ImgBbox3dSyncPolicy(100), // 队列大小
                *image_sub,
                *bbox3d_sub
            );

            // Error: Lambda error, can't adapt the template
            // sync->registerCallback(
            //     [this, camera](const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
            //                    const vision_msgs::msg::Detection3DArray::ConstSharedPtr& bbox3d_msg) {
            //         this->synced_camera_bbox3d_callback(image_msg, bbox3d_msg, camera);
            //     });
            sync->registerCallback(std::bind(&NuScenesDataCollectorIssac::synced_camera_bbox3d_callback, 
                                    this, std::placeholders::_1, std::placeholders::_2, camera));
            synchronizers_.push_back(sync);
        }
    }

    void synced_camera_bbox3d_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& bbox3d_msg,
        const std::string &camera_name
    ) {
        RCLCPP_INFO(this->get_logger(), "Received synced data for camera: %s", camera_name.c_str());
        try {
            if ((!latest_ego_pose_.empty()) && (!latest_imu_.empty())) {
                bool is_sample_trigger = sample_triggers_[camera_name];
                save_image_data(image_msg, camera_name);
                save_bbox3d_data(bbox3d_msg, camera_name, is_sample_trigger);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception occurred in img save_data: %s", e.what());
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

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // 获取当前simulation时间的 sec 和 nanosec
        int64_t sec = msg->clock.sec;
        uint32_t nanosec = msg->clock.nanosec;
        rclcpp::Time cur_time(sec, nanosec);
        rclcpp::Duration time_diff = cur_time - last_sample_time_;

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
        }
    }

    // Odometry data callback
    // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &device_name) {
    //     RCLCPP_INFO(this->get_logger(), "Received odometry data");
    //     save_data(msg, device_name);
    // }

    // void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg, const std::string &device_name)
    // {
    //     save_data(msg, device_name);
    // }

    void main_character_tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg,  const std::string &device_name){
        RCLCPP_INFO(this->get_logger(), "Received ego pose data");
        save_data(msg, device_name);
    }

    /// Template function to save data to sweeps directory
    template <typename T>
    void save_data(const T &msg, const std::string &device_name) {
        if (sample_frame_ >= target_sample_frame_) {
            node_shutdown_callback();
            return;
        }

        try {
            // Check if the message type is Odometry
            // if constexpr (std::is_same_v<T, std::shared_ptr<nav_msgs::msg::Odometry>>) {
            //     save_odom_data(msg);
            // }
            if constexpr (std::is_same_v<T, std::shared_ptr<tf2_msgs::msg::TFMessage>>) {
                save_main_character_tf(msg);
                cal_imu_data();
            }            
            // Check if the message type is IMU and we have valid ego_pose
            // else if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::Imu>>) {
            //     if (!latest_ego_pose_.empty()) {
            //         save_imu_data(msg);
            //     }
            // }
            // Ensure we must have an ego pose and imu before processing other types
            if ((!latest_ego_pose_.empty()) && (!latest_imu_.empty())) {
                // Handle Image messages
                if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::Image>>) {
                    save_image_data(msg, device_name);
                }
                // Handle Detection3DArray messages
                else if constexpr (std::is_same_v<T, std::shared_ptr<vision_msgs::msg::Detection3DArray>>) {
                    save_bbox3d_data(msg, device_name);
                }
                // Handle PointCloud2 (Lidar) messages
                else if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::PointCloud2>>) {
                    save_lidar_data(msg, device_name);
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception caught while saving data: %s", e.what());
            // Optionally return or continue depending on the behavior you want
        }
    }

    // write odom data in nuscenes ego_pose format and save to ego_pose.json
    // void save_odom_data(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
    //     json ego_pose_data;
    //     // The timestamp can be a combination of seconds and nanoseconds
    //     uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    //     ego_pose_data["timestamp"] = timestamp;
    //     ego_pose_data["rotation"] = {
    //         msg->pose.pose.orientation.x,
    //         msg->pose.pose.orientation.y,
    //         msg->pose.pose.orientation.z,
    //         msg->pose.pose.orientation.w
    //     };
    //     ego_pose_data["translation"] = {
    //         msg->pose.pose.position.x,
    //         msg->pose.pose.position.y,
    //         msg->pose.pose.position.z
    //     };
    //     latest_ego_pose_ = ego_pose_data;
    // }

    void save_main_character_tf(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
        if (msg->transforms.empty()) {
            return;
        }
        
        // Get the first TransformStamped from the TFMessage
        const auto& transform = msg->transforms.front();
        
        // Access the timestamp from the transform header
        uint64_t timestamp = transform.header.stamp.sec * 1e9 + transform.header.stamp.nanosec;
        
        // Create JSON to store pose data
        json ego_pose_data;
        ego_pose_data["timestamp"] = timestamp;
        
        // Access rotation (Quaternion) data
        ego_pose_data["rotation"] = json::array({
            transform.transform.rotation.w,  // w放在第一位
            transform.transform.rotation.x,  // x放在第二位
            transform.transform.rotation.y,  // y放在第三位
            transform.transform.rotation.z   // z放在第四位
        });
        
        // Access translation (Position) data
        ego_pose_data["translation"] = json::array({
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        });
    
        // If previous ego pose exists, save it
        if (!latest_ego_pose_.empty()) {
            previous_ego_pose_ = latest_ego_pose_;
        }
        latest_ego_pose_ = ego_pose_data;
    }    
    
    
    /// this is for Imu Sensor
    // void save_imu_data(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
    //     // 提取线性加速度（linear_accel）
    //     auto linear_accel = msg->linear_acceleration;
    //     // 提取四元数（q）
    //     auto orientation = msg->orientation;
    //     // 提取角速度（rotation_rate）
    //     auto angular_velocity = msg->angular_velocity;
    //     // 获取消息的时间戳并转换为微秒（utime）
    //     uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    //     // 将geometry_msgs::msg::Quaternion转为JSON数组
    //     nlohmann::json orientation_json = {orientation.x, orientation.y, orientation.z, orientation.w};
    //     // 将geometry_msgs::msg::Vector3转为JSON数组
    //     nlohmann::json angular_velocity_json = {angular_velocity.x, angular_velocity.y, angular_velocity.z};
    //     nlohmann::json linear_accel_json = {linear_accel.x, linear_accel.y, linear_accel.z};
    //     // 保存数据到latest_imu_字典
    //     latest_imu_["linear_accel"] = linear_accel_json;  // 线性加速度
    //     latest_imu_["q"] = orientation_json;              // 四元数
    //     latest_imu_["rotation_rate"] = angular_velocity_json;  // 角速度
    //     latest_imu_["pos"] = latest_ego_pose_["translation"];  // 位置
    //     latest_imu_["utime"] = timestamp;  // 保存计算的utime
    // }

    /// @brief 使用插值法计算加速度、角速度
    void cal_imu_data() {
        // 1. 确保有足够的数据
        if (latest_ego_pose_.empty() || previous_ego_pose_.empty()) {
            return;  // 如果没有足够的历史数据，无法计算
        }
    
        // 获取当前位置和上一个位置的差值，计算速度
        auto current_position = latest_ego_pose_["translation"];
        auto previous_position = previous_ego_pose_["translation"];
        
        // 计算位置差值（单位：米）
        double delta_x = current_position[0].get<double>() - previous_position[0].get<double>();
        double delta_y = current_position[1].get<double>() - previous_position[1].get<double>();
        double delta_z = current_position[2].get<double>() - previous_position[2].get<double>();
    
        // 通过时间戳计算
        double time_interval = (latest_ego_pose_["timestamp"].get<uint64_t>() - previous_ego_pose_["timestamp"].get<uint64_t>()) * 1e-9;  // 时间间隔以秒为单位
        if (time_interval <= 0) {
            return; // 如果时间间隔无效，返回
        }
    
        // 计算速度（单位：米/秒）
        double velocity_x = delta_x / time_interval;
        double velocity_y = delta_y / time_interval;
        double velocity_z = delta_z / time_interval;
    
        // 2. 计算线性加速度（速度的变化率）
        static double last_velocity_x = velocity_x;
        static double last_velocity_y = velocity_y;
        static double last_velocity_z = velocity_z;
    
        // 计算加速度（单位：米/秒^2）
        double accel_x = (velocity_x - last_velocity_x) / time_interval;
        double accel_y = (velocity_y - last_velocity_y) / time_interval;
        double accel_z = (velocity_z - last_velocity_z) / time_interval;
    
        // 更新上一时刻的速度
        last_velocity_x = velocity_x;
        last_velocity_y = velocity_y;
        last_velocity_z = velocity_z;
    
        // 3. 计算角速度 - 使用四元数正确的方法
        auto current_rotation = latest_ego_pose_["rotation"];
        auto previous_rotation = previous_ego_pose_["rotation"];
        
        // 四元数的顺序是 wxyz
        double q1w = previous_rotation[0].get<double>();
        double q1x = previous_rotation[1].get<double>();
        double q1y = previous_rotation[2].get<double>();
        double q1z = previous_rotation[3].get<double>();
        
        double q2w = current_rotation[0].get<double>();
        double q2x = current_rotation[1].get<double>();
        double q2y = current_rotation[2].get<double>();
        double q2z = current_rotation[3].get<double>();
        
        // 计算四元数的差值四元数: q_diff = q2 * q1^(-1)
        // 计算q1的共轭（逆）
        double q1w_inv = q1w;
        double q1x_inv = -q1x;
        double q1y_inv = -q1y;
        double q1z_inv = -q1z;
        
        // 归一化以确保是单位四元数
        double norm = std::sqrt(q1w_inv*q1w_inv + q1x_inv*q1x_inv + q1y_inv*q1y_inv + q1z_inv*q1z_inv);
        q1w_inv /= norm;
        q1x_inv /= norm;
        q1y_inv /= norm;
        q1z_inv /= norm;
        
        // 四元数乘法：q_diff = q2 * q1^(-1)
        double diff_w = q2w * q1w_inv - q2x * q1x_inv - q2y * q1y_inv - q2z * q1z_inv;
        double diff_x = q2w * q1x_inv + q2x * q1w_inv + q2y * q1z_inv - q2z * q1y_inv;
        double diff_y = q2w * q1y_inv - q2x * q1z_inv + q2y * q1w_inv + q2z * q1x_inv;
        double diff_z = q2w * q1z_inv + q2x * q1y_inv - q2y * q1x_inv + q2z * q1w_inv;
        
        // 从四元数提取角轴和角度
        // 如果四元数表示小旋转，diff_w 接近 1，角度接近 0
        double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, diff_w))); // 限制在[-1,1]范围内
        
        // 避免除以零
        double s = std::sqrt(1.0 - diff_w * diff_w);
        double angular_velocity_x, angular_velocity_y, angular_velocity_z;
        
        if (s < 1e-6) {
            // 如果旋转很小，使用近似值
            angular_velocity_x = 2.0 * diff_x / time_interval;
            angular_velocity_y = 2.0 * diff_y / time_interval;
            angular_velocity_z = 2.0 * diff_z / time_interval;
        } else {
            // 正常情况，计算角速度矢量
            angular_velocity_x = (angle * diff_x / s) / time_interval;
            angular_velocity_y = (angle * diff_y / s) / time_interval;
            angular_velocity_z = (angle * diff_z / s) / time_interval;
        }
    
        // 4. 更新 IMU 数据
        latest_imu_["linear_accel"] = {accel_x, accel_y, accel_z};  // 线性加速度
        latest_imu_["rotation_rate"] = {angular_velocity_x, angular_velocity_y, angular_velocity_z};  // 角速度
        latest_imu_["q"] = current_rotation;  // 四元数
        latest_imu_["pos"] = current_position;  // 位置
        latest_imu_["utime"] = latest_ego_pose_["timestamp"];  // 使用最新的时间戳
    }
    

    // Save Image data as JPEG and write it to sampe_data.json in the nuscenes sample_data format
    void save_image_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg, 
                              const std::string &device_name) {
        // Determine whether to save to 'sweeps' or 'samples'
        std::string data_type = (sample_triggers_[device_name] == true) ? "samples" : "sweeps";
        bool is_save = data_type == "samples" ? false : true; // samples save to the tmp at first

        // Create the directory path for saving images
        std::string folder_path = root_path_ + "/" + data_type + "/" + device_name;
        ensure_directory_exists(folder_path);

        // Generate a file path with timestamp for the image
        std::stringstream ss;
        ss << folder_path << '/' << start_system_time_token_ << "__" << device_name << "__" << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".jpg";
        std::string image_save_file = ss.str();

        // Convert encoding to lowercase for consistent comparison
        std::string encoding = msg->encoding;
        std::transform(encoding.begin(), encoding.end(), encoding.begin(), [](unsigned char c) { return std::tolower(c); });

        cv::Mat img;

        if (encoding == sensor_msgs::image_encodings::BGR8 ||
            encoding == sensor_msgs::image_encodings::RGB8) {
            // Directly use image data if encoding is compatible
            img = cv_bridge::toCvShare(msg, encoding)->image;

            // If the image encoding is RGB, convert it to BGR
            if (encoding == sensor_msgs::image_encodings::RGB8) {
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            }
        } else if (encoding == "jpeg" || encoding == "jpg") {
            // Decode JPEG image
            std::vector<uchar> img_data(msg->data.begin(), msg->data.end());
            img = cv::imdecode(img_data, cv::IMREAD_COLOR);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }

        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to decode image");
            return;
        }
        // Encode image as JPEG
        std::vector<uchar> encoded_img;
        bool success = cv::imencode(".jpg", img, encoded_img);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode image to JPEG");
            return;
        }
        // Save the image file
        if(is_save){
            std::ofstream ofs(image_save_file, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(encoded_img.data()), encoded_img.size());
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "Image saved as JPEG: %s", image_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open image file for writing: %s", image_save_file.c_str());
                return;
            }
        }
        else{
            // Cache image data (for 'samples')
            sample_imgs_cache_[device_name] = {image_save_file, encoded_img};
            // set the sign that camera sample done in this epoch
            sample_triggers_[device_name] = false;
            sample_done_sign_[device_name] = true;
        }

        // Now, prepare the JSON data based on the format you provided
        nlohmann::json sample_data_json;
        // formula: sample_data_token = harsh(frame, "samples_data_" + device_name, start_system_time_token_)
        std::string sample_data_token_tmp = "samples_data_" + device_name;
        std::string sample_data_token = generate_unique_token(devices_frame_count_[device_name], device_name, start_system_time_token_);
        // write ego_pose
        latest_ego_pose_["token"] = sample_data_token;  // set as sample_data_token
        json_writer_.addKeyValuePairToJson("ego_pose", latest_ego_pose_, true);
        // get time_stamp and key_frame sign
        uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
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
        std::string calibrated_sensor_token = get_calibration_token(calibrated_sensor_token_map_ ,device_name);
        if(calibrated_sensor_token == "")
            RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
        // Prepare the JSON fields
        sample_data_json["token"] = sample_data_token;
        sample_data_json["sample_token"] = sample_token;  // 指向sample_data所关联的sample
        sample_data_json["ego_pose_token"] = sample_data_token;
        sample_data_json["calibrated_sensor_token"] = calibrated_sensor_token;  
        sample_data_json["timestamp"] = timestamp;  // Full timestamp
        sample_data_json["fileformat"] = "jpg";  // File format is JPEG
        sample_data_json["is_key_frame"] = is_key_frame;  // Example, this could be dynamic based on your application
        sample_data_json["height"] = img.rows;
        sample_data_json["width"] = img.cols;
        sample_data_json["filename"] = relative_path;
        sample_data_json["prev"] = "";  // Example, replace with actual previous token if needed
        sample_data_json["next"] = "";  // Example, replace with actual next token if needed
        // generate sample_data's prev and next
        generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
        sample_data_json, 
        "sample_data_" + device_name, 
        json_writer_, 
        "sample_data");
        
        devices_frame_count_[device_name]++;
        //RCLCPP_ERROR(this->get_logger(), "*************************");
    }

    void save_bbox3d_data(const std::shared_ptr<const vision_msgs::msg::Detection3DArray> &msg, 
                            const std::string &device_name, bool is_sample_trigger) {
        std::string data_type = (is_sample_trigger == true) ? "samples" : "sweeps";
        if(data_type == "samples"){
            // 用于保存所有结果的JSON对象
            std::vector<nlohmann::json> sample_annotation_jsons;
            // 生成 sample_token
            std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            // 遍历 Detection3DArray 中的每个检测结果
            for (const auto &detection : msg->detections) {
                nlohmann::json sample_annotation_json;
                // 获取class_id
                std::string class_id = detection.results[0].hypothesis.class_id;
                // filter the bbox3d from different view
                if (is_class_annotated_.find(class_id) != is_class_annotated_.end() && is_class_annotated_[class_id]) {
                    continue;
                }
                is_class_annotated_[class_id] = true;
                instance_annotated_count_[class_id]++;  // If class_id exists, it increments by 1, if not, it creates with value 1.                
                // 生成 instance_token
                std::string instance_token = generate_unique_token(0, class_id, start_system_time_token_);
                // 生成 sample_annotation_token
                std::string sample_annotation_token_tmp = "3dbox_" + instance_token;
                std::string sample_annotation_token = generate_unique_token(instance_annotated_count_[class_id], sample_annotation_token_tmp, start_system_time_token_);
                // 生成 visibility_token
                double score = detection.results[0].hypothesis.score;
                std::string visibility_token = generate_visibility_token(score);
                // 填充检测结果 JSON
                sample_annotation_json["token"] = sample_annotation_token;
                sample_annotation_json["sample_token"] = sample_token;
                sample_annotation_json["instance_token"] = instance_token;
                sample_annotation_json["visibility_token"] = visibility_token;
                sample_annotation_json["attribute_tokens"] = nlohmann::json::array({"ab83627ff28b465b85c427162dec722f"}); // pedestrain.moving
                // translation：从 bbox 中获取位置
                const auto &bbox = detection.bbox;
                // z坐标减去物体高度的一半, 作为isaac数据的补偿
                sample_annotation_json["translation"] = {
                    bbox.center.position.x, 
                    bbox.center.position.y, 
                    bbox.center.position.z - (bbox.size.z / 2.0)  // 减去物体高度的一半
                };
                // rotation：从 bbox 中获取方向（四元数）
                sample_annotation_json["rotation"] = {bbox.center.orientation.w, bbox.center.orientation.x, bbox.center.orientation.y, bbox.center.orientation.z};
                // size：从 bbox 数据获取
                sample_annotation_json["size"] = {bbox.size.x, bbox.size.y, bbox.size.z};
                // 填充 prev 和 next
                sample_annotation_json["prev"] = "";
                sample_annotation_json["next"] = "";  // this will resolve in slider
                // 填充 num_lidar_pts 和 num_radar_pts
                sample_annotation_json["num_lidar_pts"] = 99999;  
                sample_annotation_json["num_radar_pts"] = 99999;  
                
                sample_annotation_jsons.emplace_back(sample_annotation_json);
            }
            //RCLCPP_INFO(this->get_logger(), "Total sample annotations: %zu", sample_annotation_jsons.size());
            sample_bbox3d_json_cache_[device_name] = sample_annotation_jsons;
        }
    }

    /// @brief TODO:intensity is fake,which set to 1.0, to adapt the nuscenes' pcl format
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
        pcl::PointCloud<pcl::PointXYZ> cloud;  // 使用 PointXYZ
        
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

        float default_intensity = 1.0f;  // 默认强度值
        // Save point cloud to binary file in NuScenes format
        if(is_save){
            std::ofstream ofs(cloud_save_file, std::ios::binary);
            if (ofs.is_open()) {
                // Write the point cloud data to file in [x, y, z, intensity] format
                for (const auto &point : cloud.points) {
                    ofs.write(reinterpret_cast<const char*>(&point.x), sizeof(float)); // x
                    ofs.write(reinterpret_cast<const char*>(&point.y), sizeof(float)); // y
                    ofs.write(reinterpret_cast<const char*>(&point.z), sizeof(float)); // z
                    ofs.write(reinterpret_cast<const char*>(&default_intensity), sizeof(float)); // intensity (default)
                }
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "PointCloud2 saved as BIN: %s", cloud_save_file.c_str());
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
                point_cloud_data.insert(point_cloud_data.end(), reinterpret_cast<const uint8_t*>(&default_intensity), reinterpret_cast<const uint8_t*>(&default_intensity) + sizeof(float));
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
                // 如果缓存的图像已经是 JPEG 编码格式，你可以直接用 cv::imwrite 来保存
                std::string save_path = sample_img_cache.second.first;
                std::vector<uint8_t> data = sample_img_cache.second.second;  // 获取缓存的图像数据
                cv::Mat cached_img = cv::imdecode(data, cv::IMREAD_COLOR);
                if (!cached_img.empty()) {
                    cv::imwrite(save_path, cached_img);
                    RCLCPP_INFO(this->get_logger(), "Image data saved from cache: %s", save_path.c_str());
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

        for(auto& bbox3d_jsons : sample_bbox3d_json_cache_){
            // this have not to return false when it is empty
            if (!bbox3d_jsons.second.empty()) {
                // sample_bbox3d_json_cache_ 存储3D边界框数据
                for(auto& one_bbox_json : bbox3d_jsons.second){
                    // ******************update the instance global cache json********************
                    std::string instance_token = one_bbox_json["instance_token"];
                    std::string sample_annotation_token = one_bbox_json["token"];

                    auto& val = instance_recorders[instance_token];  // get the global instance cache json
                    nlohmann::json instance_json; 
                    if (val.empty()) {
                        // 如果 vector 为空，初始化一个新的 JSON 对象
                        instance_json["token"] = instance_token;
                        instance_json["category_token"] = "1fa93b757fc74fb197cdd60001ad8abf";  // only human.pedestrian.adult, 
                        instance_json["nbr_annotations"] = 1;
                        instance_json["first_annotation_token"] = sample_annotation_token;
                        instance_json["last_annotation_token"] = sample_annotation_token;
                        // 将该 JSON 对象插入到 unordered_map 对应的 vector 中
                        val.push_back(instance_json);
                    } else {
                        // 如果 vector 不为空，增加一个注解数量
                        instance_json = val.back(); // 取出最后一个元素，假设每次更新是对最后一个 JSON 对象的更新
                        instance_json["nbr_annotations"] = instance_json["nbr_annotations"].get<int>() + 1;
                        instance_json["last_annotation_token"] = sample_annotation_token; // update the last_annotation_token
                        // 更新 vector 中的最后一个 JSON 对象
                        val.back() = instance_json;
                    }

                    // save the corresponding bbox3d
                    std::string instance_key = instance_token;
                    generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
                        one_bbox_json, 
                        instance_key, 
                        json_writer_, 
                        "sample_annotation");
                }
                // 清空缓存
                bbox3d_jsons.second.clear();
                RCLCPP_INFO(this->get_logger(), "bbox3d saved.");
            }
        }
        is_class_annotated_.clear();
        
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
        // write all unique instance in one scene
        for(auto& key_val : instance_recorders){
            json& instance_json = key_val.second.front();
            json_writer_.addKeyValuePairToJson("instance", instance_json, true);
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
        // 格式化时间，去掉空格和冒号
        ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d%H%M%S");
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
    int issac_per_sec_frame_;
    float sample_duration_; // sample every 0.5s
    std::unordered_map<std::string, std::string> sensor_token_map_; // Initialize the unordered_map with device name (channel) -> token mapping
    std::unordered_map<std::string, std::string> calibrated_sensor_token_map_;
    int64_t target_sample_frame_;
    int scene_frame_; // the frame each scene should have
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
    json previous_ego_pose_;
    json latest_imu_;
    std::string start_system_time_token_;
    std::unordered_map<std::string, std::vector<json>> prev_next_sliders_;  // this is for all prev next 
    std::unordered_map<std::string, std::vector<json>> instance_recorders;  // this is used for all instance's last annotation, each instance have an unique token in one scene

    NuscenesJsonWriter json_writer_;

    std::unordered_map<std::string, std::string> camera_topics_;
    std::unordered_map<std::string, std::string> bbox3d_topics_;
    std::string odom_topic_;
    std::string lidar_topic_;
    std::string imu_topic_;

    // samples全局缓存
    std::pair<std::string, std::vector<uint8_t>> sample_lidar_cache_;
    std::unordered_map<std::string, std::pair<std::string, std::vector<uint8_t>>> sample_imgs_cache_;
    std::unordered_map<std::string, std::vector<nlohmann::json>> sample_bbox3d_json_cache_;
    std::unordered_map<std::string, bool>is_class_annotated_;  // whether this bbox3d is annotated in this sample
    std::unordered_map<std::string, unsigned long long> instance_annotated_count_;
    std::pair<uint64_t, std::pair<std::string, std::string>> sample_recorder_cache_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr> bbox3d_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr main_character_tf_subscriber_;
    // Store synchronizers to keep them alive
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> image_subs_;
    std::vector<std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection3DArray>>> bbox3d_subs_;
    std::vector<std::shared_ptr<void>> synchronizers_;

};

#endif  // NUSCENES_DATA_COLLECTOR_ISSAC_HPP