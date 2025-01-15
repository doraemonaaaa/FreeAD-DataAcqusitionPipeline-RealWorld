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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// message sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <future>
#include <boost/bind.hpp>

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
class NuScenesDataCollector : public rclcpp::Node {
public:
    NuScenesDataCollector()
    : Node("nuscenes_data_collector"),              // 初始化 Node 名称
      root_path_("/home/pyh/Documents/ros2_ws/RobotAD_Issac_ws/NuscenesData"),   // 使用 '/' 拼接路径
      json_paths_({ // json init
          root_path_ + "/ego_pose.json", 
          root_path_ + "/sample_data.json", 
          root_path_ + "/sample_annotation.json",
          root_path_ + "/sample.json",
          root_path_ + "/scene.json",
          root_path_ + "/log.json",
          root_path_ + "/instance.json"
      }),
      json_writer_(json_paths_)                     // 将 logger 传递给 json_writer_
    {
        // 检查 JSON 加载是否有错误
        if (json_writer_.isLoadError()) {
            RCLCPP_ERROR(this->get_logger(), "Json Load Error. Shutting down the node.");
            rclcpp::shutdown();  // 停止 ROS 节点
            return;  // 退出函数，确保不继续执行后续代码
        }

        // 初始化其他参数和订阅
        declare_parameters();
        //initialize_camera_subscriptions();
        initialize_sync_camera_bbox3d_subscriptions();

        rclcpp::QoS qos_profile(100);
        qos_profile.reliable();  // 设置可靠传输
        // 创建订阅者，订阅 /clock 主题，队列大小为10
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock",
            qos_profile,
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                std::async(std::launch::async, [this, msg]() {
                    this->clock_callback(msg);
                });
            }
        );

        // Odometry 订阅
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos_profile, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::async(std::launch::async, [this, msg]() {
                    this->odom_callback(msg);
                });
            }
        );

        // lidar 订阅
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, qos_profile, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::async(std::launch::async, [this, msg]() {
                    this->lidar_callback(msg);
                });
            }
        );
        
        // 传感器集初始化
        for(auto c: camera_names_) add_sample_sensors(c);
        add_sample_sensors("LIDAR_TOP");
        for(auto device : sample_sensors_){
            devices_frame_count_[device] = 0;
            sample_triggers_[device] = true;
            sample_done_sign_[device] = false;
        }
        start_system_time_token_ = get_cur_system_time();
        sample_interval_ = static_cast<int>(issac_per_sec_frame * sample_frequency);
        scene_sample_frame_count_ = 0;
        std::string cur_system_time_string = get_cur_system_time();
        cur_scene_token_ = "scene_" + scene_name_ + "_" + cur_system_time_string;  // init the first scene_token
        last_sample_time_ = rclcpp::Time(0);
        sample_frame_ = 0;
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
        odom_topic_ = "/issac/odom";
        RCLCPP_INFO(this->get_logger(), "Retrieved odom_topic: %s", odom_topic_.c_str());
        lidar_topic_ = "/issac/point_cloud";
        RCLCPP_INFO(this->get_logger(), "Retrieved lidar_topic: %s", lidar_topic_.c_str());
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
                ImgBbox3dSyncPolicy(10), // 队列大小
                *image_sub,
                *bbox3d_sub
            );

            // Lambda error, can't adapt the template
            // sync->registerCallback(
            //     [this, camera](const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
            //                    const vision_msgs::msg::Detection3DArray::ConstSharedPtr& bbox3d_msg) {
            //         this->synced_camera_bbox3d_callback(image_msg, bbox3d_msg, camera);
            //     });
            sync->registerCallback(std::bind(&NuScenesDataCollector::synced_camera_bbox3d_callback, this, std::placeholders::_1, std::placeholders::_2, camera));
            
            synchronizers_.push_back(sync);
        }
    }

    void synced_camera_bbox3d_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& bbox3d_msg,
        const std::string &camera_name
    ) {
        RCLCPP_INFO(this->get_logger(), "Received synced data for camera: %s", camera_name.c_str());
        save_data(image_msg, camera_name);
        save_data(bbox3d_msg, camera_name);
        sample_triggers_[camera_name] = false; // after processing clear the sign
    }

    // lidar data callback
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received lidar_top data");
        save_data(msg, "LIDAR_TOP");
        sample_triggers_["LIDAR_TOP"] = false; // after processing clear the sign
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // 获取当前时间的 sec 和 nanosec
        int64_t sec = msg->clock.sec;
        uint32_t nanosec = msg->clock.nanosec;
        rclcpp::Time cur_time(sec, nanosec);
        rclcpp::Duration time_diff = cur_time - last_sample_time_;
        if(sample_frame_< target_sample_frame_){
            if(time_diff.seconds() >= 0.5){
                RCLCPP_INFO(this->get_logger(), "sample trigger at time: %f", cur_time.seconds());
                last_sample_time_ = cur_time;
                // set command
                for(auto& trigger : sample_triggers_){
                    trigger.second = true;
                }
            }
        }
        else{
            node_shutdown_callback();
            return;
        }
        check_sample_done();
    }

    // Odometry data callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        save_data(msg, "odom");
    }

    /// Template function to save data to sweeps directory
    template <typename T>
    void save_data(const T &msg, const std::string &device_name) {
        // RCLCPP_INFO(this->get_logger(), "Template type T: %s", typeid(T).name());
        // RCLCPP_INFO(this->get_logger(), "Message type: %s", typeid(msg).name());
        // Handle Odometry messages
        if constexpr (std::is_same_v<T, std::shared_ptr<nav_msgs::msg::Odometry>>) {
            save_odom_data(msg);
        }
        // Handle Image messages
        else if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::Image>>) {
            save_image_data(msg, device_name);
        }
        // Handle Detection3DArray messages
        else if constexpr (std::is_same_v<T, std::shared_ptr<vision_msgs::msg::Detection3DArray>>) {
            save_bbox3d_data(msg, device_name);
        }
        else if constexpr (std::is_same_v<T, std::shared_ptr<sensor_msgs::msg::PointCloud2>>) {
            save_lidar_data(msg, device_name);
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
        latest_ego_pose = ego_pose_data;
    }


    // Save Image data as JPEG and write it to sampe_data.json in the nuscenes sample_data format
    void save_image_data(const std::shared_ptr<sensor_msgs::msg::Image> &msg, 
                              const std::string &device_name) {
        try {
            // Determine whether to save to 'sweeps' or 'samples'
            std::string data_type = (sample_triggers_[device_name] == true) ? "samples" : "sweeps";
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
            }
            else if (encoding == "jpeg" || encoding == "jpg") {
                // Decode JPEG image
                std::vector<uchar> img_data(msg->data.begin(), msg->data.end());
                img = cv::imdecode(img_data, cv::IMREAD_COLOR);
            } 
            else {
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
            std::ofstream ofs(image_save_file, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(encoded_img.data()), encoded_img.size());
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "Image saved as JPEG: %s", image_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open image file for writing: %s", image_save_file.c_str());
                return;
            }
    
            // Now, prepare the JSON data based on the format you provided
            nlohmann::json sample_data_json;
            // formula: sample_data_token = harsh(frame, "samples_data_" + device_name, start_system_time_token_)
            std::string sample_data_token_tmp = "samples_data_" + device_name;
            std::string sample_data_token = generate_unique_token(devices_frame_count_[device_name], device_name, start_system_time_token_);
            // get ego_pose and save it
            latest_ego_pose["token"] = sample_data_token;  // set as sample_data_token
            json_writer_.addKeyValuePairToJson("ego_pose", latest_ego_pose, true);
            // get time_stamp and key_frame sign
            uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
            bool is_key_frame = data_type == "samples" ? true : false;
            // ********************generate sample_token, and write to sample.json and scene.json********************
            std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            if(is_key_frame){
                // adding samples to scene, wont be full
                if(scene_sample_frame_count_ < scene_frame_ - 1)
                {
                    json sample_json;
                    sample_json["token"] = sample_token;
                    sample_json["timestamp"] = timestamp;
                    sample_json["prev"] = "";
                    sample_json["next"] = "";
                    sample_json["scene_token"] = cur_scene_token_;
                    // generate sample's prev and next
                    generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
                    sample_json, 
                    "sample" + device_name, 
                    json_writer_, 
                    "sample");

                    // scene 
                    scene_processing(device_name, sample_token);
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
                    "sample" + device_name, 
                    json_writer_, 
                    "sample");

                    // scene
                    scene_done(device_name, sample_token);
                }
                sample_done_sign_[device_name] = true;
            }
            // Extract the path from the image path, keeping only the part after 'sweeps' or 'samples'
            std::string relative_path = extractRelativePath(image_save_file);
            // get calibrated sensor token 
            std::string calibrated_sensor_token = get_calibration_token(calibrated_sensor_token_map_ ,device_name);
            if(calibrated_sensor_token == "")
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
            // cal the previous token and next token for the current device
            // std::tuple<std::string, std::string> sample_result = generate_sample_prev_and_next(sample_frame_);
            // std::string prev_token = std::get<0>(sample_result);
            // std::string next_token = std::get<1>(sample_result);
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
            "sample_data" + device_name, 
            json_writer_, 
            "sample_data");
            
            devices_frame_count_[device_name]++;

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving image data: %s", e.what());
        }
    }

    void save_bbox3d_data(const std::shared_ptr<vision_msgs::msg::Detection3DArray> &msg, const std::string &device_name) {
        std::string data_type = (sample_triggers_[device_name] == true) ? "samples" : "sweeps";
        if(data_type == "samples"){
            // 用于保存所有结果的JSON对象
            std::vector<nlohmann::json> sample_annotation_jsons;
            // 生成 sample_token
            std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            // 遍历 Detection3DArray 中的每个检测结果
            for (const auto &detection : msg->detections) {
                nlohmann::json sample_annotation_json;
                // 获取 id 和 class_id
                int id = std::stoi(detection.results[0].hypothesis.class_id);
                std::string class_id = detection.results[0].hypothesis.class_id;
                // 生成 instance_token
                std::string instance_token = generate_unique_token(id, class_id, start_system_time_token_);
                // 生成 sample_annotation_token
                std::string sample_annotation_token_tmp = "3dbox_" + device_name + instance_token;
                std::string sample_annotation_token = generate_unique_token(devices_frame_count_[device_name], sample_annotation_token_tmp, start_system_time_token_);
                // 生成 visibility_token
                double score = detection.results[0].hypothesis.score;
                std::string visibility_token = generate_visibility_token(score);
                // 填充检测结果 JSON
                sample_annotation_json["token"] = sample_annotation_token;
                sample_annotation_json["sample_token"] = sample_token;
                sample_annotation_json["instance_token"] = instance_token;
                sample_annotation_json["visibility_token"] = visibility_token;
                sample_annotation_json["attribute_tokens"] = nlohmann::json::array();  // 如果有属性 tokens，可以填充这个字段
                // translation：从 bbox 中获取位置
                const auto &bbox = detection.bbox;
                sample_annotation_json["translation"] = {bbox.center.position.x, bbox.center.position.y, bbox.center.position.z};
                // rotation：从 bbox 中获取方向（四元数）
                sample_annotation_json["rotation"] = {bbox.center.orientation.x, bbox.center.orientation.y, bbox.center.orientation.z, bbox.center.orientation.w};
                // size：从 bbox 数据获取
                sample_annotation_json["size"] = {bbox.size.x, bbox.size.y, bbox.size.z};
                // 填充 prev 和 next
                sample_annotation_json["prev"] = "";
                sample_annotation_json["next"] = "";  // this will resolve in slider
                // 填充 num_lidar_pts 和 num_radar_pts
                sample_annotation_json["num_lidar_pts"] = -1;  // 示例数据
                sample_annotation_json["num_radar_pts"] = -1;  // 示例数据
                sample_annotation_jsons.emplace_back(sample_annotation_json);

                // ******************generate instance.json********************
                auto& val = instance_recorders[instance_token];
                json instance_json; 
                if (val.empty()) {
                    // 如果 vector 为空，初始化一个新的 JSON 对象
                    instance_json["token"] = instance_token;
                    instance_json["category_token"] = "1fa93b757fc74fb197cdd60001ad8abf";  // TODO:waitting for category_token
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
            }
            // resolve next for each instance in slider map
            for(auto& json : sample_annotation_jsons){
                // auto& annotation_slider = prev_next_sliders_[c["instance_token"]];  // update the instance class in prev_next_sliders_
                // if(annotation_slider.size() < sample_annotation_cache_size){
                //     annotation_slider.push_back(json);
                //     if(annotation_slider.size() == 2){  // solve it when slider is full
                //         annotation_slider[0]["next"] = annotation_slider[1]["token"];
                //         annotation_slider[1]["prev"] = annotation_slider[0]["token"];
                //         json_writer_.addKeyValuePairToJson("sample_annotation", annotation_slider[0], true);
                //         // pop front and slide
                //         annotation_slider[0] = annotation_slider[1];  
                //         annotation_slider.pop_back();
                //     }
                // }
                std::string instance_key = json["instance_token"];
                generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
                json, instance_key, 
                json_writer_, 
                "sample_annotation");
            }
        }
    }

    void save_lidar_data(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const std::string &device_name) {
        try {
            // Determine whether to save to 'sweeps' or 'samples'
            std::string data_type = (sample_triggers_[device_name] == true) ? "samples" : "sweeps";
            // Create the directory path for saving point cloud
            std::string folder_path = root_path_ + "/" + data_type + "/" + device_name;
            ensure_directory_exists(folder_path);
            // Generate a file path with timestamp for the point cloud
            std::stringstream ss;
            ss << folder_path << '/' << start_system_time_token_ << "__" << device_name << "__" 
            << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".pcl.bin";
            std::string cloud_save_file = ss.str();

            // Convert PointCloud2 to pcl::PointCloud and save to binary file
            pcl::PointCloud<pcl::PointXYZ> cloud;

            // Step 1: Convert sensor_msgs::msg::PointCloud2 to pcl::PCLPointCloud2
            pcl::PCLPointCloud2 pcl_cloud2;
            pcl_conversions::toPCL(*msg, pcl_cloud2);  // 使用 pcl_conversions::toPCL

            // Step 2: Convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
            pcl::fromPCLPointCloud2(pcl_cloud2, cloud);  // 使用 pcl::fromPCLPointCloud2

            // Save point cloud to binary file
            std::ofstream ofs(cloud_save_file, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(cloud.points.data()), cloud.points.size() * sizeof(pcl::PointXYZ));
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "PointCloud2 saved as BIN: %s", cloud_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving point cloud: %s", cloud_save_file.c_str());
            }

            // Prepare the JSON data for sample_data
            nlohmann::json sample_data_json;

            // cal the previous token and next token for the current device
            // std::tuple<std::string, std::string> sample_result = generate_sample_prev_and_next(sample_frame_);
            // std::string prev_token = std::get<0>(sample_result);
            // std::string next_token = std::get<1>(sample_result);

            uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
            bool is_key_frame = (data_type == "samples");  // Example of setting key frame flag
            // 生成 sample_token and write sample_json
            std::string sample_token = generate_sample_token(sample_frame_, start_system_time_token_);
            if(is_key_frame){
                json sample_json;

                sample_json["token"] = sample_token;
                sample_json["timestamp"] = timestamp;
                sample_json["prev"] = "";
                sample_json["next"] = "";
                sample_json["scene_token"] = cur_scene_token_;
                // generate sample's prev and next
                generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
                sample_json, 
                "sample" + device_name, 
                json_writer_, 
                "sample");
                sample_done_sign_[device_name] = true;
            }

            std::string sample_data_token = generate_unique_token(devices_frame_count_[device_name], device_name, start_system_time_token_);
            latest_ego_pose["token"] = sample_data_token;  // set as sample_data_token
            json_writer_.addKeyValuePairToJson("ego_pose", latest_ego_pose, true);

            std::string calibrated_sensor_token = get_calibration_token(calibrated_sensor_token_map_, device_name);
            if(calibrated_sensor_token == "")
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());

            sample_data_json["token"] = sample_data_token;
            sample_data_json["sample_token"] = sample_token;
            sample_data_json["ego_pose_token"] = sample_data_token;
            sample_data_json["calibrated_sensor_token"] = calibrated_sensor_token;
            sample_data_json["timestamp"] = timestamp;
            sample_data_json["fileformat"] = "pcd";  // File format is BIN
            sample_data_json["is_key_frame"] = is_key_frame;
            sample_data_json["height"] = "";
            sample_data_json["width"] = "";
            sample_data_json["filename"] = extractRelativePath(cloud_save_file);
            sample_data_json["prev"] = "";
            sample_data_json["next"] = "";

            // generate sample_data's prev and next
            generate_prev_next_and_write_json_dynamically(prev_next_sliders_, 
            sample_data_json, 
            "sample_data" + device_name, 
            json_writer_, 
            "sample_data");

            devices_frame_count_[device_name]++;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving point cloud data: %s", e.what());
        }
    }

    void scene_processing(std::string device_name, std::string sample_token){
        if(device_name == scene_recoder_name_){
            // update count
            scene_sample_frame_count_++;
            if(scene_sample_frame_count_ == 1){  // the first sample to generate a scene_token
                // update_scene_token and reset scene_frame__count
                std::string cur_system_time_string = get_cur_system_time();
                cur_scene_token_ = "scene_" + scene_name_ + "_" + cur_system_time_string; 
                //std::string log_token = cur_scene_token_ + "_log";
                std::string log_token = "0986cb758b1d43fdaa051ab23d45582b";  // TODO: you can generate it when you wanna create a map
                std::string description = "";
                latest_scene_["token"] = cur_scene_token_;
                latest_scene_["log_token"] = log_token;
                latest_scene_["nbr_samples"] = scene_sample_frame_count_;
                latest_scene_["first_sample_token"] = sample_token;
                latest_scene_["last_sample_token"] = sample_token;
                latest_scene_["name"] = scene_name_;
                latest_scene_["description"] = description;

                latest_log_["token"] = log_token;
                latest_log_["logfile"] = "None";
                latest_log_["vehicle"] = robot_model_;
                latest_log_["date_captured"] = start_system_time_token_.substr(0, 10);  // 从字符串中提取前10个字符，即 YYYY-MM-DD
                latest_log_["location"] = location_;
            }
            else{
                latest_scene_["last_sample_token"] = sample_token;
                latest_scene_["nbr_samples"] = scene_sample_frame_count_;
            }
        }
    }

    void scene_done(std::string device_name, std::string sample_token){
        if(device_name == scene_recoder_name_){
            // update count
            scene_sample_frame_count_++;
            // the scene is full, so write it
            latest_scene_["last_sample_token"] = sample_token;
            latest_scene_["nbr_samples"] = scene_sample_frame_count_;
            json_writer_.addKeyValuePairToJson("scene", latest_scene_, true); 
            latest_scene_.clear(); // clear cache
            json_writer_.addKeyValuePairToJson("log", latest_log_, true); 
            latest_log_.clear(); // clear cache
            scene_sample_frame_count_ = 0;
        }
    }

    void scene_cache_write(){
        json_writer_.addKeyValuePairToJson("scene", latest_scene_, true); 
        latest_scene_.clear(); // clear cache
        json_writer_.addKeyValuePairToJson("log", latest_log_, true); 
        latest_log_.clear(); // clear cache
        scene_sample_frame_count_ = 0;
    }

    void check_sample_done(){
        // get sample done and clear sign
        for(auto& sign_pair : sample_done_sign_){
            if(sign_pair.second == false){
                return;
            }
        }
        for(auto& sign_pair : sample_done_sign_){
            sign_pair.second = false;
        }
        sample_frame_++;
    } 

    void node_shutdown_callback(){
        // write last json in slider which have prev and next
        for (auto& slider : prev_next_sliders_) {
            if (slider.second.size() == 1) {
                // 将唯一的 sample_annotation 写入 JSON 文件
                json_writer_.addKeyValuePairToJson("sample_annotation", slider.second[0], true);
            }
        }
        // write all unique instance in one scene
        for(auto& key_val : instance_recorders){
            json& instance_json = key_val.second.front();
            json_writer_.addKeyValuePairToJson("instance", instance_json, true);
        }
        // write the scene which is not full
        scene_cache_write();
    }

    std::string extractRelativePath(const std::string& data_path) {
        // 查找 "sweeps" 或 "samples" 的位置
        size_t pos = data_path.find("sweeps");
        if (pos == std::string::npos) {
            pos = data_path.find("samples");
        }
        // 如果找到 "sweeps" 或 "samples"，返回相对路径
        return (pos != std::string::npos) ? data_path.substr(pos) : "";
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
    
    //const std::string location_ = "china-beijing"; // specify the country we are in now 
    const std::string location_ =  "singapore-hollandvillage";
    const std::string robot_model_ = "diff_bot"; // specify the robot used for capturing data
    const std::string scene_name_ = "office";
    const int issac_per_sec_frame = 60;
    const float sample_frequency = 0.5; // sample every 0.5s
    int sample_interval_;
    double latest_time_;
    rclcpp::Time last_sample_time_;
    unsigned long long sample_frame_;
    std::unordered_map<std::string, bool> sample_triggers_;  // make devices to sample
    std::unordered_map<std::string, bool> sample_done_sign_;  // indicate sample done
    std::unordered_map<std::string, unsigned long long> devices_frame_count_;  // each device a frame count
    const unsigned long long target_sample_frame_ = 30;
    const int scene_frame_ = 2; // the frame each scene should have
    const std::string scene_recoder_name_ = "CAM_FRONT";
    std::string cur_scene_token_;
    int scene_sample_frame_count_;
    json latest_scene_;  // the newest scene we record
    json latest_log_; // the newest log we record
    std::vector<std::string> camera_names_;
    const double max_time_diff_ = 0.01; // cam和bbox3d最大容忍timestamp误差
    std::vector<std::string> sample_sensors_;
    const std::unordered_map<std::string, std::string> sensor_token_map_ = {  // Initialize the unordered_map with device name (channel) -> token mapping
        {"CAM_FRONT", "SENSOR_CAM_FRONT"},
        {"CAM_BACK", "SENSOR_CAM_BACK"},
        {"CAM_BACK_LEFT", "SENSOR_CAM_BACK_LEFT"},
        {"CAM_FRONT_LEFT", "SENSOR_CAM_FRONT_LEFT"},
        {"CAM_FRONT_RIGHT", "SENSOR_CAM_FRONT_RIGHT"},
        {"CAM_BACK_RIGHT", "SENSOR_CAM_BACK_RIGHT"},
        {"LIDAR_TOP", "SENSOR_LIDAR_TOP"}
    };
    const std::unordered_map<std::string, std::string> calibrated_sensor_token_map_ = {
        {"CAM_FRONT", "CALIBRATION_CAM_FRONT"},
        {"CAM_BACK", "CALIBRATION_CAM_BACK"},
        {"CAM_BACK_LEFT", "CALIBRATION_CAM_BACK_LEFT"},
        {"CAM_FRONT_LEFT", "CALIBRATION_CAM_FRONT_LEFT"},
        {"CAM_FRONT_RIGHT", "CALIBRATION_CAM_FRONT_RIGHT"},
        {"CAM_BACK_RIGHT", "CALIBRATION_CAM_BACK_RIGHT"},
        {"LIDAR_TOP", "CALIBRATION_LIDAR_TOP"}
    };
    json latest_ego_pose;
    std::string start_system_time_token_;
    std::unordered_map<std::string, std::vector<json>> prev_next_sliders_;  // this is used for all sample_annotation's pre and next,string record each instance
    std::unordered_map<std::string, std::vector<json>> instance_recorders;  // this is used for all instance's last annotation, each instance have an unique token in one scene
    const size_t sample_annotation_cache_size = 2;

    NuscenesJsonWriter json_writer_;

    std::unordered_map<std::string, std::string> camera_topics_;
    std::unordered_map<std::string, std::string> bbox3d_topics_;
    std::string odom_topic_;
    std::string lidar_topic_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr> bbox3d_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    // Store synchronizers to keep them alive
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> image_subs_;
    std::vector<std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection3DArray>>> bbox3d_subs_;
    std::vector<std::shared_ptr<void>> synchronizers_;

};

#endif  // NUSCENES_DATA_COLLECTOR_ISSAC_HPP


