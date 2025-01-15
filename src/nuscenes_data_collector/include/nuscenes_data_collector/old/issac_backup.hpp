#ifndef NUSCENES_DATA_COLLECTOR_ISSAC_HPP
#define NUSCENES_DATA_COLLECTOR_ISSAC_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "sensor_msgs/image_encodings.hpp" // For image encoding constants
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex> // For thread safety

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#include "nuscenes_data_collector/utils.hpp"
#include "nuscenes_data_collector/nuscenes_json_writer.hpp"

using json = nlohmann::json;

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
        initialize_camera_subscriptions();

        rclcpp::QoS qos_profile(100);
        qos_profile.reliable();  // 设置可靠传输
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
        add_sample_sensors("odom");
        add_sample_sensors("LIDAR_TOP");
        for(auto device : sample_sensors_){
            sensor_done_signs_[device] = false;
        }
        all_task_done_ = false;
        start_system_time_token_ = get_cur_system_time();
        
        sample_interval_ = static_cast<int>(issac_per_sec_frame * sample_frequency);
        sample_frame = 0;
        scene_sample_frame_count_ = 0;
        std::string cur_system_time_string = get_cur_system_time();
        cur_scene_token_ = "scene_" + scene_name_ + "_" + cur_system_time_string;  // init the first scene_token
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
    void initialize_camera_subscriptions() {
        for (const auto &camera : camera_names_) {
            rclcpp::QoS qos_profile(100);
            qos_profile.reliable();  // 设置可靠传输
            // Camera image subscription
            camera_subscriptions_[camera] = this->create_subscription<sensor_msgs::msg::Image>(
                camera_topics_[camera], qos_profile, [this, camera](const sensor_msgs::msg::Image::SharedPtr msg) {
                    std::async(std::launch::async, [this, msg, camera]() {
                        this->camera_callback(msg, camera);
                    });
                }
            );

            // Camera bbox3d subscription
            bbox3d_subscriptions_[camera] = this->create_subscription<vision_msgs::msg::Detection3DArray>(
                bbox3d_topics_[camera], qos_profile, [this, camera](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                    std::async(std::launch::async, [this, msg, camera]() {
                        this->bbox3d_callback(msg, camera);
                    });
                }
            );
        }
    }

    // Camera image callback
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string &camera_name) {
        RCLCPP_INFO(this->get_logger(), "Received image from %s", camera_name.c_str());
        save_data(msg, camera_name);
    }

    // BBox3D data callback
    void bbox3d_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg, const std::string &camera_name) {
        RCLCPP_INFO(this->get_logger(), "Received 3D bbox data from %s", camera_name.c_str());
        save_data(msg, camera_name);
    }

    // Odometry data callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        save_data(msg, "odom");
    }

    // lidar data callback
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received lidar_top data");
        save_data(msg, "LIDAR_TOP");
    }

    /// Template function to save data to sweeps directory
    template <typename T>
    void save_data(const T &msg, const std::string &device_name) {
        if(cur_frame_count_[device_name] < target_frame_count){
            // RCLCPP_INFO(this->get_logger(), "Template type T: %s", typeid(T).name());
            // RCLCPP_INFO(this->get_logger(), "Message type: %s", typeid(msg).name());
            // Handle Odometry messages
            if constexpr (std::is_same_v<T, std::shared_ptr<nav_msgs::msg::Odometry>>) {
                save_odom_data(msg, device_name);
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
                RCLCPP_INFO(this->get_logger(), "This is LIDAR data, proceeding with save_lidar_data.");
                save_lidar_data(msg, device_name);
            }
        }
        else{
            // set this device done, check if all device had done
            sensor_done_signs_[device_name] = true;
            all_task_done_ = true;
            for(auto device : sample_sensors_){
                if(sensor_done_signs_[device] == false){
                    //RCLCPP_INFO(this->get_logger(), "*******************%s", device.c_str());
                    all_task_done_ = false;
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Reached target frame count for %s. Skipping further collection.", device_name.c_str());
        }
        // check whether all task had done, then shtdown this node
        if (all_task_done_) {
            RCLCPP_INFO(this->get_logger(), "All devices have reached target frame count. Shutting down the node.");
            //RCLCPP_INFO(this->get_logger(), "*********************************************************************");
            // Iterate through all sample sensors and print their frame count and done status
            for (auto device : sample_sensors_) {
                RCLCPP_INFO(this->get_logger(), "Device: %s, Frame Count: %llu, Done Status: %s", 
                            device.c_str(), cur_frame_count_[device], 
                            sensor_done_signs_[device] ? "true" : "false");
            }
            all_task_done_callback();
            rclcpp::shutdown();  // Shut down the node if all devices are done
            return;
        }
    }

    // write odom data in nuscenes ego_pose format and save to ego_pose.json
    void save_odom_data(const std::shared_ptr<nav_msgs::msg::Odometry> msg, const std::string &device_name) {
        json ego_pose_data;
        // formula: ego_token = harsh(frame, "ego_pose_" + "odom", start_system_time_token_) 
        ego_pose_data["token"] = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
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
        json_writer_.addKeyValuePairToJson("ego_pose", ego_pose_data, true);
        cur_frame_count_[device_name]++;
    }


    // Save Image data as JPEG and write it to sampe_data.json in the nuscenes sample_data format
    void save_image_data(const std::shared_ptr<sensor_msgs::msg::Image> &msg, 
                              const std::string &device_name) {
        try {
            // Determine whether to save to 'sweeps' or 'samples'
            std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
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
            nlohmann::json sample_data;
            // formula: sample_data_token = harsh(frame, "samples_data_" + device_name, start_system_time_token_)
            std::string sample_data_token_tmp = "samples_data_" + device_name;
            std::string token = generate_unique_token(cur_frame_count_[device_name], device_name, start_system_time_token_);
            // formula: ego_token = harsh(frame, "ego_pose_" + "odom", start_system_time_token_)
            std::string ego_pose_token = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
            // get time_stamp and key_frame sign
            uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
            bool is_key_frame = data_type == "samples" ? true : false;
            // ********************generate sample_token, and write to sample.json and scene.json********************
            int sample_frame = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame, start_system_time_token_);
            if(is_key_frame){
                if(scene_sample_frame_count_ < scene_frame)
                {
                    json sample_json;
                    // cal the previous token and next token for the current device
                    std::tuple<std::string, std::string> sample_result = generate_sample_prev_and_next(sample_frame);
                    std::string prev_token = std::get<0>(sample_result);
                    std::string next_token = std::get<1>(sample_result);

                    sample_json["token"] = sample_token;
                    sample_json["timestamp"] = timestamp;
                    sample_json["prev"] = sample_prev_token;
                    sample_json["next"] = sample_next_token;
                    sample_json["scene_token"] = cur_scene_token_;
                    json_writer_.addKeyValuePairToJson("sample", sample_json, true);   
                    // save scene sample frame in and out;
                    if(scene_sample_frame_count_ == 0) scene_first_sample_token_ = sample_token;
                    else if(scene_sample_frame_count_ == scene_frame - 1) scene_last_sample_token_ = sample_token;
                    // update count
                    scene_sample_frame_count_++;
                }
                else{
                    // save comlete scene json and write log json, then save next scenes' sample json
                    json scene_json, log_json;
                    std::string description = "";
                    //std::string log_token = cur_scene_token_ + "_log";
                    std::string log_token = "0986cb758b1d43fdaa051ab23d45582b";  // TODO: you can generate it when you wanna create a map
                    scene_json["token"] = cur_scene_token_;
                    scene_json["log_token"] = log_token;
                    scene_json["nbr_samples"] = scene_sample_frame_count_;
                    scene_json["first_sample_token"] = scene_first_sample_token_;
                    scene_json["last_sample_token"] = scene_last_sample_token_;
                    scene_json["name"] = scene_name_;
                    scene_json["description"] = description;
                    json_writer_.addKeyValuePairToJson("scene", scene_json, true); 
                    log_json["token"] = log_token;
                    log_json["logfile"] = "None";
                    log_json["vehicle"] = robot_model_;
                    log_json["date_captured"] = start_system_time_token_.substr(0, 10);  // 从字符串中提取前10个字符，即 YYYY-MM-DD
                    log_json["location"] = location_;
                    json_writer_.addKeyValuePairToJson("log", log_json, true); 
                    // update_scene_token and reset scene_frame_count
                    std::string cur_system_time_string = get_cur_system_time();
                    cur_scene_token_ = "scene_" + scene_name_ + "_" + cur_system_time_string; 
                    scene_sample_frame_count_ = 0;
                    // generate new scene's first sample
                    json sample_json;
                    // generate sample's next and prev token
                    std::string sample_prev_token, sample_next_token;
                    if(sample_frame == 0)
                        sample_prev_token = generate_unique_token(sample_frame, device_name, start_system_time_token_);
                    else sample_prev_token = "";
                    if((cur_frame_count_[device_name] + sample_interval_) < target_frame_count)
                        sample_next_token = generate_unique_token(sample_frame, device_name, start_system_time_token_);
                    else sample_next_token = "";

                    sample_json["token"] = sample_token;
                    sample_json["timestamp"] = timestamp;
                    sample_json["prev"] = sample_prev_token;
                    sample_json["next"] = sample_next_token;
                    sample_json["scene_token"] = cur_scene_token_;
                    json_writer_.addKeyValuePairToJson("sample", sample_json, true);   
                    // save scene sample frame in and out;
                    if(scene_sample_frame_count_ == 0) scene_first_sample_token_ = sample_token;
                    else if(scene_sample_frame_count_ == scene_frame - 1) scene_last_sample_token_ = sample_token;
                    // update count
                    scene_sample_frame_count_++;
                }
            }
            // Extract the path from the image path, keeping only the part after 'sweeps' or 'samples'
            std::string relative_path = extractRelativePath(image_save_file);
            // get calibrated sensor token 
            std::string calibrated_sensor_token;
            auto calibrated_sensor_it = calibrated_sensor_token_map_.find(device_name);
            if (calibrated_sensor_it != calibrated_sensor_token_map_.end()) {
                calibrated_sensor_token = calibrated_sensor_it->second;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
                return;
            }
            // cal the previous token and next token for the current device
            std::tuple<std::string, std::string> sample_result = generate_sample_prev_and_next(sample_frame);
            std::string prev_token = std::get<0>(sample_result);
            std::string next_token = std::get<1>(sample_result);
            // Prepare the JSON fields
            sample_data["token"] = token;
            sample_data["sample_token"] = sample_token;  // 指向sample_data所关联的sample
            sample_data["ego_pose_token"] = ego_pose_token;
            sample_data["calibrated_sensor_token"] = calibrated_sensor_token;  
            sample_data["timestamp"] = timestamp;  // Full timestamp
            sample_data["fileformat"] = "jpg";  // File format is JPEG
            sample_data["is_key_frame"] = is_key_frame;  // Example, this could be dynamic based on your application
            sample_data["height"] = img.rows;
            sample_data["width"] = img.cols;
            sample_data["filename"] = relative_path;
            sample_data["prev"] = prev_token;  // Example, replace with actual previous token if needed
            sample_data["next"] = next_token;  // Example, replace with actual next token if needed
            // 使用 json_writer_ 将数据追加到文件
            json_writer_.addKeyValuePairToJson("sample_data", sample_data, true);
            
            cur_frame_count_[device_name]++;

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving image data: %s", e.what());
        }
    }

    void save_bbox3d_data(const std::shared_ptr<vision_msgs::msg::Detection3DArray> &msg, const std::string &device_name) {
        // Determine whether to save to 'sweeps' or 'samples'
        std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
    
        if (data_type == "samples") {
            // 用于保存所有结果的JSON对象
            std::vector<nlohmann::json> sample_annotation_jsons;
            // 生成 sample_token
            int sample_frame = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame, start_system_time_token_);
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
                std::string sample_annotation_token = generate_unique_token(cur_frame_count_[device_name], sample_annotation_token_tmp, start_system_time_token_);
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
                auto& annotation_slider = sample_annotation_sliders[json["instance_token"]];  // update the instance class in sample_annotation_sliders
                if(annotation_slider.size() < sample_annotation_cache_size){
                    annotation_slider.push_back(json);
                    if(annotation_slider.size() == 2){  // solve it when slider is full
                        annotation_slider[0]["next"] = annotation_slider[1]["token"];
                        annotation_slider[1]["prev"] = annotation_slider[0]["token"];
                        json_writer_.addKeyValuePairToJson("sample_annotation", annotation_slider[0], true);
                        // pop front and slide
                        annotation_slider[0] = annotation_slider[1];  
                        annotation_slider.pop_back();
                    }
                }
            }
        }
    }

    void save_lidar_data(const std::shared_ptr<sensor_msgs::msg::PointCloud2> &msg, const std::string &device_name) {
        try {
            // Determine whether to save to 'sweeps' or 'samples'
            std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
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
            nlohmann::json sample_data;

            uint64_t timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

            bool is_key_frame = (data_type == "samples");  // Example of setting key frame flag
            // 生成 sample_token
            int sample_frame = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame, start_system_time_token_);
            if(is_key_frame){
                json sample_json;
                // cal the previous token and next token for the current device
                std::tuple<std::string, std::string> sample_result = generate_sample_prev_and_next(sample_frame);
                std::string prev_token = std::get<0>(sample_result);
                std::string next_token = std::get<1>(sample_result);

                sample_json["token"] = sample_token;
                sample_json["timestamp"] = timestamp;
                sample_json["prev"] = sample_prev_token;
                sample_json["next"] = sample_next_token;
                sample_json["scene_token"] = cur_scene_token_;
                json_writer_.addKeyValuePairToJson("sample", sample_json, true);  
            }

            std::string ego_pose_token = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
            std::string calibrated_sensor_token;
            auto calibrated_sensor_it = calibrated_sensor_token_map_.find(device_name);
            if (calibrated_sensor_it != calibrated_sensor_token_map_.end()) {
                calibrated_sensor_token = calibrated_sensor_it->second;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
                return;
            }

            std::string prev_token, next_token;
            if (cur_frame_count_[device_name] != 0)
                prev_token = generate_unique_token(cur_frame_count_[device_name]-1, device_name, start_system_time_token_);
            else prev_token = "";
            if (cur_frame_count_[device_name] < target_frame_count)
                next_token = generate_unique_token(cur_frame_count_[device_name]+1, device_name, start_system_time_token_);
            else next_token = "";

            sample_data["token"] = sample_data_token;
            sample_data["sample_token"] = sample_token;
            sample_data["ego_pose_token"] = ego_pose_token;
            sample_data["calibrated_sensor_token"] = calibrated_sensor_token;
            sample_data["timestamp"] = timestamp;
            sample_data["fileformat"] = "pcd";  // File format is BIN
            sample_data["is_key_frame"] = is_key_frame;
            sample_data["height"] = "";
            sample_data["width"] = "";
            sample_data["filename"] = extractRelativePath(cloud_save_file);
            sample_data["prev"] = prev_token;
            sample_data["next"] = next_token;

            json_writer_.addKeyValuePairToJson("sample_data", sample_data, true);

            cur_frame_count_[device_name]++;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error saving point cloud data: %s", e.what());
        }
    }

    void all_task_done_callback(){
        // write annotation withous next
        for (auto& slider : sample_annotation_sliders) {
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

    // 根据可见性百分比生成 visibility_token
    std::string generate_visibility_token(double visibility_percentage) {
        if (visibility_percentage >= 0 && visibility_percentage < 0.4) {
            return "1";  // v0-40
        } else if (visibility_percentage >= 0.4 && visibility_percentage < 0.6) {
            return "2";  // v40-60
        } else if (visibility_percentage >= 0.6 && visibility_percentage < 0.8) {
            return "3";  // v60-80
        } else if (visibility_percentage >= 0.8 && visibility_percentage <= 1) {
            return "4";  // v80-100
        } else {
            return "1";  // 默认值，如果可见性超出范围
        }
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
    std::unordered_map<std::string, unsigned long long> cur_frame_count_;  // each device a frame count
    unsigned long long global_frame;
    const unsigned long long target_frame_count = 30 * 4;
    const int scene_frame = 2; // the frame each scene should have
    int scene_sample_frame_count_;
    std::string cur_scene_token_;
    std::string scene_first_sample_token_;
    std::string scene_last_sample_token_;
    std::vector<std::string> camera_names_;
    std::vector<std::string> sample_sensors_;
    std::unordered_map<std::string, bool> sensor_done_signs_; // string:device(sample_sensors_)  bool: is_done
    bool all_task_done_; // sign show whether all sensor task done (capture data)
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
    const std::string ego_pose_token_const = "ego_pose_odom";
    std::string start_system_time_token_;
    std::unordered_map<std::string, std::vector<json>> sample_annotation_sliders;  // this is used for all sample_annotation's pre and next,string record each instance
    std::unordered_map<std::string, std::vector<json>> instance_recorders;  // this is used for all instance's last annotation, each instance have an unique token in one scene
    const int sample_annotation_cache_size = 2;

    NuscenesJsonWriter json_writer_;

    std::unordered_map<std::string, std::string> camera_topics_;
    std::unordered_map<std::string, std::string> bbox3d_topics_;
    std::string odom_topic_;
    std::string lidar_topic_;

    
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subscriptions_;
    std::unordered_map<std::string, rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr> bbox3d_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;

    void generate_sample_token(int sample_frame, const std::optional<std::string>& start_time){
        return generate_unique_token(sample_frame, "sample", start_time);
    }
    std::tuple<std::string, std::string> generate_sample_prev_and_next(int sample_frame) {
        // generate sample's next and prev token
        std::string sample_prev_token, sample_next_token;
        if(sample_frame > 1)
            sample_prev_token = generate_unique_token(sample_frame - 1, "sample", start_system_time_token_);
        else sample_prev_token = "";
        if((global_frame + sample_interval_) < target_frame_count)
            sample_next_token = generate_unique_token(sample_frame + 1, "sample", start_system_time_token_);
        else sample_next_token = "";
        return std::make_tuple(sample_prev_token, sample_next_token);
    }

};

#endif  // NUSCENES_DATA_COLLECTOR_ISSAC_HPP


