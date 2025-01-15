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
#include <iomanip> // For std::put_time
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex> // For thread safety

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "nuscenes_data_collector/utils.hpp"
#include "nuscenes_data_collector/nuscenes_json_writer.hpp"

using json = nlohmann::json;

// Define synchronization policies
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    vision_msgs::msg::Detection3DArray,
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::PointCloud2
> MySyncPolicy;

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
      json_writer_(json_paths_),
      // Initialize message_filters subscribers
      image_sub_(this, "/issac/CAM_FRONT", rmw_qos_profile_sensor_data),
      bbox3d_sub_(this, "/issac/CAM_FRONT/bbox3d", rmw_qos_profile_sensor_data),
      odom_sub_(this, "/issac/odom", rmw_qos_profile_sensor_data),
      lidar_sub_(this, "/issac/point_cloud", rmw_qos_profile_sensor_data),
      // Define Synchronizer with the defined policy
      sync_(MySyncPolicy(10), image_sub_, bbox3d_sub_, odom_sub_, lidar_sub_)
    {
        // 检查 JSON 加载是否有错误
        if (json_writer_.isLoadError()) {
            RCLCPP_ERROR(this->get_logger(), "Json Load Error. Shutting down the node.");
            rclcpp::shutdown();  // 停止 ROS 节点
            return;  // 退出函数，确保不继续执行后续代码
        }

        // 初始化其他参数和传感器集
        declare_parameters();
        initialize_sample_sensors();

        // 注册同步回调
        sync_.registerCallback(std::bind(&NuScenesDataCollector::synced_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2, 
                                        std::placeholders::_3, std::placeholders::_4));

        // 初始化其他变量
        all_task_done_ = false;
        start_system_time_token_ = get_cur_system_time();

        sample_interval_ = static_cast<int>(issac_per_sec_frame * sample_frequency);
        sample_frame = 0; // 初始化 sample_frame
        scene_sample_frame_count_ = 0;
        std::string cur_system_time_string = get_cur_system_time();
        cur_scene_token_ = "scene_" + scene_name_ + "_" + cur_system_time_string;  // init the first scene_token
    }

private:
    // Message Filters Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> bbox3d_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;

    // Synchronizer
    message_filters::Synchronizer<MySyncPolicy> sync_;

    // 其他成员变量
    std::string root_path_;
    std::vector<std::string> json_paths_;
    
    const std::string location_ =  "singapore-hollandvillage";
    const std::string robot_model_ = "diff_bot"; // specify the robot used for capturing data
    const std::string scene_name_ = "office";
    const int issac_per_sec_frame = 60;
    const float sample_frequency = 0.5; // sample every 0.5s
    int sample_interval_;
    int sample_frame; // 声明 sample_frame
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

    // Mutex for thread safety
    std::mutex data_mutex_;

    // Add the missing member variable
    std::unordered_map<std::string, unsigned long long> cur_frame_count_;

    // Utility functions
    void add_sample_sensors(const std::string& device_name){
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

    void initialize_sample_sensors(){
        // 添加传感器
        for(auto c: camera_names_) add_sample_sensors(c);
        add_sample_sensors("odom");
        add_sample_sensors("LIDAR_TOP");
        for(auto device : sample_sensors_){
            cur_frame_count_[device] = 0;
            sensor_done_signs_[device] = false;
        }
    }

    // 同步回调函数
    void synced_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const vision_msgs::msg::Detection3DArray::ConstSharedPtr& bbox3d_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg
    ) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_INFO(this->get_logger(), "Received synchronized messages");

        // 更新帧计数
        update_frame_count("CAM_FRONT");
        update_frame_count("odom");
        update_frame_count("LIDAR_TOP");

        // 处理和保存数据
        save_data(image_msg, "CAM_FRONT");
        save_data(bbox3d_msg, "CAM_FRONT");
        save_data(odom_msg, "odom");
        save_data(lidar_msg, "LIDAR_TOP");

        // 检查是否达到目标帧数并决定是否关闭节点
        check_and_shutdown();
    }

    // 更新帧计数
    void update_frame_count(const std::string &device_name){
        if(cur_frame_count_.find(device_name) != cur_frame_count_.end()){
            cur_frame_count_[device_name]++;
        }
    }

    /// 重载函数保存数据
    void save_data(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &device_name) {
        RCLCPP_INFO(this->get_logger(), "Received image from %s", device_name.c_str());
        save_image_data(msg, device_name);
    }

    void save_data(const vision_msgs::msg::Detection3DArray::ConstSharedPtr &msg, const std::string &device_name) {
        RCLCPP_INFO(this->get_logger(), "Received 3D bbox data from %s", device_name.c_str());
        save_bbox3d_data(msg, device_name);
    }

    void save_data(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const std::string &device_name) {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        save_odom_data(msg, device_name);
    }

    void save_data(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &device_name) {
        RCLCPP_INFO(this->get_logger(), "Received lidar_top data");
        save_lidar_data(msg, device_name);
    }

    /// 写入里程计数据到 ego_pose.json
    void save_odom_data(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const std::string &device_name) {
        json ego_pose_data;
        // 生成 token
        ego_pose_data["token"] = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
        // 生成 timestamp
        uint64_t timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;
        ego_pose_data["timestamp"] = timestamp;
        // 填充 rotation 和 translation
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
        // 将数据写入 JSON
        json_writer_.addKeyValuePairToJson("ego_pose", ego_pose_data, true);
    }

    /// 保存图像数据并写入 sample_data.json
    void save_image_data(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &device_name) {
        try {
            // 确定保存类型为 'sweeps' 或 'samples'
            std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
            // 创建保存路径
            std::string folder_path = root_path_ + "/" + data_type + "/" + device_name;
            ensure_directory_exists(folder_path);
            // 生成保存文件路径
            std::stringstream ss;
            ss << folder_path << '/' << start_system_time_token_ << "__" << device_name << "__" 
               << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".jpg";
            std::string image_save_file = ss.str();

            // 转换编码
            std::string encoding = msg->encoding;
            std::transform(encoding.begin(), encoding.end(), encoding.begin(), [](unsigned char c) { return std::tolower(c); });
            cv::Mat img;
            if (encoding == sensor_msgs::image_encodings::BGR8 ||
                encoding == sensor_msgs::image_encodings::RGB8) {
                // 直接使用图像数据
                img = cv_bridge::toCvShare(msg, encoding)->image;
            }
            else if (encoding == "jpeg" || encoding == "jpg") {
                // 解码 JPEG 图像
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
            // 编码为 JPEG
            std::vector<uchar> encoded_img;
            bool success = cv::imencode(".jpg", img, encoded_img);
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to encode image to JPEG");
                return;
            }
            // 保存图像文件
            std::ofstream ofs(image_save_file, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(encoded_img.data()), encoded_img.size());
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "Image saved as JPEG: %s", image_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open image file for writing: %s", image_save_file.c_str());
                return;
            }

            // 准备 JSON 数据
            json sample_data;
            // 生成 sample_data token
            std::string token = generate_unique_token(cur_frame_count_[device_name], "samples_data_" + device_name, start_system_time_token_);
            // 生成 ego_pose token
            std::string ego_pose_token = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
            // 生成 sample token
            int sample_frame_num = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame_num, start_system_time_token_);
            // 生成 prev 和 next token
            auto [prev_token, next_token] = generate_sample_prev_and_next(sample_frame_num);

            // 提取相对路径
            std::string relative_path = extractRelativePath(image_save_file);

            // 获取校准传感器 token
            std::string calibrated_sensor_token;
            auto calibrated_sensor_it = calibrated_sensor_token_map_.find(device_name);
            if (calibrated_sensor_it != calibrated_sensor_token_map_.end()) {
                calibrated_sensor_token = calibrated_sensor_it->second;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
                return;
            }

            // 填充 JSON 字段
            sample_data["token"] = token;
            sample_data["sample_token"] = sample_token;  // 指向 sample 所关联的 sample_data
            sample_data["ego_pose_token"] = ego_pose_token;
            sample_data["calibrated_sensor_token"] = calibrated_sensor_token;  
            sample_data["timestamp"] = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;  // Full timestamp
            sample_data["fileformat"] = "jpg";  // File format is JPEG
            sample_data["is_key_frame"] = (data_type == "samples") ? true : false;  // 根据 data_type 设置 key_frame
            sample_data["height"] = img.rows;
            sample_data["width"] = img.cols;
            sample_data["filename"] = relative_path;
            sample_data["prev"] = prev_token;  // 上一帧 token
            sample_data["next"] = next_token;  // 下一帧 token

            // 将数据写入 JSON
            json_writer_.addKeyValuePairToJson("sample_data", sample_data, true);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in save_image_data: %s", e.what());
        }
    }

    /// 保存 BBox3D 数据并写入 sample_annotation.json
    void save_bbox3d_data(const vision_msgs::msg::Detection3DArray::ConstSharedPtr &msg, const std::string &device_name) {
        // 确定保存类型为 'sweeps' 或 'samples'
        std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
    
        if (data_type == "samples") {
            // 用于保存所有结果的JSON对象
            std::vector<json> sample_annotation_jsons;
            // 生成 sample_token
            int sample_frame_num = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame_num, start_system_time_token_);
            // 遍历 Detection3DArray 中的每个检测结果
            for (const auto &detection : msg->detections) {
                json sample_annotation_json;
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
                sample_annotation_json["attribute_tokens"] = json::array();  // 如果有属性 tokens，可以填充这个字段
                // translation：从 bbox 中获取位置
                const auto &bbox = detection.bbox;
                sample_annotation_json["translation"] = {bbox.center.position.x, bbox.center.position.y, bbox.center.position.z};
                // rotation：从 bbox 中获取方向（四元数）
                sample_annotation_json["rotation"] = {bbox.center.orientation.x, bbox.center.orientation.y, bbox.center.orientation.z, bbox.center.orientation.w};
                // size：从 bbox 数据获取
                sample_annotation_json["size"] = {bbox.size.x, bbox.size.y, bbox.size.z};
                // 填充 prev 和 next
                sample_annotation_json["prev"] = "";
                sample_annotation_json["next"] = "";  // 将在滑动窗口中解决
                // 填充 num_lidar_pts 和 num_radar_pts
                sample_annotation_json["num_lidar_pts"] = -1;  // 示例数据
                sample_annotation_json["num_radar_pts"] = -1;  // 示例数据
                sample_annotation_jsons.emplace_back(sample_annotation_json);

                // ******************生成 instance.json********************
                auto& val = instance_recorders[instance_token];
                json instance_json; 
                if (val.empty()) {
                    // 如果 vector 为空，初始化一个新的 JSON 对象
                    instance_json["token"] = instance_token;
                    instance_json["category_token"] = "1fa93b757fc74fb197cdd60001ad8abf";  // TODO:等待获取 category_token
                    instance_json["nbr_annotations"] = 1;
                    instance_json["first_annotation_token"] = sample_annotation_token;
                    instance_json["last_annotation_token"] = sample_annotation_token;
                    // 将该 JSON 对象插入到 unordered_map 对应的 vector 中
                    val.push_back(instance_json);
                } else {
                    // 如果 vector 不为空，增加一个注解数量
                    instance_json = val.back(); // 取出最后一个元素，假设每次更新是对最后一个 JSON 对象的更新
                    instance_json["nbr_annotations"] = instance_json["nbr_annotations"].get<int>() + 1;
                    instance_json["last_annotation_token"] = sample_annotation_token; // 更新 last_annotation_token
                    // 更新 vector 中的最后一个 JSON 对象
                    val.back() = instance_json;
                }
            }
            // 处理滑动窗口中的 prev 和 next
            for(auto& annotation_json : sample_annotation_jsons){
                auto& annotation_slider = sample_annotation_sliders[annotation_json["instance_token"]];  // 更新 sample_annotation_sliders 中的 instance
                if(annotation_slider.size() < sample_annotation_cache_size){
                    annotation_slider.push_back(annotation_json);
                    if(annotation_slider.size() == 2){  // 当滑动窗口满时
                        annotation_slider[0]["next"] = annotation_slider[1]["token"];
                        annotation_slider[1]["prev"] = annotation_slider[0]["token"];
                        json_writer_.addKeyValuePairToJson("sample_annotation", annotation_slider[0], true);
                        // 移动滑动窗口
                        annotation_slider[0] = annotation_slider[1];  
                        annotation_slider.pop_back();
                    }
                }
            }
        }
    }

    /// 保存激光雷达数据并写入 sample_data.json
    void save_lidar_data(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &device_name) {
        try {
            // 确定保存类型为 'sweeps' 或 'samples'
            std::string data_type = (cur_frame_count_[device_name] % sample_interval_ == 0) ? "samples" : "sweeps";
            // 创建保存路径
            std::string folder_path = root_path_ + "/" + data_type + "/" + device_name;
            ensure_directory_exists(folder_path);
            // 生成保存文件路径
            std::stringstream ss;
            ss << folder_path << '/' << start_system_time_token_ << "__" << device_name << "__" 
               << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".pcl.bin";
            std::string cloud_save_file = ss.str();

            // 转换 PointCloud2 到 pcl::PointCloud
            pcl::PointCloud<pcl::PointXYZ> cloud;

            // 步骤 1: Convert sensor_msgs::msg::PointCloud2 to pcl::PCLPointCloud2
            pcl::PCLPointCloud2 pcl_cloud2;
            pcl_conversions::toPCL(*msg, pcl_cloud2);  // 使用 pcl_conversions::toPCL

            // 步骤 2: Convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
            pcl::fromPCLPointCloud2(pcl_cloud2, cloud);  // 使用 pcl::fromPCLPointCloud2

            // 保存点云到二进制文件
            std::ofstream ofs(cloud_save_file, std::ios::binary);
            if (ofs.is_open()) {
                ofs.write(reinterpret_cast<const char*>(cloud.points.data()), cloud.points.size() * sizeof(pcl::PointXYZ));
                ofs.close();
                RCLCPP_INFO(this->get_logger(), "PointCloud2 saved as BIN: %s", cloud_save_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for saving point cloud: %s", cloud_save_file.c_str());
                return;
            }

            // 准备 JSON 数据
            json sample_data;

            uint64_t timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1e9 + msg->header.stamp.nanosec;

            bool is_key_frame = (data_type == "samples");  // 根据 data_type 设置 key_frame

            // 生成 sample_token
            int sample_frame_num = cur_frame_count_[device_name] / sample_interval_;
            std::string sample_token = generate_sample_token(sample_frame_num, start_system_time_token_);
            if(is_key_frame){
                json sample_json;
                // 生成 prev 和 next token
                auto [prev_token, next_token] = generate_sample_prev_and_next(sample_frame_num);

                sample_json["token"] = sample_token;
                sample_json["timestamp"] = timestamp;
                sample_json["prev"] = prev_token;
                sample_json["next"] = next_token;
                sample_json["scene_token"] = cur_scene_token_;
                json_writer_.addKeyValuePairToJson("sample", sample_json, true);  
            }

            // 获取 ego_pose_token
            std::string ego_pose_token = generate_unique_token(cur_frame_count_[device_name], ego_pose_token_const, start_system_time_token_);
            // 获取 calibrated_sensor_token
            std::string calibrated_sensor_token;
            auto calibrated_sensor_it = calibrated_sensor_token_map_.find(device_name);
            if (calibrated_sensor_it != calibrated_sensor_token_map_.end()) {
                calibrated_sensor_token = calibrated_sensor_it->second;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Device name not found in calibrated_sensor_token_map_: %s", device_name.c_str());
                return;
            }

            // 生成 prev 和 next token
            auto [prev_token, next_token] = generate_sample_prev_and_next(sample_frame_num);

            // 提取相对路径
            std::string relative_path = extractRelativePath(cloud_save_file);

            // 填充 JSON 字段
            sample_data["token"] = generate_unique_token(cur_frame_count_[device_name], "samples_data_" + device_name, start_system_time_token_);
            sample_data["sample_token"] = sample_token;
            sample_data["ego_pose_token"] = ego_pose_token;
            sample_data["calibrated_sensor_token"] = calibrated_sensor_token;
            sample_data["timestamp"] = timestamp;
            sample_data["fileformat"] = "bin";  // File format matches the extension
            sample_data["is_key_frame"] = is_key_frame;
            sample_data["height"] = 0; // Or appropriate value
            sample_data["width"] = 0;  // Or appropriate value
            sample_data["filename"] = relative_path;
            sample_data["prev"] = prev_token;
            sample_data["next"] = next_token;

            // 将数据写入 JSON
            json_writer_.addKeyValuePairToJson("sample_data", sample_data, true);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in save_lidar_data: %s", e.what());
        }
    }

    // 生成 sample_token
    std::string generate_sample_token(int sample_frame, const std::string& start_time){
        return generate_unique_token(sample_frame, "sample", start_time);
    }

    // 生成 prev 和 next token
    std::tuple<std::string, std::string> generate_sample_prev_and_next(int sample_frame) {
        std::string sample_prev_token, sample_next_token;
        if(sample_frame > 0)
            sample_prev_token = generate_unique_token(sample_frame - 1, "sample", start_system_time_token_);
        else 
            sample_prev_token = "";
        if((sample_frame + 1) < target_frame_count)
            sample_next_token = generate_unique_token(sample_frame + 1, "sample", start_system_time_token_);
        else 
            sample_next_token = "";
        return std::make_tuple(sample_prev_token, sample_next_token);
    }

    // 检查是否所有任务完成
    void check_and_shutdown(){
        all_task_done_ = true;
        for(auto &device : sample_sensors_){
            if(cur_frame_count_[device] < target_frame_count){
                all_task_done_ = false;
                break;
            }
        }

        if(all_task_done_){
            RCLCPP_INFO(this->get_logger(), "All devices have reached target frame count. Shutting down the node.");
            // 遍历所有传感器并打印其帧计数和完成状态
            for (auto device : sample_sensors_) {
                RCLCPP_INFO(this->get_logger(), "Device: %s, Frame Count: %llu, Done Status: %s", 
                            device.c_str(), cur_frame_count_[device], 
                            sensor_done_signs_[device] ? "true" : "false");
            }
            all_task_done_callback();
            rclcpp::shutdown();  // 如果所有设备完成，关闭节点
        }
    }

    void all_task_done_callback(){
        // 写入 sample_annotation (不包含 next)
        for (auto& slider : sample_annotation_sliders) {
            if (slider.second.size() == 1) {
                // 将唯一的 sample_annotation 写入 JSON 文件
                json_writer_.addKeyValuePairToJson("sample_annotation", slider.second[0], true);
            }
        }
        // 写入所有唯一的 instance 到一个 scene
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
        ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
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

    // 确保指定目录存在
    void ensure_directory_exists(const std::string &path) {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path); // 使用 C++17 的文件系统库来创建目录
            RCLCPP_INFO(this->get_logger(), "Directory created: %s", path.c_str());
        }
    }
};

#endif  // NUSCENES_DATA_COLLECTOR_ISSAC_HPP
