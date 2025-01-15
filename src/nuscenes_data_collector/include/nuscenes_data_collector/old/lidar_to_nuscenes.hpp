#ifndef LIDAR_TO_NUSCENES_HPP
#define LIDAR_TO_NUSCENES_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

class LidarToNuScenes : public rclcpp::Node
{
public:
    // 修改构造函数，接受节点名和topic名
    LidarToNuScenes()
        : Node("lidar_to_nuscenes")
    {
        // 声明参数
        this->declare_parameter<std::string>("lidar_topic", "/issac/point_cloud");
        // 获取参数值
        std::string topic_name = this->get_parameter("lidar_topic").as_string();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&LidarToNuScenes::lidar_callback, this, std::placeholders::_1));
    }

private:
    // 用于确保目录存在
    void ensure_directory_exists(const std::string &path) {
        std::filesystem::create_directories(path);  // 创建目录及其父目录
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 转换 ROS 消息为 PCL 点云
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // 获取时间戳，并构建文件名
        std::string timestamp = std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nanosec);
        std::string save_path = "/home/pyh/Documents/DataSet";  // 定义保存路径
        std::string lidar_bin_filename = save_path + "/lidar/" + timestamp + ".bin";  // 文件路径包含 timestamp

        // 确保目录存在，确保 /home/pyh/Documents/DataSet/lidar 目录存在
        ensure_directory_exists(save_path + "/lidar");

        // 保存 LIDAR 点云数据到 .bin 文件
        std::ofstream ofs(lidar_bin_filename, std::ios::binary);
        for (const auto& point : cloud.points)
        {
            ofs.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
            ofs.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
            ofs.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
        }
        ofs.close();

        // 创建与 LIDAR 数据关联的 JSON 数据
        json sample_data;
        sample_data["token"] = "unique_token";
        sample_data["ego_pose_token"] = "ego_pose_token";
        sample_data["timestamp"] = msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec;
        sample_data["sensor_modality"] = "lidar";
        sample_data["filename"] = lidar_bin_filename;
        sample_data["is_key_frame"] = true;
        sample_data["sensor_token"] = "sensor_token";
        sample_data["calibration"] = {
            {"camera_intrinsic", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0}},
            {"translation", {0.0, 0.0, 0.0}},
            {"rotation", {0.0, 0.0, 0.0}}
        };

        // 确保目录存在，确保 /home/pyh/Documents/DataSet/lidar 目录存在
        ensure_directory_exists(save_path + "/lidar");

        // 保存 JSON 文件到指定路径
        std::ofstream json_file(save_path + "/lidar/sample_data_" + timestamp + ".json");
        json_file << sample_data.dump(4);
        json_file.close();

        // 输出日志
        RCLCPP_INFO(this->get_logger(), "LIDAR data saved to %s", lidar_bin_filename.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#endif  // LIDAR_TO_NUSCENES_HPP
