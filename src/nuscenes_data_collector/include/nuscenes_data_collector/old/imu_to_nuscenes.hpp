#ifndef IMU_TO_NUSCENES_HPP
#define IMU_TO_NUSCENES_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>  // C++17 文件系统库

using namespace std;
using json = nlohmann::json;

// 用于确保目录存在
void ensure_directory_exists(const std::string &path) {
    std::filesystem::create_directories(path);  // 创建目录及其父目录
}

class ImuToNuScenes : public rclcpp::Node
{
public:
    // 修改构造函数，接受节点名和topic名
    ImuToNuScenes()
        : Node("imu_to_nuscenes")
    {
        // 声明参数
        this->declare_parameter<std::string>("imu_topic", "/issac/imu");
        // 获取参数值
        std::string topic_name = this->get_parameter("imu_topic").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, 10, std::bind(&ImuToNuScenes::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 使用 timestamp 生成唯一文件名
        std::string timestamp = std::to_string(msg->header.stamp.sec) + "_" + std::to_string(msg->header.stamp.nanosec);
        std::string save_path = "/home/pyh/Documents/DataSet/imu";  // 设置 IMU 数据保存的路径，确保路径以 /imu 结尾

        // 确保目录存在
        ensure_directory_exists(save_path);  // 确保目录存在，save_path 已包含 /imu

        // 创建 NuScenes 数据格式的 JSON 结构
        json imu_data;
        imu_data["timestamp"] = msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec;
        imu_data["orientation"] = {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
        imu_data["angular_velocity"] = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        imu_data["linear_acceleration"] = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};

        // 保存数据到 JSON 文件，文件名包含 timestamp
        std::ofstream json_file(save_path + "/imu_data_" + timestamp + ".json");
        if (json_file.is_open()) {
            json_file << imu_data.dump(4);  // 使用 pretty print 格式输出
            json_file.close();
            RCLCPP_INFO(this->get_logger(), "IMU data saved to imu_data_%s.json", timestamp.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: imu_data_%s.json", timestamp.c_str());
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

#endif  // IMU_TO_NUSCENES_HPP
