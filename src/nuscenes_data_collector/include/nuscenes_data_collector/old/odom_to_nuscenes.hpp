#ifndef ODOM_TO_NUSCENES_HPP
#define ODOM_TO_NUSCENES_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>

using namespace std;
using json = nlohmann::json;

class OdomToNuScenes : public rclcpp::Node
{
public:
    OdomToNuScenes()
        : Node("odom_to_nuscenes")
    {
        // Declare parameter for topic name
        this->declare_parameter<std::string>("odom_topic", "/issac/odom");
        std::string topic_name = this->get_parameter("odom_topic").as_string();

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, 10, std::bind(&OdomToNuScenes::odom_callback, this, std::placeholders::_1));
    }

private:
    void ensure_directory_exists(const std::string &path) {
        std::filesystem::create_directories(path);  // Ensure the directory exists
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Construct timestamp and file path
        std::string timestamp = std::to_string(msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec);
        std::string save_path = "/home/pyh/Documents/DataSet";  // Define save path
        std::string odom_json_filename = save_path + "/odom/" + timestamp + ".json";  // File path with timestamp

        // Ensure the directory exists
        ensure_directory_exists(save_path + "/odom");

        // Create JSON structure matching ego_pose.json format
        json sample_data;
        sample_data["token"] = "unique_token";  // Replace with actual token generation if needed
        sample_data["timestamp"] = msg->header.stamp.sec * 1000000000 + msg->header.stamp.nanosec;

        // Extract pose information
        sample_data["rotation"] = {
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        };

        sample_data["translation"] = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };

        // Save JSON to file
        std::ofstream json_file(odom_json_filename);
        json_file << sample_data.dump(4);  // Indented for readability
        json_file.close();

        // Log the saving action
        RCLCPP_INFO(this->get_logger(), "ODOM data saved to %s", odom_json_filename.c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

#endif  // ODOM_TO_NUSCENES_HPP
