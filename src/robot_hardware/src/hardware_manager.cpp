#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include "robot_hardware/msg/hardware_data.hpp" // 引入 HardwareData 消息
#include <opencv2/opencv.hpp> // 引入 OpenCV
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp> // 引入 ROS 传感器消息,图像的
#include <sensor_msgs/msg/imu.hpp>             // IMU 消息
#include <sensor_msgs/msg/magnetic_field.hpp>  // 磁力计消息
#include "wheel.hpp"


class HardwareManager : public rclcpp::Node {
public:
    HardwareManager() : Node("hardware_manager") {

        left_wheel = Wheel("Left_Wheel", 0.12);
        right_wheel = Wheel("Right_Wheel", 0.12);
        
        // 发布和订阅 ROS 2 消息
        expect_speed_pub_ = this->create_publisher<robot_hardware::msg::HardwareData>("/serial/hardware_expect_speed_msg", 10);

        real_speed_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
            "/serial/hardware_real_speed_msg", 10, std::bind(&HardwareManager::receive_real_rpm_callback, this, std::placeholders::_1));

        // 订阅键盘命令，使用 HardwareData 消息类型
        keyboard_cmd_sub_ = this->create_subscription<robot_hardware::msg::HardwareData>(
            "keyboard_cmd", 10, std::bind(&HardwareManager::receive_keyboard_command_callback, this, std::placeholders::_1));

        // 订阅 IMU 数据
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/demo/imu", 10, std::bind(&HardwareManager::imu_callback, this, std::placeholders::_1));

        // 订阅磁力计数据
        mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/mag", 10, std::bind(&HardwareManager::mag_callback, this, std::placeholders::_1));

        // 初始化定时器，定期发布期望速度,帧率khz,串口下发速率也是khz
        feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&HardwareManager::publish_expect_rpm, this));

        // 创建 image_transport 订阅器
        // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "camera/image_raw", 10, std::bind(&HardwareManager::usb_cam_image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hardware Manager initialized.");
    }

private:
    robot_hardware::msg::HardwareData hardware_data_;  // 硬件数据
    rclcpp::Publisher<robot_hardware::msg::HardwareData>::SharedPtr expect_speed_pub_;
    rclcpp::Subscription<robot_hardware::msg::HardwareData>::SharedPtr real_speed_sub_;
    rclcpp::Subscription<robot_hardware::msg::HardwareData>::SharedPtr keyboard_cmd_sub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // 图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;           // IMU 订阅器
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_; // 磁力计订阅器

    Wheel left_wheel; 
    Wheel right_wheel;  

    // IMU 数据回调
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        hardware_data_.imu_orientation_x = msg->orientation.x;
        hardware_data_.imu_orientation_y = msg->orientation.y;
        hardware_data_.imu_orientation_z = msg->orientation.z;
        hardware_data_.imu_orientation_w = msg->orientation.w;

        hardware_data_.imu_linear_acceleration_x = msg->linear_acceleration.x;
        hardware_data_.imu_linear_acceleration_y = msg->linear_acceleration.y;
        hardware_data_.imu_linear_acceleration_z = msg->linear_acceleration.z;

        hardware_data_.imu_angular_velocity_x = msg->angular_velocity.x;
        hardware_data_.imu_angular_velocity_y = msg->angular_velocity.y;
        hardware_data_.imu_angular_velocity_z = msg->angular_velocity.z;

        RCLCPP_INFO(this->get_logger(), "IMU data received.");
    }

    // 磁力计数据回调
    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        hardware_data_.magnetic_field_x = msg->magnetic_field.x;
        hardware_data_.magnetic_field_y = msg->magnetic_field.y;
        hardware_data_.magnetic_field_z = msg->magnetic_field.z;

        RCLCPP_INFO(this->get_logger(), "Magnetic field data received.");
    }


    // 接收键盘指令的回调函数，更新期望速度
    void receive_keyboard_command_callback(const robot_hardware::msg::HardwareData::SharedPtr msg) {
        hardware_data_.expect_left_speed_rpm = msg->expect_left_speed_rpm;
        hardware_data_.expect_right_speed_rpm = msg->expect_right_speed_rpm;
        hardware_data_.expect_left_neg_flag = msg->expect_left_neg_flag;
        hardware_data_.expect_right_neg_flag = msg->expect_right_neg_flag;

        // RCLCPP_INFO(this->get_logger(), "Received keyboard command: left=%d, right=%d",
        //             hardware_data_.expect_left_speed_rpm, hardware_data_.expect_right_speed_rpm);
    }


    // 接收实际速度的回调函数
    void receive_real_rpm_callback(const robot_hardware::msg::HardwareData::SharedPtr msg) {
        // 更新硬件数据中的实际速度和方向标志
        hardware_data_.real_left_speed_rpm = msg->real_left_speed_rpm;
        hardware_data_.real_right_speed_rpm = msg->real_right_speed_rpm;
        hardware_data_.real_left_neg_flag = msg->real_left_neg_flag;
        hardware_data_.real_right_neg_flag = msg->real_right_neg_flag;

        // RCLCPP_INFO(this->get_logger(), "Received real speed: left=%d, right=%d",
        //             hardware_data_.real_left_speed_rpm, hardware_data_.real_right_speed_rpm);
    }
    
    // 定期发布期望速度
    void publish_expect_rpm() {
        auto msg = robot_hardware::msg::HardwareData();
        msg.expect_left_speed_rpm = hardware_data_.expect_left_speed_rpm;
        msg.expect_right_speed_rpm = hardware_data_.expect_right_speed_rpm;
        msg.expect_left_neg_flag = hardware_data_.expect_left_neg_flag;
        msg.expect_right_neg_flag = hardware_data_.expect_right_neg_flag;              

        expect_speed_pub_->publish(msg);
    }

    // 图像回调函数
    void usb_cam_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将 ROS 图像消息转换为 OpenCV 图像
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 显示图像
            cv::imshow("USB Camera frame", frame);

            // 按键退出检测
            if (cv::waitKey(1) == 27) { // 按 ESC 键退出
                cv::destroyAllWindows();
                RCLCPP_INFO(this->get_logger(), "Closed USB Camera Feed.");
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
