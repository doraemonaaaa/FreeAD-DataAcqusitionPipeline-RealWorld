#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_hardware/msg/motor_data.hpp" // 引入 MotorData 消息

// 接口为  输入速度vel   输出速度vel

uint8_t tdata;
uint8_t data[24];
class SerialBridge : public rclcpp::Node
{
    public:
        SerialBridge() : Node("serial_bridge_node")
        {
            expect_speed_pub_ = this->create_publisher<robot_hardware::msg::MotorData>("/serial/hardware_expect_speed_msg", 10);
            real_speed_sub_ = this->create_subscription<robot_hardware::msg::MotorData>(
                "/serial/hardware_real_speed_msg", 10, std::bind(&SerialBridge::receive_real_rpm_callback, this, std::placeholders::_1));
        }

        ~SerialBridge()
        {
        }

        void setup(double wheel_radius){
            wheel_radius_ = wheel_radius;
        }

        std::tuple<double, double> read_motor_vel()
        {
            double vel_l, vel_r;
            vel_l = rpm_to_velocity(hardware_data_.real_left_speed_rpm);
            vel_r = rpm_to_velocity(hardware_data_.real_right_speed_rpm);
            vel_l = hardware_data_.real_left_neg_flag ? vel_l : -vel_l;
            vel_r = hardware_data_.real_left_neg_flag ? vel_r : -vel_r;
            return std::make_tuple(vel_l, vel_r);
        }

        void set_motor_vel(double vel_l, double vel_r)
        {
            // 调试输出
            //RCLCPP_INFO(rclcpp::get_logger("MotorControl"), "Received motor speeds: left=%.2f, right=%.2f", vel_l, vel_r);
            hardware_data_.expect_left_speed_rpm = velocity_to_rpm(std::abs(vel_l));
            hardware_data_.expect_right_speed_rpm = velocity_to_rpm(std::abs(vel_r));
            hardware_data_.expect_left_neg_flag = vel_l >= 0 ? 1 : 0;
            hardware_data_.expect_right_neg_flag = vel_r >= 0 ? 1 : 0;
            publish_expect_rpm();
        }

        // 接收实际速度的回调函数
        void receive_real_rpm_callback(const robot_hardware::msg::MotorData::SharedPtr msg) {
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
            auto msg = robot_hardware::msg::MotorData();
            msg.expect_left_speed_rpm = hardware_data_.expect_left_speed_rpm;
            msg.expect_right_speed_rpm = hardware_data_.expect_right_speed_rpm;
            msg.expect_left_neg_flag = hardware_data_.expect_left_neg_flag;
            msg.expect_right_neg_flag = hardware_data_.expect_right_neg_flag;              

            expect_speed_pub_->publish(msg);
        }


    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<robot_hardware::msg::MotorData>::SharedPtr expect_speed_pub_; // 发布期望速度
        rclcpp::Subscription<robot_hardware::msg::MotorData>::SharedPtr real_speed_sub_; // 订阅实际速度
        robot_hardware::msg::MotorData hardware_data_;  // 硬件数据
        double wheel_radius_;

        // 将 RPM 转换为线速度（米/秒）
        double rpm_to_velocity(std::int16_t rpm) const {
            return (rpm * 2.0 * M_PI * wheel_radius_) / 60.0;
        }

        // 将线速度（米/秒）转换为 RPM
        std::int16_t velocity_to_rpm(double velocity_m_per_s) const {
            return static_cast<std::int16_t>((velocity_m_per_s * 60.0) / (2.0 * M_PI * wheel_radius_));
        }

};
