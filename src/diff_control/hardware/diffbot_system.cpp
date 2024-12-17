// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diff_control/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_control
{
    // 定义了DiffBotSystemHardware类的on_init函数，用于硬件初始化
    hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        // 调用父类的on_init函数，确保初始化成功
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // rclcpp::init(0, nullptr);
        // rclcpp::spin(serial_bridge_p_);

        // 从配置文件中读取硬件参数并进行初始化
        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"]; 
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];  
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]); 
        cfg_.device = info_.hardware_parameters["device"];  
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);  
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);  
        cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);  
        cfg_.wheel_radius = std::stod(info_.hardware_parameters["wheel_radius"]); 

        try {
                // 设置左右轮子的参数
             // 设置轮子
            bool wheel_l_setup = wheel_l_.setup(cfg_.left_wheel_name, cfg_.wheel_radius);
            bool wheel_r_setup = wheel_r_.setup(cfg_.right_wheel_name, cfg_.wheel_radius);

            if (!wheel_l_setup || !wheel_r_setup) {
                throw std::runtime_error("Failed to setup wheels.");
            }

            // 设置串口
            bool serial_work = serial_bridge_p_->setup(cfg_.wheel_radius);
            if (!serial_work) {
                throw std::runtime_error("Failed to initialize serial communication.");
            }
        } catch (const std::exception& e) {
            // 处理所有异常，包括轮子初始化和串口初始化失败
            RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Error: %s", e.what());
            rclcpp::shutdown();  // 停止 ROS 2 系统
            return hardware_interface::CallbackReturn::ERROR;  // 返回错误，停止硬件接口的初始化
        }


        // 对关节配置进行检查
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 导出状态接口，将各状态信息添加到接口列表
    std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos_));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel_));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos_));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel_));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.ex_vel_));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.ex_vel_));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Confiugring ...please wait...");

        // serial_bridge_p_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Cleaning up ...please wait...");

        // serial_bridge_p_.disconnect();

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

        // serial_bridge_p_->set_pid_values(30, 20, 0, 100);

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffBotSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto [vel_l, vel_r] = serial_bridge_p_->read_motor_vel();
        wheel_l_.update_velocity_and_pos(vel_l);
        wheel_r_.update_velocity_and_pos(vel_r);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type diff_control ::DiffBotSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // wheel_l_.ex_vel_ = -1;
        // wheel_r_.ex_vel_ = -1;
        serial_bridge_p_->set_motor_vel(wheel_l_.ex_vel_, wheel_r_.ex_vel_);

        return hardware_interface::return_type::OK;
    }

} // namespace diff_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diff_control::DiffBotSystemHardware, hardware_interface::SystemInterface)
