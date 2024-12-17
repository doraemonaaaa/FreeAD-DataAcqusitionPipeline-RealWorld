#ifndef HARDWARE_BRIDGE_INTERFACE_HPP
#define HARDWARE_BRIDGE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_manager.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HardwareBridgeInterface : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::shared_ptr<HardwareManager> hardware_manager_;
    double hw_states_[2] = {0.0, 0.0};    // 左右轮子速度
    double hw_commands_[2] = {0.0, 0.0};  // 左右轮子目标速度
};

#endif // HARDWARE_BRIDGE_INTERFACE_HPP
