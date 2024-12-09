// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__BUILDER_HPP_
#define ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_hardware/msg/detail/hardware_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_hardware
{

namespace msg
{

namespace builder
{

class Init_HardwareData_magnetic_field_z
{
public:
  explicit Init_HardwareData_magnetic_field_z(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  ::robot_hardware::msg::HardwareData magnetic_field_z(::robot_hardware::msg::HardwareData::_magnetic_field_z_type arg)
  {
    msg_.magnetic_field_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_magnetic_field_y
{
public:
  explicit Init_HardwareData_magnetic_field_y(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_magnetic_field_z magnetic_field_y(::robot_hardware::msg::HardwareData::_magnetic_field_y_type arg)
  {
    msg_.magnetic_field_y = std::move(arg);
    return Init_HardwareData_magnetic_field_z(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_magnetic_field_x
{
public:
  explicit Init_HardwareData_magnetic_field_x(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_magnetic_field_y magnetic_field_x(::robot_hardware::msg::HardwareData::_magnetic_field_x_type arg)
  {
    msg_.magnetic_field_x = std::move(arg);
    return Init_HardwareData_magnetic_field_y(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_angular_velocity_z
{
public:
  explicit Init_HardwareData_imu_angular_velocity_z(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_magnetic_field_x imu_angular_velocity_z(::robot_hardware::msg::HardwareData::_imu_angular_velocity_z_type arg)
  {
    msg_.imu_angular_velocity_z = std::move(arg);
    return Init_HardwareData_magnetic_field_x(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_angular_velocity_y
{
public:
  explicit Init_HardwareData_imu_angular_velocity_y(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_angular_velocity_z imu_angular_velocity_y(::robot_hardware::msg::HardwareData::_imu_angular_velocity_y_type arg)
  {
    msg_.imu_angular_velocity_y = std::move(arg);
    return Init_HardwareData_imu_angular_velocity_z(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_angular_velocity_x
{
public:
  explicit Init_HardwareData_imu_angular_velocity_x(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_angular_velocity_y imu_angular_velocity_x(::robot_hardware::msg::HardwareData::_imu_angular_velocity_x_type arg)
  {
    msg_.imu_angular_velocity_x = std::move(arg);
    return Init_HardwareData_imu_angular_velocity_y(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_linear_acceleration_z
{
public:
  explicit Init_HardwareData_imu_linear_acceleration_z(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_angular_velocity_x imu_linear_acceleration_z(::robot_hardware::msg::HardwareData::_imu_linear_acceleration_z_type arg)
  {
    msg_.imu_linear_acceleration_z = std::move(arg);
    return Init_HardwareData_imu_angular_velocity_x(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_linear_acceleration_y
{
public:
  explicit Init_HardwareData_imu_linear_acceleration_y(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_linear_acceleration_z imu_linear_acceleration_y(::robot_hardware::msg::HardwareData::_imu_linear_acceleration_y_type arg)
  {
    msg_.imu_linear_acceleration_y = std::move(arg);
    return Init_HardwareData_imu_linear_acceleration_z(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_linear_acceleration_x
{
public:
  explicit Init_HardwareData_imu_linear_acceleration_x(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_linear_acceleration_y imu_linear_acceleration_x(::robot_hardware::msg::HardwareData::_imu_linear_acceleration_x_type arg)
  {
    msg_.imu_linear_acceleration_x = std::move(arg);
    return Init_HardwareData_imu_linear_acceleration_y(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_orientation_w
{
public:
  explicit Init_HardwareData_imu_orientation_w(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_linear_acceleration_x imu_orientation_w(::robot_hardware::msg::HardwareData::_imu_orientation_w_type arg)
  {
    msg_.imu_orientation_w = std::move(arg);
    return Init_HardwareData_imu_linear_acceleration_x(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_orientation_z
{
public:
  explicit Init_HardwareData_imu_orientation_z(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_orientation_w imu_orientation_z(::robot_hardware::msg::HardwareData::_imu_orientation_z_type arg)
  {
    msg_.imu_orientation_z = std::move(arg);
    return Init_HardwareData_imu_orientation_w(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_orientation_y
{
public:
  explicit Init_HardwareData_imu_orientation_y(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_orientation_z imu_orientation_y(::robot_hardware::msg::HardwareData::_imu_orientation_y_type arg)
  {
    msg_.imu_orientation_y = std::move(arg);
    return Init_HardwareData_imu_orientation_z(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_imu_orientation_x
{
public:
  explicit Init_HardwareData_imu_orientation_x(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_orientation_y imu_orientation_x(::robot_hardware::msg::HardwareData::_imu_orientation_x_type arg)
  {
    msg_.imu_orientation_x = std::move(arg);
    return Init_HardwareData_imu_orientation_y(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_expect_right_neg_flag
{
public:
  explicit Init_HardwareData_expect_right_neg_flag(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_imu_orientation_x expect_right_neg_flag(::robot_hardware::msg::HardwareData::_expect_right_neg_flag_type arg)
  {
    msg_.expect_right_neg_flag = std::move(arg);
    return Init_HardwareData_imu_orientation_x(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_expect_left_neg_flag
{
public:
  explicit Init_HardwareData_expect_left_neg_flag(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_right_neg_flag expect_left_neg_flag(::robot_hardware::msg::HardwareData::_expect_left_neg_flag_type arg)
  {
    msg_.expect_left_neg_flag = std::move(arg);
    return Init_HardwareData_expect_right_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_expect_right_speed_rpm
{
public:
  explicit Init_HardwareData_expect_right_speed_rpm(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_left_neg_flag expect_right_speed_rpm(::robot_hardware::msg::HardwareData::_expect_right_speed_rpm_type arg)
  {
    msg_.expect_right_speed_rpm = std::move(arg);
    return Init_HardwareData_expect_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_expect_left_speed_rpm
{
public:
  explicit Init_HardwareData_expect_left_speed_rpm(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_right_speed_rpm expect_left_speed_rpm(::robot_hardware::msg::HardwareData::_expect_left_speed_rpm_type arg)
  {
    msg_.expect_left_speed_rpm = std::move(arg);
    return Init_HardwareData_expect_right_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_real_right_neg_flag
{
public:
  explicit Init_HardwareData_real_right_neg_flag(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_left_speed_rpm real_right_neg_flag(::robot_hardware::msg::HardwareData::_real_right_neg_flag_type arg)
  {
    msg_.real_right_neg_flag = std::move(arg);
    return Init_HardwareData_expect_left_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_real_left_neg_flag
{
public:
  explicit Init_HardwareData_real_left_neg_flag(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_real_right_neg_flag real_left_neg_flag(::robot_hardware::msg::HardwareData::_real_left_neg_flag_type arg)
  {
    msg_.real_left_neg_flag = std::move(arg);
    return Init_HardwareData_real_right_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_real_right_speed_rpm
{
public:
  explicit Init_HardwareData_real_right_speed_rpm(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_real_left_neg_flag real_right_speed_rpm(::robot_hardware::msg::HardwareData::_real_right_speed_rpm_type arg)
  {
    msg_.real_right_speed_rpm = std::move(arg);
    return Init_HardwareData_real_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_real_left_speed_rpm
{
public:
  Init_HardwareData_real_left_speed_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HardwareData_real_right_speed_rpm real_left_speed_rpm(::robot_hardware::msg::HardwareData::_real_left_speed_rpm_type arg)
  {
    msg_.real_left_speed_rpm = std::move(arg);
    return Init_HardwareData_real_right_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware::msg::HardwareData>()
{
  return robot_hardware::msg::builder::Init_HardwareData_real_left_speed_rpm();
}

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__BUILDER_HPP_
