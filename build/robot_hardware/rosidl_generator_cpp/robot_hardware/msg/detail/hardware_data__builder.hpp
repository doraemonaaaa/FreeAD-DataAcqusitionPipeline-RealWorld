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

class Init_HardwareData_expect_right_neg_flag
{
public:
  explicit Init_HardwareData_expect_right_neg_flag(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  ::robot_hardware::msg::HardwareData expect_right_neg_flag(::robot_hardware::msg::HardwareData::_expect_right_neg_flag_type arg)
  {
    msg_.expect_right_neg_flag = std::move(arg);
    return std::move(msg_);
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

class Init_HardwareData_expect_right_speed
{
public:
  explicit Init_HardwareData_expect_right_speed(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_left_neg_flag expect_right_speed(::robot_hardware::msg::HardwareData::_expect_right_speed_type arg)
  {
    msg_.expect_right_speed = std::move(arg);
    return Init_HardwareData_expect_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_expect_left_speed
{
public:
  explicit Init_HardwareData_expect_left_speed(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_expect_right_speed expect_left_speed(::robot_hardware::msg::HardwareData::_expect_left_speed_type arg)
  {
    msg_.expect_left_speed = std::move(arg);
    return Init_HardwareData_expect_right_speed(msg_);
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
  Init_HardwareData_expect_left_speed real_right_neg_flag(::robot_hardware::msg::HardwareData::_real_right_neg_flag_type arg)
  {
    msg_.real_right_neg_flag = std::move(arg);
    return Init_HardwareData_expect_left_speed(msg_);
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

class Init_HardwareData_real_right_speed
{
public:
  explicit Init_HardwareData_real_right_speed(::robot_hardware::msg::HardwareData & msg)
  : msg_(msg)
  {}
  Init_HardwareData_real_left_neg_flag real_right_speed(::robot_hardware::msg::HardwareData::_real_right_speed_type arg)
  {
    msg_.real_right_speed = std::move(arg);
    return Init_HardwareData_real_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::HardwareData msg_;
};

class Init_HardwareData_real_left_speed
{
public:
  Init_HardwareData_real_left_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HardwareData_real_right_speed real_left_speed(::robot_hardware::msg::HardwareData::_real_left_speed_type arg)
  {
    msg_.real_left_speed = std::move(arg);
    return Init_HardwareData_real_right_speed(msg_);
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
  return robot_hardware::msg::builder::Init_HardwareData_real_left_speed();
}

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__BUILDER_HPP_
