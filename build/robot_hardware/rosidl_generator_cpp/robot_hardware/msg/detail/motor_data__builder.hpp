// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_hardware:msg/MotorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__BUILDER_HPP_
#define ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_hardware/msg/detail/motor_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_hardware
{

namespace msg
{

namespace builder
{

class Init_MotorData_expect_right_neg_flag
{
public:
  explicit Init_MotorData_expect_right_neg_flag(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  ::robot_hardware::msg::MotorData expect_right_neg_flag(::robot_hardware::msg::MotorData::_expect_right_neg_flag_type arg)
  {
    msg_.expect_right_neg_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_expect_left_neg_flag
{
public:
  explicit Init_MotorData_expect_left_neg_flag(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_expect_right_neg_flag expect_left_neg_flag(::robot_hardware::msg::MotorData::_expect_left_neg_flag_type arg)
  {
    msg_.expect_left_neg_flag = std::move(arg);
    return Init_MotorData_expect_right_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_expect_right_speed_rpm
{
public:
  explicit Init_MotorData_expect_right_speed_rpm(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_expect_left_neg_flag expect_right_speed_rpm(::robot_hardware::msg::MotorData::_expect_right_speed_rpm_type arg)
  {
    msg_.expect_right_speed_rpm = std::move(arg);
    return Init_MotorData_expect_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_expect_left_speed_rpm
{
public:
  explicit Init_MotorData_expect_left_speed_rpm(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_expect_right_speed_rpm expect_left_speed_rpm(::robot_hardware::msg::MotorData::_expect_left_speed_rpm_type arg)
  {
    msg_.expect_left_speed_rpm = std::move(arg);
    return Init_MotorData_expect_right_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_real_right_neg_flag
{
public:
  explicit Init_MotorData_real_right_neg_flag(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_expect_left_speed_rpm real_right_neg_flag(::robot_hardware::msg::MotorData::_real_right_neg_flag_type arg)
  {
    msg_.real_right_neg_flag = std::move(arg);
    return Init_MotorData_expect_left_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_real_left_neg_flag
{
public:
  explicit Init_MotorData_real_left_neg_flag(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_real_right_neg_flag real_left_neg_flag(::robot_hardware::msg::MotorData::_real_left_neg_flag_type arg)
  {
    msg_.real_left_neg_flag = std::move(arg);
    return Init_MotorData_real_right_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_real_right_speed_rpm
{
public:
  explicit Init_MotorData_real_right_speed_rpm(::robot_hardware::msg::MotorData & msg)
  : msg_(msg)
  {}
  Init_MotorData_real_left_neg_flag real_right_speed_rpm(::robot_hardware::msg::MotorData::_real_right_speed_rpm_type arg)
  {
    msg_.real_right_speed_rpm = std::move(arg);
    return Init_MotorData_real_left_neg_flag(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

class Init_MotorData_real_left_speed_rpm
{
public:
  Init_MotorData_real_left_speed_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorData_real_right_speed_rpm real_left_speed_rpm(::robot_hardware::msg::MotorData::_real_left_speed_rpm_type arg)
  {
    msg_.real_left_speed_rpm = std::move(arg);
    return Init_MotorData_real_right_speed_rpm(msg_);
  }

private:
  ::robot_hardware::msg::MotorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_hardware::msg::MotorData>()
{
  return robot_hardware::msg::builder::Init_MotorData_real_left_speed_rpm();
}

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__BUILDER_HPP_
