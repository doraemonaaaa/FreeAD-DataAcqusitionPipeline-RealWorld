// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_hardware:msg/MotorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__STRUCT_HPP_
#define ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_hardware__msg__MotorData __attribute__((deprecated))
#else
# define DEPRECATED__robot_hardware__msg__MotorData __declspec(deprecated)
#endif

namespace robot_hardware
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorData_
{
  using Type = MotorData_<ContainerAllocator>;

  explicit MotorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->real_left_speed_rpm = 0;
      this->real_right_speed_rpm = 0;
      this->real_left_neg_flag = 0;
      this->real_right_neg_flag = 0;
      this->expect_left_speed_rpm = 0;
      this->expect_right_speed_rpm = 0;
      this->expect_left_neg_flag = 0;
      this->expect_right_neg_flag = 0;
    }
  }

  explicit MotorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->real_left_speed_rpm = 0;
      this->real_right_speed_rpm = 0;
      this->real_left_neg_flag = 0;
      this->real_right_neg_flag = 0;
      this->expect_left_speed_rpm = 0;
      this->expect_right_speed_rpm = 0;
      this->expect_left_neg_flag = 0;
      this->expect_right_neg_flag = 0;
    }
  }

  // field types and members
  using _real_left_speed_rpm_type =
    int16_t;
  _real_left_speed_rpm_type real_left_speed_rpm;
  using _real_right_speed_rpm_type =
    int16_t;
  _real_right_speed_rpm_type real_right_speed_rpm;
  using _real_left_neg_flag_type =
    uint8_t;
  _real_left_neg_flag_type real_left_neg_flag;
  using _real_right_neg_flag_type =
    uint8_t;
  _real_right_neg_flag_type real_right_neg_flag;
  using _expect_left_speed_rpm_type =
    int16_t;
  _expect_left_speed_rpm_type expect_left_speed_rpm;
  using _expect_right_speed_rpm_type =
    int16_t;
  _expect_right_speed_rpm_type expect_right_speed_rpm;
  using _expect_left_neg_flag_type =
    uint8_t;
  _expect_left_neg_flag_type expect_left_neg_flag;
  using _expect_right_neg_flag_type =
    uint8_t;
  _expect_right_neg_flag_type expect_right_neg_flag;

  // setters for named parameter idiom
  Type & set__real_left_speed_rpm(
    const int16_t & _arg)
  {
    this->real_left_speed_rpm = _arg;
    return *this;
  }
  Type & set__real_right_speed_rpm(
    const int16_t & _arg)
  {
    this->real_right_speed_rpm = _arg;
    return *this;
  }
  Type & set__real_left_neg_flag(
    const uint8_t & _arg)
  {
    this->real_left_neg_flag = _arg;
    return *this;
  }
  Type & set__real_right_neg_flag(
    const uint8_t & _arg)
  {
    this->real_right_neg_flag = _arg;
    return *this;
  }
  Type & set__expect_left_speed_rpm(
    const int16_t & _arg)
  {
    this->expect_left_speed_rpm = _arg;
    return *this;
  }
  Type & set__expect_right_speed_rpm(
    const int16_t & _arg)
  {
    this->expect_right_speed_rpm = _arg;
    return *this;
  }
  Type & set__expect_left_neg_flag(
    const uint8_t & _arg)
  {
    this->expect_left_neg_flag = _arg;
    return *this;
  }
  Type & set__expect_right_neg_flag(
    const uint8_t & _arg)
  {
    this->expect_right_neg_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_hardware::msg::MotorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_hardware::msg::MotorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_hardware::msg::MotorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_hardware::msg::MotorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_hardware::msg::MotorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_hardware::msg::MotorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_hardware::msg::MotorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_hardware::msg::MotorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_hardware::msg::MotorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_hardware::msg::MotorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_hardware__msg__MotorData
    std::shared_ptr<robot_hardware::msg::MotorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_hardware__msg__MotorData
    std::shared_ptr<robot_hardware::msg::MotorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorData_ & other) const
  {
    if (this->real_left_speed_rpm != other.real_left_speed_rpm) {
      return false;
    }
    if (this->real_right_speed_rpm != other.real_right_speed_rpm) {
      return false;
    }
    if (this->real_left_neg_flag != other.real_left_neg_flag) {
      return false;
    }
    if (this->real_right_neg_flag != other.real_right_neg_flag) {
      return false;
    }
    if (this->expect_left_speed_rpm != other.expect_left_speed_rpm) {
      return false;
    }
    if (this->expect_right_speed_rpm != other.expect_right_speed_rpm) {
      return false;
    }
    if (this->expect_left_neg_flag != other.expect_left_neg_flag) {
      return false;
    }
    if (this->expect_right_neg_flag != other.expect_right_neg_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorData_

// alias to use template instance with default allocator
using MotorData =
  robot_hardware::msg::MotorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MSG__DETAIL__MOTOR_DATA__STRUCT_HPP_
