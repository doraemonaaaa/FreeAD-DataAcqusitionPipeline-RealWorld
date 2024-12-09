// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_HPP_
#define ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_hardware__msg__HardwareData __attribute__((deprecated))
#else
# define DEPRECATED__robot_hardware__msg__HardwareData __declspec(deprecated)
#endif

namespace robot_hardware
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HardwareData_
{
  using Type = HardwareData_<ContainerAllocator>;

  explicit HardwareData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->imu_orientation_x = 0.0;
      this->imu_orientation_y = 0.0;
      this->imu_orientation_z = 0.0;
      this->imu_orientation_w = 0.0;
      this->imu_linear_acceleration_x = 0.0;
      this->imu_linear_acceleration_y = 0.0;
      this->imu_linear_acceleration_z = 0.0;
      this->imu_angular_velocity_x = 0.0;
      this->imu_angular_velocity_y = 0.0;
      this->imu_angular_velocity_z = 0.0;
      this->magnetic_field_x = 0.0;
      this->magnetic_field_y = 0.0;
      this->magnetic_field_z = 0.0;
    }
  }

  explicit HardwareData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->imu_orientation_x = 0.0;
      this->imu_orientation_y = 0.0;
      this->imu_orientation_z = 0.0;
      this->imu_orientation_w = 0.0;
      this->imu_linear_acceleration_x = 0.0;
      this->imu_linear_acceleration_y = 0.0;
      this->imu_linear_acceleration_z = 0.0;
      this->imu_angular_velocity_x = 0.0;
      this->imu_angular_velocity_y = 0.0;
      this->imu_angular_velocity_z = 0.0;
      this->magnetic_field_x = 0.0;
      this->magnetic_field_y = 0.0;
      this->magnetic_field_z = 0.0;
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
  using _imu_orientation_x_type =
    double;
  _imu_orientation_x_type imu_orientation_x;
  using _imu_orientation_y_type =
    double;
  _imu_orientation_y_type imu_orientation_y;
  using _imu_orientation_z_type =
    double;
  _imu_orientation_z_type imu_orientation_z;
  using _imu_orientation_w_type =
    double;
  _imu_orientation_w_type imu_orientation_w;
  using _imu_linear_acceleration_x_type =
    double;
  _imu_linear_acceleration_x_type imu_linear_acceleration_x;
  using _imu_linear_acceleration_y_type =
    double;
  _imu_linear_acceleration_y_type imu_linear_acceleration_y;
  using _imu_linear_acceleration_z_type =
    double;
  _imu_linear_acceleration_z_type imu_linear_acceleration_z;
  using _imu_angular_velocity_x_type =
    double;
  _imu_angular_velocity_x_type imu_angular_velocity_x;
  using _imu_angular_velocity_y_type =
    double;
  _imu_angular_velocity_y_type imu_angular_velocity_y;
  using _imu_angular_velocity_z_type =
    double;
  _imu_angular_velocity_z_type imu_angular_velocity_z;
  using _magnetic_field_x_type =
    double;
  _magnetic_field_x_type magnetic_field_x;
  using _magnetic_field_y_type =
    double;
  _magnetic_field_y_type magnetic_field_y;
  using _magnetic_field_z_type =
    double;
  _magnetic_field_z_type magnetic_field_z;

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
  Type & set__imu_orientation_x(
    const double & _arg)
  {
    this->imu_orientation_x = _arg;
    return *this;
  }
  Type & set__imu_orientation_y(
    const double & _arg)
  {
    this->imu_orientation_y = _arg;
    return *this;
  }
  Type & set__imu_orientation_z(
    const double & _arg)
  {
    this->imu_orientation_z = _arg;
    return *this;
  }
  Type & set__imu_orientation_w(
    const double & _arg)
  {
    this->imu_orientation_w = _arg;
    return *this;
  }
  Type & set__imu_linear_acceleration_x(
    const double & _arg)
  {
    this->imu_linear_acceleration_x = _arg;
    return *this;
  }
  Type & set__imu_linear_acceleration_y(
    const double & _arg)
  {
    this->imu_linear_acceleration_y = _arg;
    return *this;
  }
  Type & set__imu_linear_acceleration_z(
    const double & _arg)
  {
    this->imu_linear_acceleration_z = _arg;
    return *this;
  }
  Type & set__imu_angular_velocity_x(
    const double & _arg)
  {
    this->imu_angular_velocity_x = _arg;
    return *this;
  }
  Type & set__imu_angular_velocity_y(
    const double & _arg)
  {
    this->imu_angular_velocity_y = _arg;
    return *this;
  }
  Type & set__imu_angular_velocity_z(
    const double & _arg)
  {
    this->imu_angular_velocity_z = _arg;
    return *this;
  }
  Type & set__magnetic_field_x(
    const double & _arg)
  {
    this->magnetic_field_x = _arg;
    return *this;
  }
  Type & set__magnetic_field_y(
    const double & _arg)
  {
    this->magnetic_field_y = _arg;
    return *this;
  }
  Type & set__magnetic_field_z(
    const double & _arg)
  {
    this->magnetic_field_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_hardware::msg::HardwareData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_hardware::msg::HardwareData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_hardware::msg::HardwareData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_hardware::msg::HardwareData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_hardware__msg__HardwareData
    std::shared_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_hardware__msg__HardwareData
    std::shared_ptr<robot_hardware::msg::HardwareData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HardwareData_ & other) const
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
    if (this->imu_orientation_x != other.imu_orientation_x) {
      return false;
    }
    if (this->imu_orientation_y != other.imu_orientation_y) {
      return false;
    }
    if (this->imu_orientation_z != other.imu_orientation_z) {
      return false;
    }
    if (this->imu_orientation_w != other.imu_orientation_w) {
      return false;
    }
    if (this->imu_linear_acceleration_x != other.imu_linear_acceleration_x) {
      return false;
    }
    if (this->imu_linear_acceleration_y != other.imu_linear_acceleration_y) {
      return false;
    }
    if (this->imu_linear_acceleration_z != other.imu_linear_acceleration_z) {
      return false;
    }
    if (this->imu_angular_velocity_x != other.imu_angular_velocity_x) {
      return false;
    }
    if (this->imu_angular_velocity_y != other.imu_angular_velocity_y) {
      return false;
    }
    if (this->imu_angular_velocity_z != other.imu_angular_velocity_z) {
      return false;
    }
    if (this->magnetic_field_x != other.magnetic_field_x) {
      return false;
    }
    if (this->magnetic_field_y != other.magnetic_field_y) {
      return false;
    }
    if (this->magnetic_field_z != other.magnetic_field_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const HardwareData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HardwareData_

// alias to use template instance with default allocator
using HardwareData =
  robot_hardware::msg::HardwareData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_HPP_
