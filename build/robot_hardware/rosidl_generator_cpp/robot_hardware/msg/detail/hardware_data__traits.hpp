// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__TRAITS_HPP_
#define ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_hardware/msg/detail/hardware_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_hardware
{

namespace msg
{

inline void to_flow_style_yaml(
  const HardwareData & msg,
  std::ostream & out)
{
  out << "{";
  // member: real_left_speed_rpm
  {
    out << "real_left_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_speed_rpm, out);
    out << ", ";
  }

  // member: real_right_speed_rpm
  {
    out << "real_right_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_speed_rpm, out);
    out << ", ";
  }

  // member: real_left_neg_flag
  {
    out << "real_left_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_neg_flag, out);
    out << ", ";
  }

  // member: real_right_neg_flag
  {
    out << "real_right_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_neg_flag, out);
    out << ", ";
  }

  // member: expect_left_speed_rpm
  {
    out << "expect_left_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_speed_rpm, out);
    out << ", ";
  }

  // member: expect_right_speed_rpm
  {
    out << "expect_right_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_speed_rpm, out);
    out << ", ";
  }

  // member: expect_left_neg_flag
  {
    out << "expect_left_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_neg_flag, out);
    out << ", ";
  }

  // member: expect_right_neg_flag
  {
    out << "expect_right_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_neg_flag, out);
    out << ", ";
  }

  // member: imu_orientation_x
  {
    out << "imu_orientation_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_x, out);
    out << ", ";
  }

  // member: imu_orientation_y
  {
    out << "imu_orientation_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_y, out);
    out << ", ";
  }

  // member: imu_orientation_z
  {
    out << "imu_orientation_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_z, out);
    out << ", ";
  }

  // member: imu_orientation_w
  {
    out << "imu_orientation_w: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_w, out);
    out << ", ";
  }

  // member: imu_linear_acceleration_x
  {
    out << "imu_linear_acceleration_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_x, out);
    out << ", ";
  }

  // member: imu_linear_acceleration_y
  {
    out << "imu_linear_acceleration_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_y, out);
    out << ", ";
  }

  // member: imu_linear_acceleration_z
  {
    out << "imu_linear_acceleration_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_z, out);
    out << ", ";
  }

  // member: imu_angular_velocity_x
  {
    out << "imu_angular_velocity_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_x, out);
    out << ", ";
  }

  // member: imu_angular_velocity_y
  {
    out << "imu_angular_velocity_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_y, out);
    out << ", ";
  }

  // member: imu_angular_velocity_z
  {
    out << "imu_angular_velocity_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_z, out);
    out << ", ";
  }

  // member: magnetic_field_x
  {
    out << "magnetic_field_x: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_x, out);
    out << ", ";
  }

  // member: magnetic_field_y
  {
    out << "magnetic_field_y: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_y, out);
    out << ", ";
  }

  // member: magnetic_field_z
  {
    out << "magnetic_field_z: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HardwareData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: real_left_speed_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_left_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_speed_rpm, out);
    out << "\n";
  }

  // member: real_right_speed_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_right_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_speed_rpm, out);
    out << "\n";
  }

  // member: real_left_neg_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_left_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_neg_flag, out);
    out << "\n";
  }

  // member: real_right_neg_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_right_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_neg_flag, out);
    out << "\n";
  }

  // member: expect_left_speed_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_left_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_speed_rpm, out);
    out << "\n";
  }

  // member: expect_right_speed_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_right_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_speed_rpm, out);
    out << "\n";
  }

  // member: expect_left_neg_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_left_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_neg_flag, out);
    out << "\n";
  }

  // member: expect_right_neg_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_right_neg_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_neg_flag, out);
    out << "\n";
  }

  // member: imu_orientation_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_orientation_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_x, out);
    out << "\n";
  }

  // member: imu_orientation_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_orientation_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_y, out);
    out << "\n";
  }

  // member: imu_orientation_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_orientation_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_z, out);
    out << "\n";
  }

  // member: imu_orientation_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_orientation_w: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_orientation_w, out);
    out << "\n";
  }

  // member: imu_linear_acceleration_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_linear_acceleration_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_x, out);
    out << "\n";
  }

  // member: imu_linear_acceleration_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_linear_acceleration_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_y, out);
    out << "\n";
  }

  // member: imu_linear_acceleration_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_linear_acceleration_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_linear_acceleration_z, out);
    out << "\n";
  }

  // member: imu_angular_velocity_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_angular_velocity_x: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_x, out);
    out << "\n";
  }

  // member: imu_angular_velocity_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_angular_velocity_y: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_y, out);
    out << "\n";
  }

  // member: imu_angular_velocity_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_angular_velocity_z: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_angular_velocity_z, out);
    out << "\n";
  }

  // member: magnetic_field_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_field_x: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_x, out);
    out << "\n";
  }

  // member: magnetic_field_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_field_y: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_y, out);
    out << "\n";
  }

  // member: magnetic_field_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_field_z: ";
    rosidl_generator_traits::value_to_yaml(msg.magnetic_field_z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HardwareData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_hardware

namespace rosidl_generator_traits
{

[[deprecated("use robot_hardware::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_hardware::msg::HardwareData & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_hardware::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_hardware::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_hardware::msg::HardwareData & msg)
{
  return robot_hardware::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_hardware::msg::HardwareData>()
{
  return "robot_hardware::msg::HardwareData";
}

template<>
inline const char * name<robot_hardware::msg::HardwareData>()
{
  return "robot_hardware/msg/HardwareData";
}

template<>
struct has_fixed_size<robot_hardware::msg::HardwareData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_hardware::msg::HardwareData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_hardware::msg::HardwareData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__TRAITS_HPP_
