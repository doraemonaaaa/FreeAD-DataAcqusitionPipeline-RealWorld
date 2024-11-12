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
  // member: real_left_speed
  {
    out << "real_left_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_speed, out);
    out << ", ";
  }

  // member: real_right_speed
  {
    out << "real_right_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_speed, out);
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

  // member: expect_left_speed
  {
    out << "expect_left_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_speed, out);
    out << ", ";
  }

  // member: expect_right_speed
  {
    out << "expect_right_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_speed, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HardwareData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: real_left_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_left_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.real_left_speed, out);
    out << "\n";
  }

  // member: real_right_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real_right_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.real_right_speed, out);
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

  // member: expect_left_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_left_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_left_speed, out);
    out << "\n";
  }

  // member: expect_right_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expect_right_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.expect_right_speed, out);
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
