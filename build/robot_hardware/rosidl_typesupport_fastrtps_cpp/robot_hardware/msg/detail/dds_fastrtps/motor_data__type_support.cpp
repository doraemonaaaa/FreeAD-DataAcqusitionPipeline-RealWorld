// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from robot_hardware:msg/MotorData.idl
// generated code does not contain a copyright notice
#include "robot_hardware/msg/detail/motor_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "robot_hardware/msg/detail/motor_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace robot_hardware
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
cdr_serialize(
  const robot_hardware::msg::MotorData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: real_left_speed_rpm
  cdr << ros_message.real_left_speed_rpm;
  // Member: real_right_speed_rpm
  cdr << ros_message.real_right_speed_rpm;
  // Member: real_left_neg_flag
  cdr << ros_message.real_left_neg_flag;
  // Member: real_right_neg_flag
  cdr << ros_message.real_right_neg_flag;
  // Member: expect_left_speed_rpm
  cdr << ros_message.expect_left_speed_rpm;
  // Member: expect_right_speed_rpm
  cdr << ros_message.expect_right_speed_rpm;
  // Member: expect_left_neg_flag
  cdr << ros_message.expect_left_neg_flag;
  // Member: expect_right_neg_flag
  cdr << ros_message.expect_right_neg_flag;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_hardware::msg::MotorData & ros_message)
{
  // Member: real_left_speed_rpm
  cdr >> ros_message.real_left_speed_rpm;

  // Member: real_right_speed_rpm
  cdr >> ros_message.real_right_speed_rpm;

  // Member: real_left_neg_flag
  cdr >> ros_message.real_left_neg_flag;

  // Member: real_right_neg_flag
  cdr >> ros_message.real_right_neg_flag;

  // Member: expect_left_speed_rpm
  cdr >> ros_message.expect_left_speed_rpm;

  // Member: expect_right_speed_rpm
  cdr >> ros_message.expect_right_speed_rpm;

  // Member: expect_left_neg_flag
  cdr >> ros_message.expect_left_neg_flag;

  // Member: expect_right_neg_flag
  cdr >> ros_message.expect_right_neg_flag;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
get_serialized_size(
  const robot_hardware::msg::MotorData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: real_left_speed_rpm
  {
    size_t item_size = sizeof(ros_message.real_left_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: real_right_speed_rpm
  {
    size_t item_size = sizeof(ros_message.real_right_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: real_left_neg_flag
  {
    size_t item_size = sizeof(ros_message.real_left_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: real_right_neg_flag
  {
    size_t item_size = sizeof(ros_message.real_right_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: expect_left_speed_rpm
  {
    size_t item_size = sizeof(ros_message.expect_left_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: expect_right_speed_rpm
  {
    size_t item_size = sizeof(ros_message.expect_right_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: expect_left_neg_flag
  {
    size_t item_size = sizeof(ros_message.expect_left_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: expect_right_neg_flag
  {
    size_t item_size = sizeof(ros_message.expect_right_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
max_serialized_size_MotorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: real_left_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: real_right_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: real_left_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: real_right_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: expect_left_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: expect_right_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: expect_left_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: expect_right_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_hardware::msg::MotorData;
    is_plain =
      (
      offsetof(DataType, expect_right_neg_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MotorData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const robot_hardware::msg::MotorData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MotorData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<robot_hardware::msg::MotorData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MotorData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const robot_hardware::msg::MotorData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MotorData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MotorData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MotorData__callbacks = {
  "robot_hardware::msg",
  "MotorData",
  _MotorData__cdr_serialize,
  _MotorData__cdr_deserialize,
  _MotorData__get_serialized_size,
  _MotorData__max_serialized_size
};

static rosidl_message_type_support_t _MotorData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MotorData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robot_hardware

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_robot_hardware
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_hardware::msg::MotorData>()
{
  return &robot_hardware::msg::typesupport_fastrtps_cpp::_MotorData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_hardware, msg, MotorData)() {
  return &robot_hardware::msg::typesupport_fastrtps_cpp::_MotorData__handle;
}

#ifdef __cplusplus
}
#endif
