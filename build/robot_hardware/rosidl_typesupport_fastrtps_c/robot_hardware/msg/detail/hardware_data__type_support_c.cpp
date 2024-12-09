// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice
#include "robot_hardware/msg/detail/hardware_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_hardware/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_hardware/msg/detail/hardware_data__struct.h"
#include "robot_hardware/msg/detail/hardware_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _HardwareData__ros_msg_type = robot_hardware__msg__HardwareData;

static bool _HardwareData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HardwareData__ros_msg_type * ros_message = static_cast<const _HardwareData__ros_msg_type *>(untyped_ros_message);
  // Field name: real_left_speed_rpm
  {
    cdr << ros_message->real_left_speed_rpm;
  }

  // Field name: real_right_speed_rpm
  {
    cdr << ros_message->real_right_speed_rpm;
  }

  // Field name: real_left_neg_flag
  {
    cdr << ros_message->real_left_neg_flag;
  }

  // Field name: real_right_neg_flag
  {
    cdr << ros_message->real_right_neg_flag;
  }

  // Field name: expect_left_speed_rpm
  {
    cdr << ros_message->expect_left_speed_rpm;
  }

  // Field name: expect_right_speed_rpm
  {
    cdr << ros_message->expect_right_speed_rpm;
  }

  // Field name: expect_left_neg_flag
  {
    cdr << ros_message->expect_left_neg_flag;
  }

  // Field name: expect_right_neg_flag
  {
    cdr << ros_message->expect_right_neg_flag;
  }

  // Field name: imu_orientation_x
  {
    cdr << ros_message->imu_orientation_x;
  }

  // Field name: imu_orientation_y
  {
    cdr << ros_message->imu_orientation_y;
  }

  // Field name: imu_orientation_z
  {
    cdr << ros_message->imu_orientation_z;
  }

  // Field name: imu_orientation_w
  {
    cdr << ros_message->imu_orientation_w;
  }

  // Field name: imu_linear_acceleration_x
  {
    cdr << ros_message->imu_linear_acceleration_x;
  }

  // Field name: imu_linear_acceleration_y
  {
    cdr << ros_message->imu_linear_acceleration_y;
  }

  // Field name: imu_linear_acceleration_z
  {
    cdr << ros_message->imu_linear_acceleration_z;
  }

  // Field name: imu_angular_velocity_x
  {
    cdr << ros_message->imu_angular_velocity_x;
  }

  // Field name: imu_angular_velocity_y
  {
    cdr << ros_message->imu_angular_velocity_y;
  }

  // Field name: imu_angular_velocity_z
  {
    cdr << ros_message->imu_angular_velocity_z;
  }

  // Field name: magnetic_field_x
  {
    cdr << ros_message->magnetic_field_x;
  }

  // Field name: magnetic_field_y
  {
    cdr << ros_message->magnetic_field_y;
  }

  // Field name: magnetic_field_z
  {
    cdr << ros_message->magnetic_field_z;
  }

  return true;
}

static bool _HardwareData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HardwareData__ros_msg_type * ros_message = static_cast<_HardwareData__ros_msg_type *>(untyped_ros_message);
  // Field name: real_left_speed_rpm
  {
    cdr >> ros_message->real_left_speed_rpm;
  }

  // Field name: real_right_speed_rpm
  {
    cdr >> ros_message->real_right_speed_rpm;
  }

  // Field name: real_left_neg_flag
  {
    cdr >> ros_message->real_left_neg_flag;
  }

  // Field name: real_right_neg_flag
  {
    cdr >> ros_message->real_right_neg_flag;
  }

  // Field name: expect_left_speed_rpm
  {
    cdr >> ros_message->expect_left_speed_rpm;
  }

  // Field name: expect_right_speed_rpm
  {
    cdr >> ros_message->expect_right_speed_rpm;
  }

  // Field name: expect_left_neg_flag
  {
    cdr >> ros_message->expect_left_neg_flag;
  }

  // Field name: expect_right_neg_flag
  {
    cdr >> ros_message->expect_right_neg_flag;
  }

  // Field name: imu_orientation_x
  {
    cdr >> ros_message->imu_orientation_x;
  }

  // Field name: imu_orientation_y
  {
    cdr >> ros_message->imu_orientation_y;
  }

  // Field name: imu_orientation_z
  {
    cdr >> ros_message->imu_orientation_z;
  }

  // Field name: imu_orientation_w
  {
    cdr >> ros_message->imu_orientation_w;
  }

  // Field name: imu_linear_acceleration_x
  {
    cdr >> ros_message->imu_linear_acceleration_x;
  }

  // Field name: imu_linear_acceleration_y
  {
    cdr >> ros_message->imu_linear_acceleration_y;
  }

  // Field name: imu_linear_acceleration_z
  {
    cdr >> ros_message->imu_linear_acceleration_z;
  }

  // Field name: imu_angular_velocity_x
  {
    cdr >> ros_message->imu_angular_velocity_x;
  }

  // Field name: imu_angular_velocity_y
  {
    cdr >> ros_message->imu_angular_velocity_y;
  }

  // Field name: imu_angular_velocity_z
  {
    cdr >> ros_message->imu_angular_velocity_z;
  }

  // Field name: magnetic_field_x
  {
    cdr >> ros_message->magnetic_field_x;
  }

  // Field name: magnetic_field_y
  {
    cdr >> ros_message->magnetic_field_y;
  }

  // Field name: magnetic_field_z
  {
    cdr >> ros_message->magnetic_field_z;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_hardware
size_t get_serialized_size_robot_hardware__msg__HardwareData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HardwareData__ros_msg_type * ros_message = static_cast<const _HardwareData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name real_left_speed_rpm
  {
    size_t item_size = sizeof(ros_message->real_left_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name real_right_speed_rpm
  {
    size_t item_size = sizeof(ros_message->real_right_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name real_left_neg_flag
  {
    size_t item_size = sizeof(ros_message->real_left_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name real_right_neg_flag
  {
    size_t item_size = sizeof(ros_message->real_right_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name expect_left_speed_rpm
  {
    size_t item_size = sizeof(ros_message->expect_left_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name expect_right_speed_rpm
  {
    size_t item_size = sizeof(ros_message->expect_right_speed_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name expect_left_neg_flag
  {
    size_t item_size = sizeof(ros_message->expect_left_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name expect_right_neg_flag
  {
    size_t item_size = sizeof(ros_message->expect_right_neg_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_orientation_x
  {
    size_t item_size = sizeof(ros_message->imu_orientation_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_orientation_y
  {
    size_t item_size = sizeof(ros_message->imu_orientation_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_orientation_z
  {
    size_t item_size = sizeof(ros_message->imu_orientation_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_orientation_w
  {
    size_t item_size = sizeof(ros_message->imu_orientation_w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_linear_acceleration_x
  {
    size_t item_size = sizeof(ros_message->imu_linear_acceleration_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_linear_acceleration_y
  {
    size_t item_size = sizeof(ros_message->imu_linear_acceleration_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_linear_acceleration_z
  {
    size_t item_size = sizeof(ros_message->imu_linear_acceleration_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_angular_velocity_x
  {
    size_t item_size = sizeof(ros_message->imu_angular_velocity_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_angular_velocity_y
  {
    size_t item_size = sizeof(ros_message->imu_angular_velocity_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu_angular_velocity_z
  {
    size_t item_size = sizeof(ros_message->imu_angular_velocity_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name magnetic_field_x
  {
    size_t item_size = sizeof(ros_message->magnetic_field_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name magnetic_field_y
  {
    size_t item_size = sizeof(ros_message->magnetic_field_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name magnetic_field_z
  {
    size_t item_size = sizeof(ros_message->magnetic_field_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _HardwareData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_hardware__msg__HardwareData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_hardware
size_t max_serialized_size_robot_hardware__msg__HardwareData(
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

  // member: real_left_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: real_right_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: real_left_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: real_right_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: expect_left_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: expect_right_speed_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: expect_left_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: expect_right_neg_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: imu_orientation_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_orientation_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_orientation_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_orientation_w
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_linear_acceleration_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_linear_acceleration_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_linear_acceleration_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_angular_velocity_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_angular_velocity_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: imu_angular_velocity_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: magnetic_field_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: magnetic_field_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: magnetic_field_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_hardware__msg__HardwareData;
    is_plain =
      (
      offsetof(DataType, magnetic_field_z) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _HardwareData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_hardware__msg__HardwareData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_HardwareData = {
  "robot_hardware::msg",
  "HardwareData",
  _HardwareData__cdr_serialize,
  _HardwareData__cdr_deserialize,
  _HardwareData__get_serialized_size,
  _HardwareData__max_serialized_size
};

static rosidl_message_type_support_t _HardwareData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HardwareData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_hardware, msg, HardwareData)() {
  return &_HardwareData__type_support;
}

#if defined(__cplusplus)
}
#endif
