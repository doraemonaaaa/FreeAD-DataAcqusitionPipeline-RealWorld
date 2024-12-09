// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice
#include "robot_hardware/msg/detail/hardware_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "robot_hardware/msg/detail/hardware_data__struct.hpp"

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
  const robot_hardware::msg::HardwareData & ros_message,
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
  // Member: imu_orientation_x
  cdr << ros_message.imu_orientation_x;
  // Member: imu_orientation_y
  cdr << ros_message.imu_orientation_y;
  // Member: imu_orientation_z
  cdr << ros_message.imu_orientation_z;
  // Member: imu_orientation_w
  cdr << ros_message.imu_orientation_w;
  // Member: imu_linear_acceleration_x
  cdr << ros_message.imu_linear_acceleration_x;
  // Member: imu_linear_acceleration_y
  cdr << ros_message.imu_linear_acceleration_y;
  // Member: imu_linear_acceleration_z
  cdr << ros_message.imu_linear_acceleration_z;
  // Member: imu_angular_velocity_x
  cdr << ros_message.imu_angular_velocity_x;
  // Member: imu_angular_velocity_y
  cdr << ros_message.imu_angular_velocity_y;
  // Member: imu_angular_velocity_z
  cdr << ros_message.imu_angular_velocity_z;
  // Member: magnetic_field_x
  cdr << ros_message.magnetic_field_x;
  // Member: magnetic_field_y
  cdr << ros_message.magnetic_field_y;
  // Member: magnetic_field_z
  cdr << ros_message.magnetic_field_z;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_hardware::msg::HardwareData & ros_message)
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

  // Member: imu_orientation_x
  cdr >> ros_message.imu_orientation_x;

  // Member: imu_orientation_y
  cdr >> ros_message.imu_orientation_y;

  // Member: imu_orientation_z
  cdr >> ros_message.imu_orientation_z;

  // Member: imu_orientation_w
  cdr >> ros_message.imu_orientation_w;

  // Member: imu_linear_acceleration_x
  cdr >> ros_message.imu_linear_acceleration_x;

  // Member: imu_linear_acceleration_y
  cdr >> ros_message.imu_linear_acceleration_y;

  // Member: imu_linear_acceleration_z
  cdr >> ros_message.imu_linear_acceleration_z;

  // Member: imu_angular_velocity_x
  cdr >> ros_message.imu_angular_velocity_x;

  // Member: imu_angular_velocity_y
  cdr >> ros_message.imu_angular_velocity_y;

  // Member: imu_angular_velocity_z
  cdr >> ros_message.imu_angular_velocity_z;

  // Member: magnetic_field_x
  cdr >> ros_message.magnetic_field_x;

  // Member: magnetic_field_y
  cdr >> ros_message.magnetic_field_y;

  // Member: magnetic_field_z
  cdr >> ros_message.magnetic_field_z;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
get_serialized_size(
  const robot_hardware::msg::HardwareData & ros_message,
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
  // Member: imu_orientation_x
  {
    size_t item_size = sizeof(ros_message.imu_orientation_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_orientation_y
  {
    size_t item_size = sizeof(ros_message.imu_orientation_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_orientation_z
  {
    size_t item_size = sizeof(ros_message.imu_orientation_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_orientation_w
  {
    size_t item_size = sizeof(ros_message.imu_orientation_w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_linear_acceleration_x
  {
    size_t item_size = sizeof(ros_message.imu_linear_acceleration_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_linear_acceleration_y
  {
    size_t item_size = sizeof(ros_message.imu_linear_acceleration_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_linear_acceleration_z
  {
    size_t item_size = sizeof(ros_message.imu_linear_acceleration_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_angular_velocity_x
  {
    size_t item_size = sizeof(ros_message.imu_angular_velocity_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_angular_velocity_y
  {
    size_t item_size = sizeof(ros_message.imu_angular_velocity_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu_angular_velocity_z
  {
    size_t item_size = sizeof(ros_message.imu_angular_velocity_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: magnetic_field_x
  {
    size_t item_size = sizeof(ros_message.magnetic_field_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: magnetic_field_y
  {
    size_t item_size = sizeof(ros_message.magnetic_field_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: magnetic_field_z
  {
    size_t item_size = sizeof(ros_message.magnetic_field_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_hardware
max_serialized_size_HardwareData(
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

  // Member: imu_orientation_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_orientation_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_orientation_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_orientation_w
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_linear_acceleration_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_linear_acceleration_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_linear_acceleration_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_angular_velocity_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_angular_velocity_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: imu_angular_velocity_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: magnetic_field_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: magnetic_field_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: magnetic_field_z
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
    using DataType = robot_hardware::msg::HardwareData;
    is_plain =
      (
      offsetof(DataType, magnetic_field_z) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _HardwareData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const robot_hardware::msg::HardwareData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _HardwareData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<robot_hardware::msg::HardwareData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _HardwareData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const robot_hardware::msg::HardwareData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _HardwareData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_HardwareData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _HardwareData__callbacks = {
  "robot_hardware::msg",
  "HardwareData",
  _HardwareData__cdr_serialize,
  _HardwareData__cdr_deserialize,
  _HardwareData__get_serialized_size,
  _HardwareData__max_serialized_size
};

static rosidl_message_type_support_t _HardwareData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_HardwareData__callbacks,
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
get_message_type_support_handle<robot_hardware::msg::HardwareData>()
{
  return &robot_hardware::msg::typesupport_fastrtps_cpp::_HardwareData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_hardware, msg, HardwareData)() {
  return &robot_hardware::msg::typesupport_fastrtps_cpp::_HardwareData__handle;
}

#ifdef __cplusplus
}
#endif
