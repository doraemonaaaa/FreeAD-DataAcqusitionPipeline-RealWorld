// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_H_
#define ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HardwareData in the package robot_hardware.
typedef struct robot_hardware__msg__HardwareData
{
  /// 左轮实际速度
  int16_t real_left_speed;
  /// 右轮实际速度
  int16_t real_right_speed;
  /// 左轮负向标志
  uint8_t real_left_neg_flag;
  /// 右轮负向标志
  uint8_t real_right_neg_flag;
  /// 左轮期望速度
  int16_t expect_left_speed;
  /// 右轮期望速度
  int16_t expect_right_speed;
  /// 左轮负向标志
  uint8_t expect_left_neg_flag;
  /// 右轮负向标志
  uint8_t expect_right_neg_flag;
} robot_hardware__msg__HardwareData;

// Struct for a sequence of robot_hardware__msg__HardwareData.
typedef struct robot_hardware__msg__HardwareData__Sequence
{
  robot_hardware__msg__HardwareData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_hardware__msg__HardwareData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_HARDWARE__MSG__DETAIL__HARDWARE_DATA__STRUCT_H_
