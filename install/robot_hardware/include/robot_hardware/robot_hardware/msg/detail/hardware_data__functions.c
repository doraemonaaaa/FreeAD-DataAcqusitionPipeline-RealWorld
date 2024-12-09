// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice
#include "robot_hardware/msg/detail/hardware_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_hardware__msg__HardwareData__init(robot_hardware__msg__HardwareData * msg)
{
  if (!msg) {
    return false;
  }
  // real_left_speed_rpm
  // real_right_speed_rpm
  // real_left_neg_flag
  // real_right_neg_flag
  // expect_left_speed_rpm
  // expect_right_speed_rpm
  // expect_left_neg_flag
  // expect_right_neg_flag
  // imu_orientation_x
  // imu_orientation_y
  // imu_orientation_z
  // imu_orientation_w
  // imu_linear_acceleration_x
  // imu_linear_acceleration_y
  // imu_linear_acceleration_z
  // imu_angular_velocity_x
  // imu_angular_velocity_y
  // imu_angular_velocity_z
  // magnetic_field_x
  // magnetic_field_y
  // magnetic_field_z
  return true;
}

void
robot_hardware__msg__HardwareData__fini(robot_hardware__msg__HardwareData * msg)
{
  if (!msg) {
    return;
  }
  // real_left_speed_rpm
  // real_right_speed_rpm
  // real_left_neg_flag
  // real_right_neg_flag
  // expect_left_speed_rpm
  // expect_right_speed_rpm
  // expect_left_neg_flag
  // expect_right_neg_flag
  // imu_orientation_x
  // imu_orientation_y
  // imu_orientation_z
  // imu_orientation_w
  // imu_linear_acceleration_x
  // imu_linear_acceleration_y
  // imu_linear_acceleration_z
  // imu_angular_velocity_x
  // imu_angular_velocity_y
  // imu_angular_velocity_z
  // magnetic_field_x
  // magnetic_field_y
  // magnetic_field_z
}

bool
robot_hardware__msg__HardwareData__are_equal(const robot_hardware__msg__HardwareData * lhs, const robot_hardware__msg__HardwareData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // real_left_speed_rpm
  if (lhs->real_left_speed_rpm != rhs->real_left_speed_rpm) {
    return false;
  }
  // real_right_speed_rpm
  if (lhs->real_right_speed_rpm != rhs->real_right_speed_rpm) {
    return false;
  }
  // real_left_neg_flag
  if (lhs->real_left_neg_flag != rhs->real_left_neg_flag) {
    return false;
  }
  // real_right_neg_flag
  if (lhs->real_right_neg_flag != rhs->real_right_neg_flag) {
    return false;
  }
  // expect_left_speed_rpm
  if (lhs->expect_left_speed_rpm != rhs->expect_left_speed_rpm) {
    return false;
  }
  // expect_right_speed_rpm
  if (lhs->expect_right_speed_rpm != rhs->expect_right_speed_rpm) {
    return false;
  }
  // expect_left_neg_flag
  if (lhs->expect_left_neg_flag != rhs->expect_left_neg_flag) {
    return false;
  }
  // expect_right_neg_flag
  if (lhs->expect_right_neg_flag != rhs->expect_right_neg_flag) {
    return false;
  }
  // imu_orientation_x
  if (lhs->imu_orientation_x != rhs->imu_orientation_x) {
    return false;
  }
  // imu_orientation_y
  if (lhs->imu_orientation_y != rhs->imu_orientation_y) {
    return false;
  }
  // imu_orientation_z
  if (lhs->imu_orientation_z != rhs->imu_orientation_z) {
    return false;
  }
  // imu_orientation_w
  if (lhs->imu_orientation_w != rhs->imu_orientation_w) {
    return false;
  }
  // imu_linear_acceleration_x
  if (lhs->imu_linear_acceleration_x != rhs->imu_linear_acceleration_x) {
    return false;
  }
  // imu_linear_acceleration_y
  if (lhs->imu_linear_acceleration_y != rhs->imu_linear_acceleration_y) {
    return false;
  }
  // imu_linear_acceleration_z
  if (lhs->imu_linear_acceleration_z != rhs->imu_linear_acceleration_z) {
    return false;
  }
  // imu_angular_velocity_x
  if (lhs->imu_angular_velocity_x != rhs->imu_angular_velocity_x) {
    return false;
  }
  // imu_angular_velocity_y
  if (lhs->imu_angular_velocity_y != rhs->imu_angular_velocity_y) {
    return false;
  }
  // imu_angular_velocity_z
  if (lhs->imu_angular_velocity_z != rhs->imu_angular_velocity_z) {
    return false;
  }
  // magnetic_field_x
  if (lhs->magnetic_field_x != rhs->magnetic_field_x) {
    return false;
  }
  // magnetic_field_y
  if (lhs->magnetic_field_y != rhs->magnetic_field_y) {
    return false;
  }
  // magnetic_field_z
  if (lhs->magnetic_field_z != rhs->magnetic_field_z) {
    return false;
  }
  return true;
}

bool
robot_hardware__msg__HardwareData__copy(
  const robot_hardware__msg__HardwareData * input,
  robot_hardware__msg__HardwareData * output)
{
  if (!input || !output) {
    return false;
  }
  // real_left_speed_rpm
  output->real_left_speed_rpm = input->real_left_speed_rpm;
  // real_right_speed_rpm
  output->real_right_speed_rpm = input->real_right_speed_rpm;
  // real_left_neg_flag
  output->real_left_neg_flag = input->real_left_neg_flag;
  // real_right_neg_flag
  output->real_right_neg_flag = input->real_right_neg_flag;
  // expect_left_speed_rpm
  output->expect_left_speed_rpm = input->expect_left_speed_rpm;
  // expect_right_speed_rpm
  output->expect_right_speed_rpm = input->expect_right_speed_rpm;
  // expect_left_neg_flag
  output->expect_left_neg_flag = input->expect_left_neg_flag;
  // expect_right_neg_flag
  output->expect_right_neg_flag = input->expect_right_neg_flag;
  // imu_orientation_x
  output->imu_orientation_x = input->imu_orientation_x;
  // imu_orientation_y
  output->imu_orientation_y = input->imu_orientation_y;
  // imu_orientation_z
  output->imu_orientation_z = input->imu_orientation_z;
  // imu_orientation_w
  output->imu_orientation_w = input->imu_orientation_w;
  // imu_linear_acceleration_x
  output->imu_linear_acceleration_x = input->imu_linear_acceleration_x;
  // imu_linear_acceleration_y
  output->imu_linear_acceleration_y = input->imu_linear_acceleration_y;
  // imu_linear_acceleration_z
  output->imu_linear_acceleration_z = input->imu_linear_acceleration_z;
  // imu_angular_velocity_x
  output->imu_angular_velocity_x = input->imu_angular_velocity_x;
  // imu_angular_velocity_y
  output->imu_angular_velocity_y = input->imu_angular_velocity_y;
  // imu_angular_velocity_z
  output->imu_angular_velocity_z = input->imu_angular_velocity_z;
  // magnetic_field_x
  output->magnetic_field_x = input->magnetic_field_x;
  // magnetic_field_y
  output->magnetic_field_y = input->magnetic_field_y;
  // magnetic_field_z
  output->magnetic_field_z = input->magnetic_field_z;
  return true;
}

robot_hardware__msg__HardwareData *
robot_hardware__msg__HardwareData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__HardwareData * msg = (robot_hardware__msg__HardwareData *)allocator.allocate(sizeof(robot_hardware__msg__HardwareData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_hardware__msg__HardwareData));
  bool success = robot_hardware__msg__HardwareData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_hardware__msg__HardwareData__destroy(robot_hardware__msg__HardwareData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_hardware__msg__HardwareData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_hardware__msg__HardwareData__Sequence__init(robot_hardware__msg__HardwareData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__HardwareData * data = NULL;

  if (size) {
    data = (robot_hardware__msg__HardwareData *)allocator.zero_allocate(size, sizeof(robot_hardware__msg__HardwareData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_hardware__msg__HardwareData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_hardware__msg__HardwareData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_hardware__msg__HardwareData__Sequence__fini(robot_hardware__msg__HardwareData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_hardware__msg__HardwareData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_hardware__msg__HardwareData__Sequence *
robot_hardware__msg__HardwareData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__HardwareData__Sequence * array = (robot_hardware__msg__HardwareData__Sequence *)allocator.allocate(sizeof(robot_hardware__msg__HardwareData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_hardware__msg__HardwareData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_hardware__msg__HardwareData__Sequence__destroy(robot_hardware__msg__HardwareData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_hardware__msg__HardwareData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_hardware__msg__HardwareData__Sequence__are_equal(const robot_hardware__msg__HardwareData__Sequence * lhs, const robot_hardware__msg__HardwareData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_hardware__msg__HardwareData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_hardware__msg__HardwareData__Sequence__copy(
  const robot_hardware__msg__HardwareData__Sequence * input,
  robot_hardware__msg__HardwareData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_hardware__msg__HardwareData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_hardware__msg__HardwareData * data =
      (robot_hardware__msg__HardwareData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_hardware__msg__HardwareData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_hardware__msg__HardwareData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_hardware__msg__HardwareData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
