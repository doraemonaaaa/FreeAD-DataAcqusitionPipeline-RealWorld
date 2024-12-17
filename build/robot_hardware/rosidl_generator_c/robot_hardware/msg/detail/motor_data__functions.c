// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_hardware:msg/MotorData.idl
// generated code does not contain a copyright notice
#include "robot_hardware/msg/detail/motor_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_hardware__msg__MotorData__init(robot_hardware__msg__MotorData * msg)
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
  return true;
}

void
robot_hardware__msg__MotorData__fini(robot_hardware__msg__MotorData * msg)
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
}

bool
robot_hardware__msg__MotorData__are_equal(const robot_hardware__msg__MotorData * lhs, const robot_hardware__msg__MotorData * rhs)
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
  return true;
}

bool
robot_hardware__msg__MotorData__copy(
  const robot_hardware__msg__MotorData * input,
  robot_hardware__msg__MotorData * output)
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
  return true;
}

robot_hardware__msg__MotorData *
robot_hardware__msg__MotorData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__MotorData * msg = (robot_hardware__msg__MotorData *)allocator.allocate(sizeof(robot_hardware__msg__MotorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_hardware__msg__MotorData));
  bool success = robot_hardware__msg__MotorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_hardware__msg__MotorData__destroy(robot_hardware__msg__MotorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_hardware__msg__MotorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_hardware__msg__MotorData__Sequence__init(robot_hardware__msg__MotorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__MotorData * data = NULL;

  if (size) {
    data = (robot_hardware__msg__MotorData *)allocator.zero_allocate(size, sizeof(robot_hardware__msg__MotorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_hardware__msg__MotorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_hardware__msg__MotorData__fini(&data[i - 1]);
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
robot_hardware__msg__MotorData__Sequence__fini(robot_hardware__msg__MotorData__Sequence * array)
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
      robot_hardware__msg__MotorData__fini(&array->data[i]);
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

robot_hardware__msg__MotorData__Sequence *
robot_hardware__msg__MotorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_hardware__msg__MotorData__Sequence * array = (robot_hardware__msg__MotorData__Sequence *)allocator.allocate(sizeof(robot_hardware__msg__MotorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_hardware__msg__MotorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_hardware__msg__MotorData__Sequence__destroy(robot_hardware__msg__MotorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_hardware__msg__MotorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_hardware__msg__MotorData__Sequence__are_equal(const robot_hardware__msg__MotorData__Sequence * lhs, const robot_hardware__msg__MotorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_hardware__msg__MotorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_hardware__msg__MotorData__Sequence__copy(
  const robot_hardware__msg__MotorData__Sequence * input,
  robot_hardware__msg__MotorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_hardware__msg__MotorData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_hardware__msg__MotorData * data =
      (robot_hardware__msg__MotorData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_hardware__msg__MotorData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_hardware__msg__MotorData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_hardware__msg__MotorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
