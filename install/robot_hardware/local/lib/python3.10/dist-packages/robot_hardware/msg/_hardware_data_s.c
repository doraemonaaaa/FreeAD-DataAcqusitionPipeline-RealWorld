// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_hardware:msg/HardwareData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_hardware/msg/detail/hardware_data__struct.h"
#include "robot_hardware/msg/detail/hardware_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_hardware__msg__hardware_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_hardware.msg._hardware_data.HardwareData", full_classname_dest, 46) == 0);
  }
  robot_hardware__msg__HardwareData * ros_message = _ros_message;
  {  // real_left_speed_rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_left_speed_rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_left_speed_rpm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // real_right_speed_rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_right_speed_rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_right_speed_rpm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // real_left_neg_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_left_neg_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_left_neg_flag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // real_right_neg_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_right_neg_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_right_neg_flag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // expect_left_speed_rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_left_speed_rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_left_speed_rpm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // expect_right_speed_rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_right_speed_rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_right_speed_rpm = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // expect_left_neg_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_left_neg_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_left_neg_flag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // expect_right_neg_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_right_neg_flag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_right_neg_flag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // imu_orientation_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_orientation_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_orientation_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_orientation_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_orientation_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_orientation_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_orientation_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_orientation_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_orientation_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_orientation_w
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_orientation_w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_orientation_w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_linear_acceleration_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_linear_acceleration_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_linear_acceleration_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_linear_acceleration_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_linear_acceleration_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_linear_acceleration_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_linear_acceleration_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_linear_acceleration_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_linear_acceleration_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_angular_velocity_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_angular_velocity_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_angular_velocity_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_angular_velocity_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_angular_velocity_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_angular_velocity_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_angular_velocity_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_angular_velocity_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->imu_angular_velocity_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // magnetic_field_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "magnetic_field_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->magnetic_field_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // magnetic_field_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "magnetic_field_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->magnetic_field_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // magnetic_field_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "magnetic_field_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->magnetic_field_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_hardware__msg__hardware_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HardwareData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_hardware.msg._hardware_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HardwareData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_hardware__msg__HardwareData * ros_message = (robot_hardware__msg__HardwareData *)raw_ros_message;
  {  // real_left_speed_rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->real_left_speed_rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_left_speed_rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // real_right_speed_rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->real_right_speed_rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_right_speed_rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // real_left_neg_flag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->real_left_neg_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_left_neg_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // real_right_neg_flag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->real_right_neg_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_right_neg_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // expect_left_speed_rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->expect_left_speed_rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_left_speed_rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // expect_right_speed_rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->expect_right_speed_rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_right_speed_rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // expect_left_neg_flag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->expect_left_neg_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_left_neg_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // expect_right_neg_flag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->expect_right_neg_flag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_right_neg_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_orientation_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_orientation_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_orientation_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_orientation_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_orientation_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_orientation_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_orientation_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_orientation_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_orientation_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_orientation_w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_orientation_w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_orientation_w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_linear_acceleration_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_linear_acceleration_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_linear_acceleration_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_linear_acceleration_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_linear_acceleration_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_linear_acceleration_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_linear_acceleration_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_linear_acceleration_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_linear_acceleration_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_angular_velocity_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_angular_velocity_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_angular_velocity_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_angular_velocity_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_angular_velocity_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_angular_velocity_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_angular_velocity_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->imu_angular_velocity_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_angular_velocity_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // magnetic_field_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->magnetic_field_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "magnetic_field_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // magnetic_field_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->magnetic_field_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "magnetic_field_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // magnetic_field_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->magnetic_field_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "magnetic_field_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
