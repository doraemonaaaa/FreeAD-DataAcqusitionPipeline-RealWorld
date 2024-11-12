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
  {  // real_left_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_left_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_left_speed = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // real_right_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "real_right_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->real_right_speed = (int16_t)PyLong_AsLong(field);
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
  {  // expect_left_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_left_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_left_speed = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // expect_right_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "expect_right_speed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->expect_right_speed = (int16_t)PyLong_AsLong(field);
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
  {  // real_left_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->real_left_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_left_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // real_right_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->real_right_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "real_right_speed", field);
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
  {  // expect_left_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->expect_left_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_left_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // expect_right_speed
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->expect_right_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "expect_right_speed", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
