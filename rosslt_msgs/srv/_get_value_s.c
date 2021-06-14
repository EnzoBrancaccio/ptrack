// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rosslt_msgs:srv\GetValue.idl
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
#include "rosslt_msgs/srv/detail/get_value__struct.h"
#include "rosslt_msgs/srv/detail/get_value__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rosslt_msgs__srv__get_value__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("rosslt_msgs.srv._get_value.GetValue_Request", full_classname_dest, 43) == 0);
  }
  rosslt_msgs__srv__GetValue_Request * ros_message = _ros_message;
  {  // location_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "location_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->location_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rosslt_msgs__srv__get_value__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetValue_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rosslt_msgs.srv._get_value");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetValue_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rosslt_msgs__srv__GetValue_Request * ros_message = (rosslt_msgs__srv__GetValue_Request *)raw_ros_message;
  {  // location_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->location_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "location_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosslt_msgs/srv/detail/get_value__struct.h"
// already included above
// #include "rosslt_msgs/srv/detail/get_value__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rosslt_msgs__srv__get_value__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("rosslt_msgs.srv._get_value.GetValue_Response", full_classname_dest, 44) == 0);
  }
  rosslt_msgs__srv__GetValue_Response * ros_message = _ros_message;
  {  // valid_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid_id");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid_id = (Py_True == field);
    Py_DECREF(field);
  }
  {  // current_value
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_value");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->current_value, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rosslt_msgs__srv__get_value__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetValue_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rosslt_msgs.srv._get_value");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetValue_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rosslt_msgs__srv__GetValue_Response * ros_message = (rosslt_msgs__srv__GetValue_Response *)raw_ros_message;
  {  // valid_id
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid_id ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_value
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->current_value.data,
      strlen(ros_message->current_value.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_value", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
