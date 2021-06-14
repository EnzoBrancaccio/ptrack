// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rosslt_msgs:msg\Location.idl
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
#include "rosslt_msgs/msg/detail/location__struct.h"
#include "rosslt_msgs/msg/detail/location__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rosslt_msgs__msg__location__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("rosslt_msgs.msg._location.Location", full_classname_dest, 34) == 0);
  }
  rosslt_msgs__msg__Location * ros_message = _ros_message;
  {  // source_node
    PyObject * field = PyObject_GetAttrString(_pymsg, "source_node");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->source_node, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // location_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "location_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->location_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // expression
    PyObject * field = PyObject_GetAttrString(_pymsg, "expression");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->expression, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rosslt_msgs__msg__location__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Location */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rosslt_msgs.msg._location");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Location");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rosslt_msgs__msg__Location * ros_message = (rosslt_msgs__msg__Location *)raw_ros_message;
  {  // source_node
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->source_node.data,
      strlen(ros_message->source_node.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "source_node", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
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
  {  // expression
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->expression.data,
      strlen(ros_message->expression.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "expression", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
