// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/LogicalCameraImage.idl
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
#include "ros_gz_interfaces/msg/detail/logical_camera_image__struct.h"
#include "ros_gz_interfaces/msg/detail/logical_camera_image__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ros_gz_interfaces/msg/detail/logical_camera_image_model__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
bool ros_gz_interfaces__msg__logical_camera_image_model__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros_gz_interfaces__msg__logical_camera_image_model__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__logical_camera_image__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
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
    assert(strncmp("ros_gz_interfaces.msg._logical_camera_image.LogicalCameraImage", full_classname_dest, 62) == 0);
  }
  ros_gz_interfaces__msg__LogicalCameraImage * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // model
    PyObject * field = PyObject_GetAttrString(_pymsg, "model");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'model'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ros_gz_interfaces__msg__LogicalCameraImageModel__Sequence__init(&(ros_message->model), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ros_gz_interfaces__msg__LogicalCameraImageModel__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ros_gz_interfaces__msg__LogicalCameraImageModel * dest = ros_message->model.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ros_gz_interfaces__msg__logical_camera_image_model__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__logical_camera_image__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LogicalCameraImage */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._logical_camera_image");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LogicalCameraImage");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__LogicalCameraImage * ros_message = (ros_gz_interfaces__msg__LogicalCameraImage *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // model
    PyObject * field = NULL;
    size_t size = ros_message->model.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ros_gz_interfaces__msg__LogicalCameraImageModel * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->model.data[i]);
      PyObject * pyitem = ros_gz_interfaces__msg__logical_camera_image_model__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "model", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
