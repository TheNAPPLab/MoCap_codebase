// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/LogicalCameraImageModel.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/LogicalCameraImageModel in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__LogicalCameraImageModel
{
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Pose pose;
} ros_gz_interfaces__msg__LogicalCameraImageModel;

// Struct for a sequence of ros_gz_interfaces__msg__LogicalCameraImageModel.
typedef struct ros_gz_interfaces__msg__LogicalCameraImageModel__Sequence
{
  ros_gz_interfaces__msg__LogicalCameraImageModel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__LogicalCameraImageModel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_H_
