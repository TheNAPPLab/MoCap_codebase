// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/LogicalCameraImage.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'model'
#include "ros_gz_interfaces/msg/detail/logical_camera_image_model__struct.h"

/// Struct defined in msg/LogicalCameraImage in the package ros_gz_interfaces.
/**
  * Optional header data.
 */
typedef struct ros_gz_interfaces__msg__LogicalCameraImage
{
  std_msgs__msg__Header header;
  /// Logical camera pose
  geometry_msgs__msg__Pose pose;
  /// Detected models
  ros_gz_interfaces__msg__LogicalCameraImageModel__Sequence model;
} ros_gz_interfaces__msg__LogicalCameraImage;

// Struct for a sequence of ros_gz_interfaces__msg__LogicalCameraImage.
typedef struct ros_gz_interfaces__msg__LogicalCameraImage__Sequence
{
  ros_gz_interfaces__msg__LogicalCameraImage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__LogicalCameraImage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__STRUCT_H_
