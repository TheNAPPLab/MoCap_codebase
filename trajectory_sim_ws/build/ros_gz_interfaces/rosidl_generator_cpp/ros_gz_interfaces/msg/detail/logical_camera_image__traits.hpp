// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/LogicalCameraImage.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/logical_camera_image__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'model'
#include "ros_gz_interfaces/msg/detail/logical_camera_image_model__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LogicalCameraImage & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: model
  {
    if (msg.model.size() == 0) {
      out << "model: []";
    } else {
      out << "model: [";
      size_t pending_items = msg.model.size();
      for (auto item : msg.model) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LogicalCameraImage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.model.size() == 0) {
      out << "model: []\n";
    } else {
      out << "model:\n";
      for (auto item : msg.model) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LogicalCameraImage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros_gz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros_gz_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_gz_interfaces::msg::LogicalCameraImage & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::LogicalCameraImage & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::LogicalCameraImage>()
{
  return "ros_gz_interfaces::msg::LogicalCameraImage";
}

template<>
inline const char * name<ros_gz_interfaces::msg::LogicalCameraImage>()
{
  return "ros_gz_interfaces/msg/LogicalCameraImage";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::LogicalCameraImage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::LogicalCameraImage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::LogicalCameraImage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__TRAITS_HPP_
