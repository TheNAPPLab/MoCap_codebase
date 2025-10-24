// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/LogicalCameraImage.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/logical_camera_image__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_LogicalCameraImage_model
{
public:
  explicit Init_LogicalCameraImage_model(::ros_gz_interfaces::msg::LogicalCameraImage & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::LogicalCameraImage model(::ros_gz_interfaces::msg::LogicalCameraImage::_model_type arg)
  {
    msg_.model = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::LogicalCameraImage msg_;
};

class Init_LogicalCameraImage_pose
{
public:
  explicit Init_LogicalCameraImage_pose(::ros_gz_interfaces::msg::LogicalCameraImage & msg)
  : msg_(msg)
  {}
  Init_LogicalCameraImage_model pose(::ros_gz_interfaces::msg::LogicalCameraImage::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_LogicalCameraImage_model(msg_);
  }

private:
  ::ros_gz_interfaces::msg::LogicalCameraImage msg_;
};

class Init_LogicalCameraImage_header
{
public:
  Init_LogicalCameraImage_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogicalCameraImage_pose header(::ros_gz_interfaces::msg::LogicalCameraImage::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LogicalCameraImage_pose(msg_);
  }

private:
  ::ros_gz_interfaces::msg::LogicalCameraImage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::LogicalCameraImage>()
{
  return ros_gz_interfaces::msg::builder::Init_LogicalCameraImage_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE__BUILDER_HPP_
