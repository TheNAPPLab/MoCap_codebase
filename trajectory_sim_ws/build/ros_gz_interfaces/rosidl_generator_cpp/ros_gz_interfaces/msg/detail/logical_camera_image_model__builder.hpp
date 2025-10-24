// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/LogicalCameraImageModel.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/logical_camera_image_model__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_LogicalCameraImageModel_pose
{
public:
  explicit Init_LogicalCameraImageModel_pose(::ros_gz_interfaces::msg::LogicalCameraImageModel & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::LogicalCameraImageModel pose(::ros_gz_interfaces::msg::LogicalCameraImageModel::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::LogicalCameraImageModel msg_;
};

class Init_LogicalCameraImageModel_name
{
public:
  Init_LogicalCameraImageModel_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogicalCameraImageModel_pose name(::ros_gz_interfaces::msg::LogicalCameraImageModel::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_LogicalCameraImageModel_pose(msg_);
  }

private:
  ::ros_gz_interfaces::msg::LogicalCameraImageModel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::LogicalCameraImageModel>()
{
  return ros_gz_interfaces::msg::builder::Init_LogicalCameraImageModel_name();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__BUILDER_HPP_
