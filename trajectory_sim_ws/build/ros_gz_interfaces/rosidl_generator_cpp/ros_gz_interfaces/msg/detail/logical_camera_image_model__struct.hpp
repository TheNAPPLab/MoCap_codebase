// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/LogicalCameraImageModel.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__LogicalCameraImageModel __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__LogicalCameraImageModel __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LogicalCameraImageModel_
{
  using Type = LogicalCameraImageModel_<ContainerAllocator>;

  explicit LogicalCameraImageModel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit LogicalCameraImageModel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__LogicalCameraImageModel
    std::shared_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__LogicalCameraImageModel
    std::shared_ptr<ros_gz_interfaces::msg::LogicalCameraImageModel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LogicalCameraImageModel_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const LogicalCameraImageModel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LogicalCameraImageModel_

// alias to use template instance with default allocator
using LogicalCameraImageModel =
  ros_gz_interfaces::msg::LogicalCameraImageModel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LOGICAL_CAMERA_IMAGE_MODEL__STRUCT_HPP_
