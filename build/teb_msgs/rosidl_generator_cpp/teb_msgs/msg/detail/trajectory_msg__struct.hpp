// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from teb_msgs:msg/TrajectoryMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'trajectory'
#include "teb_msgs/msg/detail/trajectory_point_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__teb_msgs__msg__TrajectoryMsg __attribute__((deprecated))
#else
# define DEPRECATED__teb_msgs__msg__TrajectoryMsg __declspec(deprecated)
#endif

namespace teb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryMsg_
{
  using Type = TrajectoryMsg_<ContainerAllocator>;

  explicit TrajectoryMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit TrajectoryMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trajectory_type =
    std::vector<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>::other>;
  _trajectory_type trajectory;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trajectory(
    const std::vector<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>::other> & _arg)
  {
    this->trajectory = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__teb_msgs__msg__TrajectoryMsg
    std::shared_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__teb_msgs__msg__TrajectoryMsg
    std::shared_ptr<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trajectory != other.trajectory) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryMsg_

// alias to use template instance with default allocator
using TrajectoryMsg =
  teb_msgs::msg::TrajectoryMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_HPP_
