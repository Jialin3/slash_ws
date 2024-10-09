// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__STRUCT_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'time_from_start'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__teb_msgs__msg__TrajectoryPointMsg __attribute__((deprecated))
#else
# define DEPRECATED__teb_msgs__msg__TrajectoryPointMsg __declspec(deprecated)
#endif

namespace teb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryPointMsg_
{
  using Type = TrajectoryPointMsg_<ContainerAllocator>;

  explicit TrajectoryPointMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    velocity(_init),
    acceleration(_init),
    time_from_start(_init)
  {
    (void)_init;
  }

  explicit TrajectoryPointMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init),
    time_from_start(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _time_from_start_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_from_start_type time_from_start;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__time_from_start(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_from_start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__teb_msgs__msg__TrajectoryPointMsg
    std::shared_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__teb_msgs__msg__TrajectoryPointMsg
    std::shared_ptr<teb_msgs::msg::TrajectoryPointMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryPointMsg_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->time_from_start != other.time_from_start) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryPointMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryPointMsg_

// alias to use template instance with default allocator
using TrajectoryPointMsg =
  teb_msgs::msg::TrajectoryPointMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__STRUCT_HPP_
