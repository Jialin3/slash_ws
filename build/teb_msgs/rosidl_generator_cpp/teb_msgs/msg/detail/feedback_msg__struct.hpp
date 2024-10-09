// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__STRUCT_HPP_
#define TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__STRUCT_HPP_

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
// Member 'trajectories'
#include "teb_msgs/msg/detail/trajectory_msg__struct.hpp"
// Member 'obstacles_msg'
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__teb_msgs__msg__FeedbackMsg __attribute__((deprecated))
#else
# define DEPRECATED__teb_msgs__msg__FeedbackMsg __declspec(deprecated)
#endif

namespace teb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FeedbackMsg_
{
  using Type = FeedbackMsg_<ContainerAllocator>;

  explicit FeedbackMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    obstacles_msg(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_trajectory_idx = 0;
    }
  }

  explicit FeedbackMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    obstacles_msg(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_trajectory_idx = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trajectories_type =
    std::vector<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>::other>;
  _trajectories_type trajectories;
  using _selected_trajectory_idx_type =
    uint16_t;
  _selected_trajectory_idx_type selected_trajectory_idx;
  using _obstacles_msg_type =
    costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>;
  _obstacles_msg_type obstacles_msg;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trajectories(
    const std::vector<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<teb_msgs::msg::TrajectoryMsg_<ContainerAllocator>>::other> & _arg)
  {
    this->trajectories = _arg;
    return *this;
  }
  Type & set__selected_trajectory_idx(
    const uint16_t & _arg)
  {
    this->selected_trajectory_idx = _arg;
    return *this;
  }
  Type & set__obstacles_msg(
    const costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> & _arg)
  {
    this->obstacles_msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    teb_msgs::msg::FeedbackMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const teb_msgs::msg::FeedbackMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::FeedbackMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      teb_msgs::msg::FeedbackMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__teb_msgs__msg__FeedbackMsg
    std::shared_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__teb_msgs__msg__FeedbackMsg
    std::shared_ptr<teb_msgs::msg::FeedbackMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FeedbackMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trajectories != other.trajectories) {
      return false;
    }
    if (this->selected_trajectory_idx != other.selected_trajectory_idx) {
      return false;
    }
    if (this->obstacles_msg != other.obstacles_msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const FeedbackMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FeedbackMsg_

// alias to use template instance with default allocator
using FeedbackMsg =
  teb_msgs::msg::FeedbackMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__STRUCT_HPP_
