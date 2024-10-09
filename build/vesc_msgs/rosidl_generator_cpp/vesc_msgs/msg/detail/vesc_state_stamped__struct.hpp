// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_HPP_

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
// Member 'state'
#include "vesc_msgs/msg/detail/vesc_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vesc_msgs__msg__VescStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__vesc_msgs__msg__VescStateStamped __declspec(deprecated)
#endif

namespace vesc_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VescStateStamped_
{
  using Type = VescStateStamped_<ContainerAllocator>;

  explicit VescStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state(_init)
  {
    (void)_init;
  }

  explicit VescStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    vesc_msgs::msg::VescState_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const vesc_msgs::msg::VescState_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vesc_msgs::msg::VescStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const vesc_msgs::msg::VescStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vesc_msgs__msg__VescStateStamped
    std::shared_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vesc_msgs__msg__VescStateStamped
    std::shared_ptr<vesc_msgs::msg::VescStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VescStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const VescStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VescStateStamped_

// alias to use template instance with default allocator
using VescStateStamped =
  vesc_msgs::msg::VescStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_HPP_
