// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_

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
// Member 'obstacles'
#include "costmap_converter_msgs/msg/detail/obstacle_msg__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__costmap_converter_msgs__msg__ObstacleArrayMsg __attribute__((deprecated))
#else
# define DEPRECATED__costmap_converter_msgs__msg__ObstacleArrayMsg __declspec(deprecated)
#endif

namespace costmap_converter_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleArrayMsg_
{
  using Type = ObstacleArrayMsg_<ContainerAllocator>;

  explicit ObstacleArrayMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObstacleArrayMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _obstacles_type =
    std::vector<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>::other>;
  _obstacles_type obstacles;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__obstacles(
    const std::vector<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>, typename ContainerAllocator::template rebind<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>::other> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__costmap_converter_msgs__msg__ObstacleArrayMsg
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__costmap_converter_msgs__msg__ObstacleArrayMsg
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleArrayMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleArrayMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->obstacles != other.obstacles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleArrayMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleArrayMsg_

// alias to use template instance with default allocator
using ObstacleArrayMsg =
  costmap_converter_msgs::msg::ObstacleArrayMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace costmap_converter_msgs

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_HPP_
