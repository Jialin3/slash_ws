// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vesc_msgs:msg/VescImuStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_HPP_

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
// Member 'imu'
#include "vesc_msgs/msg/detail/vesc_imu__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vesc_msgs__msg__VescImuStamped __attribute__((deprecated))
#else
# define DEPRECATED__vesc_msgs__msg__VescImuStamped __declspec(deprecated)
#endif

namespace vesc_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VescImuStamped_
{
  using Type = VescImuStamped_<ContainerAllocator>;

  explicit VescImuStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    imu(_init)
  {
    (void)_init;
  }

  explicit VescImuStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    imu(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _imu_type =
    vesc_msgs::msg::VescImu_<ContainerAllocator>;
  _imu_type imu;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__imu(
    const vesc_msgs::msg::VescImu_<ContainerAllocator> & _arg)
  {
    this->imu = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vesc_msgs::msg::VescImuStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const vesc_msgs::msg::VescImuStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescImuStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescImuStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vesc_msgs__msg__VescImuStamped
    std::shared_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vesc_msgs__msg__VescImuStamped
    std::shared_ptr<vesc_msgs::msg::VescImuStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VescImuStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->imu != other.imu) {
      return false;
    }
    return true;
  }
  bool operator!=(const VescImuStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VescImuStamped_

// alias to use template instance with default allocator
using VescImuStamped =
  vesc_msgs::msg::VescImuStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_HPP_
