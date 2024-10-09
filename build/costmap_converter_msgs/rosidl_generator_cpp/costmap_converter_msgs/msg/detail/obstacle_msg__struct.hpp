// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_HPP_

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
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'velocities'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__costmap_converter_msgs__msg__ObstacleMsg __attribute__((deprecated))
#else
# define DEPRECATED__costmap_converter_msgs__msg__ObstacleMsg __declspec(deprecated)
#endif

namespace costmap_converter_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleMsg_
{
  using Type = ObstacleMsg_<ContainerAllocator>;

  explicit ObstacleMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    polygon(_init),
    orientation(_init),
    velocities(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0;
      this->id = 0ll;
    }
  }

  explicit ObstacleMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    polygon(_alloc, _init),
    orientation(_alloc, _init),
    velocities(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0;
      this->id = 0ll;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;
  using _radius_type =
    double;
  _radius_type radius;
  using _id_type =
    int64_t;
  _id_type id;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _velocities_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _velocities_type velocities;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__radius(
    const double & _arg)
  {
    this->radius = _arg;
    return *this;
  }
  Type & set__id(
    const int64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__velocities(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->velocities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__costmap_converter_msgs__msg__ObstacleMsg
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__costmap_converter_msgs__msg__ObstacleMsg
    std::shared_ptr<costmap_converter_msgs::msg::ObstacleMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->velocities != other.velocities) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleMsg_

// alias to use template instance with default allocator
using ObstacleMsg =
  costmap_converter_msgs::msg::ObstacleMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace costmap_converter_msgs

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__STRUCT_HPP_
