// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_HPP_
#define FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__fast_lio__msg__Pose6D __attribute__((deprecated))
#else
# define DEPRECATED__fast_lio__msg__Pose6D __declspec(deprecated)
#endif

namespace fast_lio
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose6D_
{
  using Type = Pose6D_<ContainerAllocator>;

  explicit Pose6D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offset_time = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyr.begin(), this->gyr.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->vel.begin(), this->vel.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->pos.begin(), this->pos.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->rot.begin(), this->rot.end(), 0.0);
    }
  }

  explicit Pose6D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : acc(_alloc),
    gyr(_alloc),
    vel(_alloc),
    pos(_alloc),
    rot(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offset_time = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyr.begin(), this->gyr.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->vel.begin(), this->vel.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->pos.begin(), this->pos.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->rot.begin(), this->rot.end(), 0.0);
    }
  }

  // field types and members
  using _offset_time_type =
    double;
  _offset_time_type offset_time;
  using _acc_type =
    std::array<double, 3>;
  _acc_type acc;
  using _gyr_type =
    std::array<double, 3>;
  _gyr_type gyr;
  using _vel_type =
    std::array<double, 3>;
  _vel_type vel;
  using _pos_type =
    std::array<double, 3>;
  _pos_type pos;
  using _rot_type =
    std::array<double, 9>;
  _rot_type rot;

  // setters for named parameter idiom
  Type & set__offset_time(
    const double & _arg)
  {
    this->offset_time = _arg;
    return *this;
  }
  Type & set__acc(
    const std::array<double, 3> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__gyr(
    const std::array<double, 3> & _arg)
  {
    this->gyr = _arg;
    return *this;
  }
  Type & set__vel(
    const std::array<double, 3> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__pos(
    const std::array<double, 3> & _arg)
  {
    this->pos = _arg;
    return *this;
  }
  Type & set__rot(
    const std::array<double, 9> & _arg)
  {
    this->rot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fast_lio::msg::Pose6D_<ContainerAllocator> *;
  using ConstRawPtr =
    const fast_lio::msg::Pose6D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fast_lio::msg::Pose6D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fast_lio::msg::Pose6D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fast_lio::msg::Pose6D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fast_lio::msg::Pose6D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fast_lio::msg::Pose6D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fast_lio::msg::Pose6D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fast_lio::msg::Pose6D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fast_lio::msg::Pose6D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fast_lio__msg__Pose6D
    std::shared_ptr<fast_lio::msg::Pose6D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fast_lio__msg__Pose6D
    std::shared_ptr<fast_lio::msg::Pose6D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose6D_ & other) const
  {
    if (this->offset_time != other.offset_time) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->gyr != other.gyr) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->pos != other.pos) {
      return false;
    }
    if (this->rot != other.rot) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose6D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose6D_

// alias to use template instance with default allocator
using Pose6D =
  fast_lio::msg::Pose6D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace fast_lio

#endif  // FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_HPP_
