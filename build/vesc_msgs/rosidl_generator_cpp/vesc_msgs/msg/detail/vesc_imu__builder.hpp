// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU__BUILDER_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU__BUILDER_HPP_

#include "vesc_msgs/msg/detail/vesc_imu__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vesc_msgs
{

namespace msg
{

namespace builder
{

class Init_VescImu_orientation
{
public:
  explicit Init_VescImu_orientation(::vesc_msgs::msg::VescImu & msg)
  : msg_(msg)
  {}
  ::vesc_msgs::msg::VescImu orientation(::vesc_msgs::msg::VescImu::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_msgs::msg::VescImu msg_;
};

class Init_VescImu_compass
{
public:
  explicit Init_VescImu_compass(::vesc_msgs::msg::VescImu & msg)
  : msg_(msg)
  {}
  Init_VescImu_orientation compass(::vesc_msgs::msg::VescImu::_compass_type arg)
  {
    msg_.compass = std::move(arg);
    return Init_VescImu_orientation(msg_);
  }

private:
  ::vesc_msgs::msg::VescImu msg_;
};

class Init_VescImu_angular_velocity
{
public:
  explicit Init_VescImu_angular_velocity(::vesc_msgs::msg::VescImu & msg)
  : msg_(msg)
  {}
  Init_VescImu_compass angular_velocity(::vesc_msgs::msg::VescImu::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_VescImu_compass(msg_);
  }

private:
  ::vesc_msgs::msg::VescImu msg_;
};

class Init_VescImu_linear_acceleration
{
public:
  explicit Init_VescImu_linear_acceleration(::vesc_msgs::msg::VescImu & msg)
  : msg_(msg)
  {}
  Init_VescImu_angular_velocity linear_acceleration(::vesc_msgs::msg::VescImu::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_VescImu_angular_velocity(msg_);
  }

private:
  ::vesc_msgs::msg::VescImu msg_;
};

class Init_VescImu_ypr
{
public:
  Init_VescImu_ypr()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescImu_linear_acceleration ypr(::vesc_msgs::msg::VescImu::_ypr_type arg)
  {
    msg_.ypr = std::move(arg);
    return Init_VescImu_linear_acceleration(msg_);
  }

private:
  ::vesc_msgs::msg::VescImu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_msgs::msg::VescImu>()
{
  return vesc_msgs::msg::builder::Init_VescImu_ypr();
}

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU__BUILDER_HPP_
