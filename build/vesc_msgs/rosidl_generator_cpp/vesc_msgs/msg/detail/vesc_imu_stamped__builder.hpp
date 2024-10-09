// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_msgs:msg/VescImuStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__BUILDER_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__BUILDER_HPP_

#include "vesc_msgs/msg/detail/vesc_imu_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vesc_msgs
{

namespace msg
{

namespace builder
{

class Init_VescImuStamped_imu
{
public:
  explicit Init_VescImuStamped_imu(::vesc_msgs::msg::VescImuStamped & msg)
  : msg_(msg)
  {}
  ::vesc_msgs::msg::VescImuStamped imu(::vesc_msgs::msg::VescImuStamped::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_msgs::msg::VescImuStamped msg_;
};

class Init_VescImuStamped_header
{
public:
  Init_VescImuStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescImuStamped_imu header(::vesc_msgs::msg::VescImuStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VescImuStamped_imu(msg_);
  }

private:
  ::vesc_msgs::msg::VescImuStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_msgs::msg::VescImuStamped>()
{
  return vesc_msgs::msg::builder::Init_VescImuStamped_header();
}

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__BUILDER_HPP_
