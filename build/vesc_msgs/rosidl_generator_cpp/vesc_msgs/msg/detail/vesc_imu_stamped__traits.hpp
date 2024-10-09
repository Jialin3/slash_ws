// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vesc_msgs:msg/VescImuStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__TRAITS_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__TRAITS_HPP_

#include "vesc_msgs/msg/detail/vesc_imu_stamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'imu'
#include "vesc_msgs/msg/detail/vesc_imu__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vesc_msgs::msg::VescImuStamped>()
{
  return "vesc_msgs::msg::VescImuStamped";
}

template<>
inline const char * name<vesc_msgs::msg::VescImuStamped>()
{
  return "vesc_msgs/msg/VescImuStamped";
}

template<>
struct has_fixed_size<vesc_msgs::msg::VescImuStamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<vesc_msgs::msg::VescImu>::value> {};

template<>
struct has_bounded_size<vesc_msgs::msg::VescImuStamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<vesc_msgs::msg::VescImu>::value> {};

template<>
struct is_message<vesc_msgs::msg::VescImuStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__TRAITS_HPP_
