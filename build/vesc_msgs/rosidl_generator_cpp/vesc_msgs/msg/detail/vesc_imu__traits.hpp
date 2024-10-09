// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_

#include "vesc_msgs/msg/detail/vesc_imu__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'ypr'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'compass'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vesc_msgs::msg::VescImu>()
{
  return "vesc_msgs::msg::VescImu";
}

template<>
inline const char * name<vesc_msgs::msg::VescImu>()
{
  return "vesc_msgs/msg/VescImu";
}

template<>
struct has_fixed_size<vesc_msgs::msg::VescImu>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<vesc_msgs::msg::VescImu>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<vesc_msgs::msg::VescImu>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU__TRAITS_HPP_
