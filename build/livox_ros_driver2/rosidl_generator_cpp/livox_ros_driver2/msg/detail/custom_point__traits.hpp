// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_
#define LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_

#include "livox_ros_driver2/msg/detail/custom_point__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<livox_ros_driver2::msg::CustomPoint>()
{
  return "livox_ros_driver2::msg::CustomPoint";
}

template<>
inline const char * name<livox_ros_driver2::msg::CustomPoint>()
{
  return "livox_ros_driver2/msg/CustomPoint";
}

template<>
struct has_fixed_size<livox_ros_driver2::msg::CustomPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<livox_ros_driver2::msg::CustomPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<livox_ros_driver2::msg::CustomPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIVOX_ROS_DRIVER2__MSG__DETAIL__CUSTOM_POINT__TRAITS_HPP_
