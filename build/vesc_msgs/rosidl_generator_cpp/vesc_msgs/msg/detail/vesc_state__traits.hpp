// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_

#include "vesc_msgs/msg/detail/vesc_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vesc_msgs::msg::VescState>()
{
  return "vesc_msgs::msg::VescState";
}

template<>
inline const char * name<vesc_msgs::msg::VescState>()
{
  return "vesc_msgs/msg/VescState";
}

template<>
struct has_fixed_size<vesc_msgs::msg::VescState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vesc_msgs::msg::VescState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vesc_msgs::msg::VescState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__TRAITS_HPP_
