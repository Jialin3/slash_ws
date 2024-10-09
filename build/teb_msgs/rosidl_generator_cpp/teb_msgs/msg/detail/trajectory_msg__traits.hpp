// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from teb_msgs:msg/TrajectoryMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__TRAITS_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__TRAITS_HPP_

#include "teb_msgs/msg/detail/trajectory_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<teb_msgs::msg::TrajectoryMsg>()
{
  return "teb_msgs::msg::TrajectoryMsg";
}

template<>
inline const char * name<teb_msgs::msg::TrajectoryMsg>()
{
  return "teb_msgs/msg/TrajectoryMsg";
}

template<>
struct has_fixed_size<teb_msgs::msg::TrajectoryMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<teb_msgs::msg::TrajectoryMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<teb_msgs::msg::TrajectoryMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__TRAITS_HPP_
