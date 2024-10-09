// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__TRAITS_HPP_
#define TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__TRAITS_HPP_

#include "teb_msgs/msg/detail/feedback_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'obstacles_msg'
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<teb_msgs::msg::FeedbackMsg>()
{
  return "teb_msgs::msg::FeedbackMsg";
}

template<>
inline const char * name<teb_msgs::msg::FeedbackMsg>()
{
  return "teb_msgs/msg/FeedbackMsg";
}

template<>
struct has_fixed_size<teb_msgs::msg::FeedbackMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<teb_msgs::msg::FeedbackMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<teb_msgs::msg::FeedbackMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__TRAITS_HPP_
