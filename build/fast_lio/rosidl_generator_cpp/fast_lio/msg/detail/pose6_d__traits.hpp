// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef FAST_LIO__MSG__DETAIL__POSE6_D__TRAITS_HPP_
#define FAST_LIO__MSG__DETAIL__POSE6_D__TRAITS_HPP_

#include "fast_lio/msg/detail/pose6_d__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<fast_lio::msg::Pose6D>()
{
  return "fast_lio::msg::Pose6D";
}

template<>
inline const char * name<fast_lio::msg::Pose6D>()
{
  return "fast_lio/msg/Pose6D";
}

template<>
struct has_fixed_size<fast_lio::msg::Pose6D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<fast_lio::msg::Pose6D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<fast_lio::msg::Pose6D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FAST_LIO__MSG__DETAIL__POSE6_D__TRAITS_HPP_
