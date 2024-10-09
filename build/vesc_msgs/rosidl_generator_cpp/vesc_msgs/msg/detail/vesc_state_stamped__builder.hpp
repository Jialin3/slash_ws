// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_

#include "vesc_msgs/msg/detail/vesc_state_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vesc_msgs
{

namespace msg
{

namespace builder
{

class Init_VescStateStamped_state
{
public:
  explicit Init_VescStateStamped_state(::vesc_msgs::msg::VescStateStamped & msg)
  : msg_(msg)
  {}
  ::vesc_msgs::msg::VescStateStamped state(::vesc_msgs::msg::VescStateStamped::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_msgs::msg::VescStateStamped msg_;
};

class Init_VescStateStamped_header
{
public:
  Init_VescStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescStateStamped_state header(::vesc_msgs::msg::VescStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VescStateStamped_state(msg_);
  }

private:
  ::vesc_msgs::msg::VescStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_msgs::msg::VescStateStamped>()
{
  return vesc_msgs::msg::builder::Init_VescStateStamped_header();
}

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_
