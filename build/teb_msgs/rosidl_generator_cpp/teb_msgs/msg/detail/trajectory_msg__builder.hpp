// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teb_msgs:msg/TrajectoryMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__BUILDER_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__BUILDER_HPP_

#include "teb_msgs/msg/detail/trajectory_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace teb_msgs
{

namespace msg
{

namespace builder
{

class Init_TrajectoryMsg_trajectory
{
public:
  explicit Init_TrajectoryMsg_trajectory(::teb_msgs::msg::TrajectoryMsg & msg)
  : msg_(msg)
  {}
  ::teb_msgs::msg::TrajectoryMsg trajectory(::teb_msgs::msg::TrajectoryMsg::_trajectory_type arg)
  {
    msg_.trajectory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryMsg msg_;
};

class Init_TrajectoryMsg_header
{
public:
  Init_TrajectoryMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryMsg_trajectory header(::teb_msgs::msg::TrajectoryMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrajectoryMsg_trajectory(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::teb_msgs::msg::TrajectoryMsg>()
{
  return teb_msgs::msg::builder::Init_TrajectoryMsg_header();
}

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__BUILDER_HPP_
