// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__BUILDER_HPP_
#define TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__BUILDER_HPP_

#include "teb_msgs/msg/detail/feedback_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace teb_msgs
{

namespace msg
{

namespace builder
{

class Init_FeedbackMsg_obstacles_msg
{
public:
  explicit Init_FeedbackMsg_obstacles_msg(::teb_msgs::msg::FeedbackMsg & msg)
  : msg_(msg)
  {}
  ::teb_msgs::msg::FeedbackMsg obstacles_msg(::teb_msgs::msg::FeedbackMsg::_obstacles_msg_type arg)
  {
    msg_.obstacles_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teb_msgs::msg::FeedbackMsg msg_;
};

class Init_FeedbackMsg_selected_trajectory_idx
{
public:
  explicit Init_FeedbackMsg_selected_trajectory_idx(::teb_msgs::msg::FeedbackMsg & msg)
  : msg_(msg)
  {}
  Init_FeedbackMsg_obstacles_msg selected_trajectory_idx(::teb_msgs::msg::FeedbackMsg::_selected_trajectory_idx_type arg)
  {
    msg_.selected_trajectory_idx = std::move(arg);
    return Init_FeedbackMsg_obstacles_msg(msg_);
  }

private:
  ::teb_msgs::msg::FeedbackMsg msg_;
};

class Init_FeedbackMsg_trajectories
{
public:
  explicit Init_FeedbackMsg_trajectories(::teb_msgs::msg::FeedbackMsg & msg)
  : msg_(msg)
  {}
  Init_FeedbackMsg_selected_trajectory_idx trajectories(::teb_msgs::msg::FeedbackMsg::_trajectories_type arg)
  {
    msg_.trajectories = std::move(arg);
    return Init_FeedbackMsg_selected_trajectory_idx(msg_);
  }

private:
  ::teb_msgs::msg::FeedbackMsg msg_;
};

class Init_FeedbackMsg_header
{
public:
  Init_FeedbackMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FeedbackMsg_trajectories header(::teb_msgs::msg::FeedbackMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FeedbackMsg_trajectories(msg_);
  }

private:
  ::teb_msgs::msg::FeedbackMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::teb_msgs::msg::FeedbackMsg>()
{
  return teb_msgs::msg::builder::Init_FeedbackMsg_header();
}

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__BUILDER_HPP_
