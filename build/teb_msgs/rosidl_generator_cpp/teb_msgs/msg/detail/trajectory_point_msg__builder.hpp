// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__BUILDER_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__BUILDER_HPP_

#include "teb_msgs/msg/detail/trajectory_point_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace teb_msgs
{

namespace msg
{

namespace builder
{

class Init_TrajectoryPointMsg_time_from_start
{
public:
  explicit Init_TrajectoryPointMsg_time_from_start(::teb_msgs::msg::TrajectoryPointMsg & msg)
  : msg_(msg)
  {}
  ::teb_msgs::msg::TrajectoryPointMsg time_from_start(::teb_msgs::msg::TrajectoryPointMsg::_time_from_start_type arg)
  {
    msg_.time_from_start = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryPointMsg msg_;
};

class Init_TrajectoryPointMsg_acceleration
{
public:
  explicit Init_TrajectoryPointMsg_acceleration(::teb_msgs::msg::TrajectoryPointMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPointMsg_time_from_start acceleration(::teb_msgs::msg::TrajectoryPointMsg::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TrajectoryPointMsg_time_from_start(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryPointMsg msg_;
};

class Init_TrajectoryPointMsg_velocity
{
public:
  explicit Init_TrajectoryPointMsg_velocity(::teb_msgs::msg::TrajectoryPointMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPointMsg_acceleration velocity(::teb_msgs::msg::TrajectoryPointMsg::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_TrajectoryPointMsg_acceleration(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryPointMsg msg_;
};

class Init_TrajectoryPointMsg_pose
{
public:
  Init_TrajectoryPointMsg_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryPointMsg_velocity pose(::teb_msgs::msg::TrajectoryPointMsg::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_TrajectoryPointMsg_velocity(msg_);
  }

private:
  ::teb_msgs::msg::TrajectoryPointMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::teb_msgs::msg::TrajectoryPointMsg>()
{
  return teb_msgs::msg::builder::Init_TrajectoryPointMsg_pose();
}

}  // namespace teb_msgs

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__BUILDER_HPP_
