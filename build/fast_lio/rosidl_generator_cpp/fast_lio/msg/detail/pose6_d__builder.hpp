// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef FAST_LIO__MSG__DETAIL__POSE6_D__BUILDER_HPP_
#define FAST_LIO__MSG__DETAIL__POSE6_D__BUILDER_HPP_

#include "fast_lio/msg/detail/pose6_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace fast_lio
{

namespace msg
{

namespace builder
{

class Init_Pose6D_rot
{
public:
  explicit Init_Pose6D_rot(::fast_lio::msg::Pose6D & msg)
  : msg_(msg)
  {}
  ::fast_lio::msg::Pose6D rot(::fast_lio::msg::Pose6D::_rot_type arg)
  {
    msg_.rot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

class Init_Pose6D_pos
{
public:
  explicit Init_Pose6D_pos(::fast_lio::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_rot pos(::fast_lio::msg::Pose6D::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_Pose6D_rot(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

class Init_Pose6D_vel
{
public:
  explicit Init_Pose6D_vel(::fast_lio::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_pos vel(::fast_lio::msg::Pose6D::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_Pose6D_pos(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

class Init_Pose6D_gyr
{
public:
  explicit Init_Pose6D_gyr(::fast_lio::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_vel gyr(::fast_lio::msg::Pose6D::_gyr_type arg)
  {
    msg_.gyr = std::move(arg);
    return Init_Pose6D_vel(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

class Init_Pose6D_acc
{
public:
  explicit Init_Pose6D_acc(::fast_lio::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_gyr acc(::fast_lio::msg::Pose6D::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_Pose6D_gyr(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

class Init_Pose6D_offset_time
{
public:
  Init_Pose6D_offset_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose6D_acc offset_time(::fast_lio::msg::Pose6D::_offset_time_type arg)
  {
    msg_.offset_time = std::move(arg);
    return Init_Pose6D_acc(msg_);
  }

private:
  ::fast_lio::msg::Pose6D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::fast_lio::msg::Pose6D>()
{
  return fast_lio::msg::builder::Init_Pose6D_offset_time();
}

}  // namespace fast_lio

#endif  // FAST_LIO__MSG__DETAIL__POSE6_D__BUILDER_HPP_
