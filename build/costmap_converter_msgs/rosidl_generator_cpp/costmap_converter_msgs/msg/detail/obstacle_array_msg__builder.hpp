// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_

#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace costmap_converter_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleArrayMsg_obstacles
{
public:
  explicit Init_ObstacleArrayMsg_obstacles(::costmap_converter_msgs::msg::ObstacleArrayMsg & msg)
  : msg_(msg)
  {}
  ::costmap_converter_msgs::msg::ObstacleArrayMsg obstacles(::costmap_converter_msgs::msg::ObstacleArrayMsg::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleArrayMsg msg_;
};

class Init_ObstacleArrayMsg_header
{
public:
  Init_ObstacleArrayMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleArrayMsg_obstacles header(::costmap_converter_msgs::msg::ObstacleArrayMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleArrayMsg_obstacles(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleArrayMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::costmap_converter_msgs::msg::ObstacleArrayMsg>()
{
  return costmap_converter_msgs::msg::builder::Init_ObstacleArrayMsg_header();
}

}  // namespace costmap_converter_msgs

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__BUILDER_HPP_
