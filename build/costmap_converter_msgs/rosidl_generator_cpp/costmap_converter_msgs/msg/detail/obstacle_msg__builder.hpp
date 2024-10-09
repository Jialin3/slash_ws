// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from costmap_converter_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_

#include "costmap_converter_msgs/msg/detail/obstacle_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace costmap_converter_msgs
{

namespace msg
{

namespace builder
{

class Init_ObstacleMsg_velocities
{
public:
  explicit Init_ObstacleMsg_velocities(::costmap_converter_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  ::costmap_converter_msgs::msg::ObstacleMsg velocities(::costmap_converter_msgs::msg::ObstacleMsg::_velocities_type arg)
  {
    msg_.velocities = std::move(arg);
    return std::move(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_orientation
{
public:
  explicit Init_ObstacleMsg_orientation(::costmap_converter_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_velocities orientation(::costmap_converter_msgs::msg::ObstacleMsg::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_ObstacleMsg_velocities(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_id
{
public:
  explicit Init_ObstacleMsg_id(::costmap_converter_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_orientation id(::costmap_converter_msgs::msg::ObstacleMsg::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ObstacleMsg_orientation(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_radius
{
public:
  explicit Init_ObstacleMsg_radius(::costmap_converter_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_id radius(::costmap_converter_msgs::msg::ObstacleMsg::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_ObstacleMsg_id(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_polygon
{
public:
  explicit Init_ObstacleMsg_polygon(::costmap_converter_msgs::msg::ObstacleMsg & msg)
  : msg_(msg)
  {}
  Init_ObstacleMsg_radius polygon(::costmap_converter_msgs::msg::ObstacleMsg::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return Init_ObstacleMsg_radius(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

class Init_ObstacleMsg_header
{
public:
  Init_ObstacleMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleMsg_polygon header(::costmap_converter_msgs::msg::ObstacleMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleMsg_polygon(msg_);
  }

private:
  ::costmap_converter_msgs::msg::ObstacleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::costmap_converter_msgs::msg::ObstacleMsg>()
{
  return costmap_converter_msgs::msg::builder::Init_ObstacleMsg_header();
}

}  // namespace costmap_converter_msgs

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_MSG__BUILDER_HPP_
