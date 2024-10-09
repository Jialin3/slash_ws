// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "teb_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "teb_msgs/msg/detail/trajectory_point_msg__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace teb_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
cdr_serialize(
  const teb_msgs::msg::TrajectoryPointMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  teb_msgs::msg::TrajectoryPointMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
get_serialized_size(
  const teb_msgs::msg::TrajectoryPointMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
max_serialized_size_TrajectoryPointMsg(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace teb_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teb_msgs, msg, TrajectoryPointMsg)();

#ifdef __cplusplus
}
#endif

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_POINT_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
