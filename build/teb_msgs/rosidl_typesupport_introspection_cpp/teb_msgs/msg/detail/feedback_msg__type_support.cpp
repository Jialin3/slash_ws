// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "teb_msgs/msg/detail/feedback_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace teb_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FeedbackMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) teb_msgs::msg::FeedbackMsg(_init);
}

void FeedbackMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<teb_msgs::msg::FeedbackMsg *>(message_memory);
  typed_message->~FeedbackMsg();
}

size_t size_function__FeedbackMsg__trajectories(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<teb_msgs::msg::TrajectoryMsg> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FeedbackMsg__trajectories(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<teb_msgs::msg::TrajectoryMsg> *>(untyped_member);
  return &member[index];
}

void * get_function__FeedbackMsg__trajectories(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<teb_msgs::msg::TrajectoryMsg> *>(untyped_member);
  return &member[index];
}

void resize_function__FeedbackMsg__trajectories(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<teb_msgs::msg::TrajectoryMsg> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FeedbackMsg_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs::msg::FeedbackMsg, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "trajectories",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<teb_msgs::msg::TrajectoryMsg>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs::msg::FeedbackMsg, trajectories),  // bytes offset in struct
    nullptr,  // default value
    size_function__FeedbackMsg__trajectories,  // size() function pointer
    get_const_function__FeedbackMsg__trajectories,  // get_const(index) function pointer
    get_function__FeedbackMsg__trajectories,  // get(index) function pointer
    resize_function__FeedbackMsg__trajectories  // resize(index) function pointer
  },
  {
    "selected_trajectory_idx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs::msg::FeedbackMsg, selected_trajectory_idx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "obstacles_msg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<costmap_converter_msgs::msg::ObstacleArrayMsg>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs::msg::FeedbackMsg, obstacles_msg),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FeedbackMsg_message_members = {
  "teb_msgs::msg",  // message namespace
  "FeedbackMsg",  // message name
  4,  // number of fields
  sizeof(teb_msgs::msg::FeedbackMsg),
  FeedbackMsg_message_member_array,  // message members
  FeedbackMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  FeedbackMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FeedbackMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FeedbackMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace teb_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teb_msgs::msg::FeedbackMsg>()
{
  return &::teb_msgs::msg::rosidl_typesupport_introspection_cpp::FeedbackMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teb_msgs, msg, FeedbackMsg)() {
  return &::teb_msgs::msg::rosidl_typesupport_introspection_cpp::FeedbackMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
