// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace costmap_converter_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ObstacleArrayMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) costmap_converter_msgs::msg::ObstacleArrayMsg(_init);
}

void ObstacleArrayMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<costmap_converter_msgs::msg::ObstacleArrayMsg *>(message_memory);
  typed_message->~ObstacleArrayMsg();
}

size_t size_function__ObstacleArrayMsg__obstacles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<costmap_converter_msgs::msg::ObstacleMsg> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObstacleArrayMsg__obstacles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<costmap_converter_msgs::msg::ObstacleMsg> *>(untyped_member);
  return &member[index];
}

void * get_function__ObstacleArrayMsg__obstacles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<costmap_converter_msgs::msg::ObstacleMsg> *>(untyped_member);
  return &member[index];
}

void resize_function__ObstacleArrayMsg__obstacles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<costmap_converter_msgs::msg::ObstacleMsg> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObstacleArrayMsg_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs::msg::ObstacleArrayMsg, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<costmap_converter_msgs::msg::ObstacleMsg>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs::msg::ObstacleArrayMsg, obstacles),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObstacleArrayMsg__obstacles,  // size() function pointer
    get_const_function__ObstacleArrayMsg__obstacles,  // get_const(index) function pointer
    get_function__ObstacleArrayMsg__obstacles,  // get(index) function pointer
    resize_function__ObstacleArrayMsg__obstacles  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObstacleArrayMsg_message_members = {
  "costmap_converter_msgs::msg",  // message namespace
  "ObstacleArrayMsg",  // message name
  2,  // number of fields
  sizeof(costmap_converter_msgs::msg::ObstacleArrayMsg),
  ObstacleArrayMsg_message_member_array,  // message members
  ObstacleArrayMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleArrayMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObstacleArrayMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObstacleArrayMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace costmap_converter_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<costmap_converter_msgs::msg::ObstacleArrayMsg>()
{
  return &::costmap_converter_msgs::msg::rosidl_typesupport_introspection_cpp::ObstacleArrayMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, costmap_converter_msgs, msg, ObstacleArrayMsg)() {
  return &::costmap_converter_msgs::msg::rosidl_typesupport_introspection_cpp::ObstacleArrayMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
