// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "fast_lio/msg/detail/pose6_d__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace fast_lio
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Pose6D_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) fast_lio::msg::Pose6D(_init);
}

void Pose6D_fini_function(void * message_memory)
{
  auto typed_message = static_cast<fast_lio::msg::Pose6D *>(message_memory);
  typed_message->~Pose6D();
}

size_t size_function__Pose6D__acc(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__Pose6D__acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__Pose6D__acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__Pose6D__gyr(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__Pose6D__gyr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__Pose6D__gyr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__Pose6D__vel(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__Pose6D__vel(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__Pose6D__vel(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__Pose6D__pos(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__Pose6D__pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__Pose6D__pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__Pose6D__rot(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__Pose6D__rot(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__Pose6D__rot(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Pose6D_message_member_array[6] = {
  {
    "offset_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, offset_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pose6D__acc,  // size() function pointer
    get_const_function__Pose6D__acc,  // get_const(index) function pointer
    get_function__Pose6D__acc,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gyr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, gyr),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pose6D__gyr,  // size() function pointer
    get_const_function__Pose6D__gyr,  // get_const(index) function pointer
    get_function__Pose6D__gyr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, vel),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pose6D__vel,  // size() function pointer
    get_const_function__Pose6D__vel,  // get_const(index) function pointer
    get_function__Pose6D__vel,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pose6D__pos,  // size() function pointer
    get_const_function__Pose6D__pos,  // get_const(index) function pointer
    get_function__Pose6D__pos,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rot",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(fast_lio::msg::Pose6D, rot),  // bytes offset in struct
    nullptr,  // default value
    size_function__Pose6D__rot,  // size() function pointer
    get_const_function__Pose6D__rot,  // get_const(index) function pointer
    get_function__Pose6D__rot,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Pose6D_message_members = {
  "fast_lio::msg",  // message namespace
  "Pose6D",  // message name
  6,  // number of fields
  sizeof(fast_lio::msg::Pose6D),
  Pose6D_message_member_array,  // message members
  Pose6D_init_function,  // function to initialize message memory (memory has to be allocated)
  Pose6D_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Pose6D_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Pose6D_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace fast_lio


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<fast_lio::msg::Pose6D>()
{
  return &::fast_lio::msg::rosidl_typesupport_introspection_cpp::Pose6D_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, fast_lio, msg, Pose6D)() {
  return &::fast_lio::msg::rosidl_typesupport_introspection_cpp::Pose6D_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
