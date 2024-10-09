// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fast_lio/msg/detail/pose6_d__rosidl_typesupport_introspection_c.h"
#include "fast_lio/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fast_lio/msg/detail/pose6_d__functions.h"
#include "fast_lio/msg/detail/pose6_d__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Pose6D__rosidl_typesupport_introspection_c__Pose6D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fast_lio__msg__Pose6D__init(message_memory);
}

void Pose6D__rosidl_typesupport_introspection_c__Pose6D_fini_function(void * message_memory)
{
  fast_lio__msg__Pose6D__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_member_array[6] = {
  {
    "offset_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, offset_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, gyr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(fast_lio__msg__Pose6D, rot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_members = {
  "fast_lio__msg",  // message namespace
  "Pose6D",  // message name
  6,  // number of fields
  sizeof(fast_lio__msg__Pose6D),
  Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_member_array,  // message members
  Pose6D__rosidl_typesupport_introspection_c__Pose6D_init_function,  // function to initialize message memory (memory has to be allocated)
  Pose6D__rosidl_typesupport_introspection_c__Pose6D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_type_support_handle = {
  0,
  &Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fast_lio
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fast_lio, msg, Pose6D)() {
  if (!Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_type_support_handle.typesupport_identifier) {
    Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Pose6D__rosidl_typesupport_introspection_c__Pose6D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
