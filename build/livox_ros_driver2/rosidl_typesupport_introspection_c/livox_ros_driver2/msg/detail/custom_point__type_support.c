// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "livox_ros_driver2/msg/detail/custom_point__rosidl_typesupport_introspection_c.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "livox_ros_driver2/msg/detail/custom_point__functions.h"
#include "livox_ros_driver2/msg/detail/custom_point__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  livox_ros_driver2__msg__CustomPoint__init(message_memory);
}

void CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_fini_function(void * message_memory)
{
  livox_ros_driver2__msg__CustomPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_member_array[7] = {
  {
    "offset_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, offset_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reflectivity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, reflectivity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, tag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "line",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomPoint, line),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_members = {
  "livox_ros_driver2__msg",  // message namespace
  "CustomPoint",  // message name
  7,  // number of fields
  sizeof(livox_ros_driver2__msg__CustomPoint),
  CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_member_array,  // message members
  CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_type_support_handle = {
  0,
  &CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_livox_ros_driver2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, livox_ros_driver2, msg, CustomPoint)() {
  if (!CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_type_support_handle.typesupport_identifier) {
    CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CustomPoint__rosidl_typesupport_introspection_c__CustomPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
