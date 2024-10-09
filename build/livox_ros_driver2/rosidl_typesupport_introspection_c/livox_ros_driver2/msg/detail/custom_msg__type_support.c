// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "livox_ros_driver2/msg/detail/custom_msg__rosidl_typesupport_introspection_c.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "livox_ros_driver2/msg/detail/custom_msg__functions.h"
#include "livox_ros_driver2/msg/detail/custom_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "livox_ros_driver2/msg/custom_point.h"
// Member `points`
#include "livox_ros_driver2/msg/detail/custom_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  livox_ros_driver2__msg__CustomMsg__init(message_memory);
}

void CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_fini_function(void * message_memory)
{
  livox_ros_driver2__msg__CustomMsg__fini(message_memory);
}

size_t CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomPoint__points(
  const void * untyped_member)
{
  const livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (const livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomPoint__points(
  const void * untyped_member, size_t index)
{
  const livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (const livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomPoint__points(
  void * untyped_member, size_t index)
{
  livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

bool CustomMsg__rosidl_typesupport_introspection_c__resize_function__CustomPoint__points(
  void * untyped_member, size_t size)
{
  livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  livox_ros_driver2__msg__CustomPoint__Sequence__fini(member);
  return livox_ros_driver2__msg__CustomPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timebase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, timebase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, point_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lidar_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, lidar_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rsvd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, rsvd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, points),  // bytes offset in struct
    NULL,  // default value
    CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomPoint__points,  // size() function pointer
    CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomPoint__points,  // get_const(index) function pointer
    CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomPoint__points,  // get(index) function pointer
    CustomMsg__rosidl_typesupport_introspection_c__resize_function__CustomPoint__points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_members = {
  "livox_ros_driver2__msg",  // message namespace
  "CustomMsg",  // message name
  6,  // number of fields
  sizeof(livox_ros_driver2__msg__CustomMsg),
  CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array,  // message members
  CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle = {
  0,
  &CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_livox_ros_driver2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, livox_ros_driver2, msg, CustomMsg)() {
  CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, livox_ros_driver2, msg, CustomPoint)();
  if (!CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle.typesupport_identifier) {
    CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
