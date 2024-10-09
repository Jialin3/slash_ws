// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__rosidl_typesupport_introspection_c.h"
#include "costmap_converter_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__functions.h"
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `obstacles`
#include "costmap_converter_msgs/msg/obstacle_msg.h"
// Member `obstacles`
#include "costmap_converter_msgs/msg/detail/obstacle_msg__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  costmap_converter_msgs__msg__ObstacleArrayMsg__init(message_memory);
}

void ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_fini_function(void * message_memory)
{
  costmap_converter_msgs__msg__ObstacleArrayMsg__fini(message_memory);
}

size_t ObstacleArrayMsg__rosidl_typesupport_introspection_c__size_function__ObstacleMsg__obstacles(
  const void * untyped_member)
{
  const costmap_converter_msgs__msg__ObstacleMsg__Sequence * member =
    (const costmap_converter_msgs__msg__ObstacleMsg__Sequence *)(untyped_member);
  return member->size;
}

const void * ObstacleArrayMsg__rosidl_typesupport_introspection_c__get_const_function__ObstacleMsg__obstacles(
  const void * untyped_member, size_t index)
{
  const costmap_converter_msgs__msg__ObstacleMsg__Sequence * member =
    (const costmap_converter_msgs__msg__ObstacleMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ObstacleArrayMsg__rosidl_typesupport_introspection_c__get_function__ObstacleMsg__obstacles(
  void * untyped_member, size_t index)
{
  costmap_converter_msgs__msg__ObstacleMsg__Sequence * member =
    (costmap_converter_msgs__msg__ObstacleMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ObstacleArrayMsg__rosidl_typesupport_introspection_c__resize_function__ObstacleMsg__obstacles(
  void * untyped_member, size_t size)
{
  costmap_converter_msgs__msg__ObstacleMsg__Sequence * member =
    (costmap_converter_msgs__msg__ObstacleMsg__Sequence *)(untyped_member);
  costmap_converter_msgs__msg__ObstacleMsg__Sequence__fini(member);
  return costmap_converter_msgs__msg__ObstacleMsg__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleArrayMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleArrayMsg, obstacles),  // bytes offset in struct
    NULL,  // default value
    ObstacleArrayMsg__rosidl_typesupport_introspection_c__size_function__ObstacleMsg__obstacles,  // size() function pointer
    ObstacleArrayMsg__rosidl_typesupport_introspection_c__get_const_function__ObstacleMsg__obstacles,  // get_const(index) function pointer
    ObstacleArrayMsg__rosidl_typesupport_introspection_c__get_function__ObstacleMsg__obstacles,  // get(index) function pointer
    ObstacleArrayMsg__rosidl_typesupport_introspection_c__resize_function__ObstacleMsg__obstacles  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_members = {
  "costmap_converter_msgs__msg",  // message namespace
  "ObstacleArrayMsg",  // message name
  2,  // number of fields
  sizeof(costmap_converter_msgs__msg__ObstacleArrayMsg),
  ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_member_array,  // message members
  ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_type_support_handle = {
  0,
  &ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_costmap_converter_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, costmap_converter_msgs, msg, ObstacleArrayMsg)() {
  ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, costmap_converter_msgs, msg, ObstacleMsg)();
  if (!ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_type_support_handle.typesupport_identifier) {
    ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObstacleArrayMsg__rosidl_typesupport_introspection_c__ObstacleArrayMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
