// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "teb_msgs/msg/detail/feedback_msg__rosidl_typesupport_introspection_c.h"
#include "teb_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "teb_msgs/msg/detail/feedback_msg__functions.h"
#include "teb_msgs/msg/detail/feedback_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `trajectories`
#include "teb_msgs/msg/trajectory_msg.h"
// Member `trajectories`
#include "teb_msgs/msg/detail/trajectory_msg__rosidl_typesupport_introspection_c.h"
// Member `obstacles_msg`
#include "costmap_converter_msgs/msg/obstacle_array_msg.h"
// Member `obstacles_msg`
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teb_msgs__msg__FeedbackMsg__init(message_memory);
}

void FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_fini_function(void * message_memory)
{
  teb_msgs__msg__FeedbackMsg__fini(message_memory);
}

size_t FeedbackMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryMsg__trajectories(
  const void * untyped_member)
{
  const teb_msgs__msg__TrajectoryMsg__Sequence * member =
    (const teb_msgs__msg__TrajectoryMsg__Sequence *)(untyped_member);
  return member->size;
}

const void * FeedbackMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryMsg__trajectories(
  const void * untyped_member, size_t index)
{
  const teb_msgs__msg__TrajectoryMsg__Sequence * member =
    (const teb_msgs__msg__TrajectoryMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

void * FeedbackMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryMsg__trajectories(
  void * untyped_member, size_t index)
{
  teb_msgs__msg__TrajectoryMsg__Sequence * member =
    (teb_msgs__msg__TrajectoryMsg__Sequence *)(untyped_member);
  return &member->data[index];
}

bool FeedbackMsg__rosidl_typesupport_introspection_c__resize_function__TrajectoryMsg__trajectories(
  void * untyped_member, size_t size)
{
  teb_msgs__msg__TrajectoryMsg__Sequence * member =
    (teb_msgs__msg__TrajectoryMsg__Sequence *)(untyped_member);
  teb_msgs__msg__TrajectoryMsg__Sequence__fini(member);
  return teb_msgs__msg__TrajectoryMsg__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__FeedbackMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectories",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__FeedbackMsg, trajectories),  // bytes offset in struct
    NULL,  // default value
    FeedbackMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryMsg__trajectories,  // size() function pointer
    FeedbackMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryMsg__trajectories,  // get_const(index) function pointer
    FeedbackMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryMsg__trajectories,  // get(index) function pointer
    FeedbackMsg__rosidl_typesupport_introspection_c__resize_function__TrajectoryMsg__trajectories  // resize(index) function pointer
  },
  {
    "selected_trajectory_idx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__FeedbackMsg, selected_trajectory_idx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacles_msg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__FeedbackMsg, obstacles_msg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_members = {
  "teb_msgs__msg",  // message namespace
  "FeedbackMsg",  // message name
  4,  // number of fields
  sizeof(teb_msgs__msg__FeedbackMsg),
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_member_array,  // message members
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_type_support_handle = {
  0,
  &FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teb_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teb_msgs, msg, FeedbackMsg)() {
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teb_msgs, msg, TrajectoryMsg)();
  FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, costmap_converter_msgs, msg, ObstacleArrayMsg)();
  if (!FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_type_support_handle.typesupport_identifier) {
    FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &FeedbackMsg__rosidl_typesupport_introspection_c__FeedbackMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
