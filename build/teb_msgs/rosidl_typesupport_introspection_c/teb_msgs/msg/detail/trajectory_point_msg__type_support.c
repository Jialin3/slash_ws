// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "teb_msgs/msg/detail/trajectory_point_msg__rosidl_typesupport_introspection_c.h"
#include "teb_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "teb_msgs/msg/detail/trajectory_point_msg__functions.h"
#include "teb_msgs/msg/detail/trajectory_point_msg__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/twist.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/duration.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teb_msgs__msg__TrajectoryPointMsg__init(message_memory);
}

void TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_fini_function(void * message_memory)
{
  teb_msgs__msg__TrajectoryPointMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array[4] = {
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__TrajectoryPointMsg, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__TrajectoryPointMsg, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__TrajectoryPointMsg, acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_from_start",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teb_msgs__msg__TrajectoryPointMsg, time_from_start),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_members = {
  "teb_msgs__msg",  // message namespace
  "TrajectoryPointMsg",  // message name
  4,  // number of fields
  sizeof(teb_msgs__msg__TrajectoryPointMsg),
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array,  // message members
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_type_support_handle = {
  0,
  &TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teb_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teb_msgs, msg, TrajectoryPointMsg)() {
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_type_support_handle.typesupport_identifier) {
    TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrajectoryPointMsg__rosidl_typesupport_introspection_c__TrajectoryPointMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
