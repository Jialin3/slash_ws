// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from costmap_converter_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "costmap_converter_msgs/msg/detail/obstacle_msg__rosidl_typesupport_introspection_c.h"
#include "costmap_converter_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "costmap_converter_msgs/msg/detail/obstacle_msg__functions.h"
#include "costmap_converter_msgs/msg/detail/obstacle_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `polygon`
#include "geometry_msgs/msg/polygon.h"
// Member `polygon`
#include "geometry_msgs/msg/detail/polygon__rosidl_typesupport_introspection_c.h"
// Member `orientation`
#include "geometry_msgs/msg/quaternion.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__rosidl_typesupport_introspection_c.h"
// Member `velocities`
#include "geometry_msgs/msg/twist_with_covariance.h"
// Member `velocities`
#include "geometry_msgs/msg/detail/twist_with_covariance__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  costmap_converter_msgs__msg__ObstacleMsg__init(message_memory);
}

void ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_fini_function(void * message_memory)
{
  costmap_converter_msgs__msg__ObstacleMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polygon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, polygon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radius",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, radius),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(costmap_converter_msgs__msg__ObstacleMsg, velocities),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_members = {
  "costmap_converter_msgs__msg",  // message namespace
  "ObstacleMsg",  // message name
  6,  // number of fields
  sizeof(costmap_converter_msgs__msg__ObstacleMsg),
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array,  // message members
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_type_support_handle = {
  0,
  &ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_costmap_converter_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, costmap_converter_msgs, msg, ObstacleMsg)() {
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Polygon)();
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Quaternion)();
  ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TwistWithCovariance)();
  if (!ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_type_support_handle.typesupport_identifier) {
    ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObstacleMsg__rosidl_typesupport_introspection_c__ObstacleMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
