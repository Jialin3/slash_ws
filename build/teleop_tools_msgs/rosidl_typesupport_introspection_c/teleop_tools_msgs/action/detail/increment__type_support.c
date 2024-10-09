// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from teleop_tools_msgs:action/Increment.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
#include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "teleop_tools_msgs/action/detail/increment__functions.h"
#include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `increment_by`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_Goal__init(message_memory);
}

void Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_member_array[1] = {
  {
    "increment_by",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_Goal, increment_by),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_Goal",  // message name
  1,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_Goal),
  Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_member_array,  // message members
  Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_type_support_handle = {
  0,
  &Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Goal)() {
  if (!Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_type_support_handle.typesupport_identifier) {
    Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_Goal__rosidl_typesupport_introspection_c__Increment_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_Result__init(message_memory);
}

void Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_Result, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_Result",  // message name
  1,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_Result),
  Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_member_array,  // message members
  Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_type_support_handle = {
  0,
  &Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Result)() {
  if (!Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_type_support_handle.typesupport_identifier) {
    Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_Result__rosidl_typesupport_introspection_c__Increment_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_Feedback__init(message_memory);
}

void Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_Feedback",  // message name
  1,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_Feedback),
  Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_member_array,  // message members
  Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_type_support_handle = {
  0,
  &Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Feedback)() {
  if (!Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_type_support_handle.typesupport_identifier) {
    Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_Feedback__rosidl_typesupport_introspection_c__Increment_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "teleop_tools_msgs/action/increment.h"
// Member `goal`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_SendGoal_Request__init(message_memory);
}

void Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request),
  Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_member_array,  // message members
  Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_type_support_handle = {
  0,
  &Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Request)() {
  Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Goal)();
  if (!Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_SendGoal_Request__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_SendGoal_Response__init(message_memory);
}

void Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response),
  Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_member_array,  // message members
  Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_type_support_handle = {
  0,
  &Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Response)() {
  Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_SendGoal_Response__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_members = {
  "teleop_tools_msgs__action",  // service namespace
  "Increment_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_type_support_handle = {
  0,
  &teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal)() {
  if (!teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_type_support_handle.typesupport_identifier) {
    teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_SendGoal_Response)()->data;
  }

  return &teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_GetResult_Request__init(message_memory);
}

void Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_GetResult_Request),
  Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_member_array,  // message members
  Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_type_support_handle = {
  0,
  &Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Request)() {
  Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_GetResult_Request__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "teleop_tools_msgs/action/increment.h"
// Member `result`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_GetResult_Response__init(message_memory);
}

void Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_GetResult_Response),
  Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_member_array,  // message members
  Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_type_support_handle = {
  0,
  &Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Response)() {
  Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Result)();
  if (!Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_GetResult_Response__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_members = {
  "teleop_tools_msgs__action",  // service namespace
  "Increment_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_type_support_handle = {
  0,
  &teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult)() {
  if (!teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_type_support_handle.typesupport_identifier) {
    teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_GetResult_Response)()->data;
  }

  return &teleop_tools_msgs__action__detail__increment__rosidl_typesupport_introspection_c__Increment_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"
// already included above
// #include "teleop_tools_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "teleop_tools_msgs/action/increment.h"
// Member `feedback`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teleop_tools_msgs__action__Increment_FeedbackMessage__init(message_memory);
}

void Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_fini_function(void * message_memory)
{
  teleop_tools_msgs__action__Increment_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teleop_tools_msgs__action__Increment_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_members = {
  "teleop_tools_msgs__action",  // message namespace
  "Increment_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage),
  Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_member_array,  // message members
  Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_type_support_handle = {
  0,
  &Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teleop_tools_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_FeedbackMessage)() {
  Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teleop_tools_msgs, action, Increment_Feedback)();
  if (!Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Increment_FeedbackMessage__rosidl_typesupport_introspection_c__Increment_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
