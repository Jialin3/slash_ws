// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from teleop_tools_msgs:action/Increment.idl
// generated code does not contain a copyright notice

#ifndef TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__STRUCT_H_
#define TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'increment_by'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_Goal
{
  rosidl_runtime_c__float__Sequence increment_by;
} teleop_tools_msgs__action__Increment_Goal;

// Struct for a sequence of teleop_tools_msgs__action__Increment_Goal.
typedef struct teleop_tools_msgs__action__Increment_Goal__Sequence
{
  teleop_tools_msgs__action__Increment_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_Result
{
  uint8_t structure_needs_at_least_one_member;
} teleop_tools_msgs__action__Increment_Result;

// Struct for a sequence of teleop_tools_msgs__action__Increment_Result.
typedef struct teleop_tools_msgs__action__Increment_Result__Sequence
{
  teleop_tools_msgs__action__Increment_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_Result__Sequence;


// Constants defined in the message

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} teleop_tools_msgs__action__Increment_Feedback;

// Struct for a sequence of teleop_tools_msgs__action__Increment_Feedback.
typedef struct teleop_tools_msgs__action__Increment_Feedback__Sequence
{
  teleop_tools_msgs__action__Increment_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "teleop_tools_msgs/action/detail/increment__struct.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  teleop_tools_msgs__action__Increment_Goal goal;
} teleop_tools_msgs__action__Increment_SendGoal_Request;

// Struct for a sequence of teleop_tools_msgs__action__Increment_SendGoal_Request.
typedef struct teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence
{
  teleop_tools_msgs__action__Increment_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} teleop_tools_msgs__action__Increment_SendGoal_Response;

// Struct for a sequence of teleop_tools_msgs__action__Increment_SendGoal_Response.
typedef struct teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence
{
  teleop_tools_msgs__action__Increment_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} teleop_tools_msgs__action__Increment_GetResult_Request;

// Struct for a sequence of teleop_tools_msgs__action__Increment_GetResult_Request.
typedef struct teleop_tools_msgs__action__Increment_GetResult_Request__Sequence
{
  teleop_tools_msgs__action__Increment_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_GetResult_Response
{
  int8_t status;
  teleop_tools_msgs__action__Increment_Result result;
} teleop_tools_msgs__action__Increment_GetResult_Response;

// Struct for a sequence of teleop_tools_msgs__action__Increment_GetResult_Response.
typedef struct teleop_tools_msgs__action__Increment_GetResult_Response__Sequence
{
  teleop_tools_msgs__action__Increment_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.h"

// Struct defined in action/Increment in the package teleop_tools_msgs.
typedef struct teleop_tools_msgs__action__Increment_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  teleop_tools_msgs__action__Increment_Feedback feedback;
} teleop_tools_msgs__action__Increment_FeedbackMessage;

// Struct for a sequence of teleop_tools_msgs__action__Increment_FeedbackMessage.
typedef struct teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence
{
  teleop_tools_msgs__action__Increment_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__STRUCT_H_
