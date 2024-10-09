// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice
#include "teb_msgs/msg/detail/feedback_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "teb_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "teb_msgs/msg/detail/feedback_msg__struct.h"
#include "teb_msgs/msg/detail/feedback_msg__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__functions.h"  // obstacles_msg
#include "std_msgs/msg/detail/header__functions.h"  // header
#include "teb_msgs/msg/detail/trajectory_msg__functions.h"  // trajectories

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
size_t get_serialized_size_costmap_converter_msgs__msg__ObstacleArrayMsg(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
size_t max_serialized_size_costmap_converter_msgs__msg__ObstacleArrayMsg(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, costmap_converter_msgs, msg, ObstacleArrayMsg)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teb_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
size_t get_serialized_size_teb_msgs__msg__TrajectoryMsg(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_teb_msgs__msg__TrajectoryMsg(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, teb_msgs, msg, TrajectoryMsg)();


using _FeedbackMsg__ros_msg_type = teb_msgs__msg__FeedbackMsg;

static bool _FeedbackMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FeedbackMsg__ros_msg_type * ros_message = static_cast<const _FeedbackMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: trajectories
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, teb_msgs, msg, TrajectoryMsg
      )()->data);
    size_t size = ros_message->trajectories.size;
    auto array_ptr = ros_message->trajectories.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: selected_trajectory_idx
  {
    cdr << ros_message->selected_trajectory_idx;
  }

  // Field name: obstacles_msg
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, costmap_converter_msgs, msg, ObstacleArrayMsg
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->obstacles_msg, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _FeedbackMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FeedbackMsg__ros_msg_type * ros_message = static_cast<_FeedbackMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: trajectories
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, teb_msgs, msg, TrajectoryMsg
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->trajectories.data) {
      teb_msgs__msg__TrajectoryMsg__Sequence__fini(&ros_message->trajectories);
    }
    if (!teb_msgs__msg__TrajectoryMsg__Sequence__init(&ros_message->trajectories, size)) {
      return "failed to create array for field 'trajectories'";
    }
    auto array_ptr = ros_message->trajectories.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: selected_trajectory_idx
  {
    cdr >> ros_message->selected_trajectory_idx;
  }

  // Field name: obstacles_msg
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, costmap_converter_msgs, msg, ObstacleArrayMsg
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->obstacles_msg))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teb_msgs
size_t get_serialized_size_teb_msgs__msg__FeedbackMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FeedbackMsg__ros_msg_type * ros_message = static_cast<const _FeedbackMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name trajectories
  {
    size_t array_size = ros_message->trajectories.size;
    auto array_ptr = ros_message->trajectories.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_teb_msgs__msg__TrajectoryMsg(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name selected_trajectory_idx
  {
    size_t item_size = sizeof(ros_message->selected_trajectory_idx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacles_msg

  current_alignment += get_serialized_size_costmap_converter_msgs__msg__ObstacleArrayMsg(
    &(ros_message->obstacles_msg), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _FeedbackMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_teb_msgs__msg__FeedbackMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teb_msgs
size_t max_serialized_size_teb_msgs__msg__FeedbackMsg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: trajectories
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_teb_msgs__msg__TrajectoryMsg(
        full_bounded, current_alignment);
    }
  }
  // member: selected_trajectory_idx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: obstacles_msg
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_costmap_converter_msgs__msg__ObstacleArrayMsg(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _FeedbackMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_teb_msgs__msg__FeedbackMsg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_FeedbackMsg = {
  "teb_msgs::msg",
  "FeedbackMsg",
  _FeedbackMsg__cdr_serialize,
  _FeedbackMsg__cdr_deserialize,
  _FeedbackMsg__get_serialized_size,
  _FeedbackMsg__max_serialized_size
};

static rosidl_message_type_support_t _FeedbackMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FeedbackMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, teb_msgs, msg, FeedbackMsg)() {
  return &_FeedbackMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
