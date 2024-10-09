// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice
#include "livox_ros_driver2/msg/detail/custom_point__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "livox_ros_driver2/msg/detail/custom_point__struct.h"
#include "livox_ros_driver2/msg/detail/custom_point__functions.h"
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


// forward declare type support functions


using _CustomPoint__ros_msg_type = livox_ros_driver2__msg__CustomPoint;

static bool _CustomPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CustomPoint__ros_msg_type * ros_message = static_cast<const _CustomPoint__ros_msg_type *>(untyped_ros_message);
  // Field name: offset_time
  {
    cdr << ros_message->offset_time;
  }

  // Field name: x
  {
    cdr << ros_message->x;
  }

  // Field name: y
  {
    cdr << ros_message->y;
  }

  // Field name: z
  {
    cdr << ros_message->z;
  }

  // Field name: reflectivity
  {
    cdr << ros_message->reflectivity;
  }

  // Field name: tag
  {
    cdr << ros_message->tag;
  }

  // Field name: line
  {
    cdr << ros_message->line;
  }

  return true;
}

static bool _CustomPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CustomPoint__ros_msg_type * ros_message = static_cast<_CustomPoint__ros_msg_type *>(untyped_ros_message);
  // Field name: offset_time
  {
    cdr >> ros_message->offset_time;
  }

  // Field name: x
  {
    cdr >> ros_message->x;
  }

  // Field name: y
  {
    cdr >> ros_message->y;
  }

  // Field name: z
  {
    cdr >> ros_message->z;
  }

  // Field name: reflectivity
  {
    cdr >> ros_message->reflectivity;
  }

  // Field name: tag
  {
    cdr >> ros_message->tag;
  }

  // Field name: line
  {
    cdr >> ros_message->line;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t get_serialized_size_livox_ros_driver2__msg__CustomPoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CustomPoint__ros_msg_type * ros_message = static_cast<const _CustomPoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name offset_time
  {
    size_t item_size = sizeof(ros_message->offset_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x
  {
    size_t item_size = sizeof(ros_message->x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y
  {
    size_t item_size = sizeof(ros_message->y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z
  {
    size_t item_size = sizeof(ros_message->z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name reflectivity
  {
    size_t item_size = sizeof(ros_message->reflectivity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tag
  {
    size_t item_size = sizeof(ros_message->tag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name line
  {
    size_t item_size = sizeof(ros_message->line);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CustomPoint__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_livox_ros_driver2__msg__CustomPoint(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_livox_ros_driver2
size_t max_serialized_size_livox_ros_driver2__msg__CustomPoint(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: offset_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: z
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: reflectivity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: tag
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: line
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _CustomPoint__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_livox_ros_driver2__msg__CustomPoint(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CustomPoint = {
  "livox_ros_driver2::msg",
  "CustomPoint",
  _CustomPoint__cdr_serialize,
  _CustomPoint__cdr_deserialize,
  _CustomPoint__get_serialized_size,
  _CustomPoint__max_serialized_size
};

static rosidl_message_type_support_t _CustomPoint__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CustomPoint,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, livox_ros_driver2, msg, CustomPoint)() {
  return &_CustomPoint__type_support;
}

#if defined(__cplusplus)
}
#endif
