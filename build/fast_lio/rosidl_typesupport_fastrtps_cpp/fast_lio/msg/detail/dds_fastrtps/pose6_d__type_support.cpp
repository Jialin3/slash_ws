// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice
#include "fast_lio/msg/detail/pose6_d__rosidl_typesupport_fastrtps_cpp.hpp"
#include "fast_lio/msg/detail/pose6_d__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace fast_lio
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fast_lio
cdr_serialize(
  const fast_lio::msg::Pose6D & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: offset_time
  cdr << ros_message.offset_time;
  // Member: acc
  {
    cdr << ros_message.acc;
  }
  // Member: gyr
  {
    cdr << ros_message.gyr;
  }
  // Member: vel
  {
    cdr << ros_message.vel;
  }
  // Member: pos
  {
    cdr << ros_message.pos;
  }
  // Member: rot
  {
    cdr << ros_message.rot;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fast_lio
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  fast_lio::msg::Pose6D & ros_message)
{
  // Member: offset_time
  cdr >> ros_message.offset_time;

  // Member: acc
  {
    cdr >> ros_message.acc;
  }

  // Member: gyr
  {
    cdr >> ros_message.gyr;
  }

  // Member: vel
  {
    cdr >> ros_message.vel;
  }

  // Member: pos
  {
    cdr >> ros_message.pos;
  }

  // Member: rot
  {
    cdr >> ros_message.rot;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fast_lio
get_serialized_size(
  const fast_lio::msg::Pose6D & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: offset_time
  {
    size_t item_size = sizeof(ros_message.offset_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acc
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.acc[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gyr
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.gyr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vel
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.vel[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pos
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rot
  {
    size_t array_size = 9;
    size_t item_size = sizeof(ros_message.rot[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fast_lio
max_serialized_size_Pose6D(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: offset_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: acc
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: gyr
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: vel
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: pos
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rot
  {
    size_t array_size = 9;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Pose6D__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const fast_lio::msg::Pose6D *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Pose6D__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<fast_lio::msg::Pose6D *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Pose6D__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const fast_lio::msg::Pose6D *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Pose6D__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Pose6D(full_bounded, 0);
}

static message_type_support_callbacks_t _Pose6D__callbacks = {
  "fast_lio::msg",
  "Pose6D",
  _Pose6D__cdr_serialize,
  _Pose6D__cdr_deserialize,
  _Pose6D__get_serialized_size,
  _Pose6D__max_serialized_size
};

static rosidl_message_type_support_t _Pose6D__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Pose6D__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace fast_lio

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_fast_lio
const rosidl_message_type_support_t *
get_message_type_support_handle<fast_lio::msg::Pose6D>()
{
  return &fast_lio::msg::typesupport_fastrtps_cpp::_Pose6D__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fast_lio, msg, Pose6D)() {
  return &fast_lio::msg::typesupport_fastrtps_cpp::_Pose6D__handle;
}

#ifdef __cplusplus
}
#endif
