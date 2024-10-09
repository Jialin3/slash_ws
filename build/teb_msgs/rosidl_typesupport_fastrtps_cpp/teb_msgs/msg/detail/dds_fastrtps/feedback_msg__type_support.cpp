// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice
#include "teb_msgs/msg/detail/feedback_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "teb_msgs/msg/detail/feedback_msg__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace teb_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const teb_msgs::msg::TrajectoryMsg &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  teb_msgs::msg::TrajectoryMsg &);
size_t get_serialized_size(
  const teb_msgs::msg::TrajectoryMsg &,
  size_t current_alignment);
size_t
max_serialized_size_TrajectoryMsg(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace teb_msgs

namespace costmap_converter_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const costmap_converter_msgs::msg::ObstacleArrayMsg &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  costmap_converter_msgs::msg::ObstacleArrayMsg &);
size_t get_serialized_size(
  const costmap_converter_msgs::msg::ObstacleArrayMsg &,
  size_t current_alignment);
size_t
max_serialized_size_ObstacleArrayMsg(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace costmap_converter_msgs


namespace teb_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
cdr_serialize(
  const teb_msgs::msg::FeedbackMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: trajectories
  {
    size_t size = ros_message.trajectories.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      teb_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.trajectories[i],
        cdr);
    }
  }
  // Member: selected_trajectory_idx
  cdr << ros_message.selected_trajectory_idx;
  // Member: obstacles_msg
  costmap_converter_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.obstacles_msg,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  teb_msgs::msg::FeedbackMsg & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: trajectories
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.trajectories.resize(size);
    for (size_t i = 0; i < size; i++) {
      teb_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.trajectories[i]);
    }
  }

  // Member: selected_trajectory_idx
  cdr >> ros_message.selected_trajectory_idx;

  // Member: obstacles_msg
  costmap_converter_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.obstacles_msg);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
get_serialized_size(
  const teb_msgs::msg::FeedbackMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: trajectories
  {
    size_t array_size = ros_message.trajectories.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        teb_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.trajectories[index], current_alignment);
    }
  }
  // Member: selected_trajectory_idx
  {
    size_t item_size = sizeof(ros_message.selected_trajectory_idx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacles_msg

  current_alignment +=
    costmap_converter_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.obstacles_msg, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teb_msgs
max_serialized_size_FeedbackMsg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: trajectories
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        teb_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_TrajectoryMsg(
        full_bounded, current_alignment);
    }
  }

  // Member: selected_trajectory_idx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: obstacles_msg
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        costmap_converter_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ObstacleArrayMsg(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _FeedbackMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const teb_msgs::msg::FeedbackMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FeedbackMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<teb_msgs::msg::FeedbackMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FeedbackMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const teb_msgs::msg::FeedbackMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FeedbackMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_FeedbackMsg(full_bounded, 0);
}

static message_type_support_callbacks_t _FeedbackMsg__callbacks = {
  "teb_msgs::msg",
  "FeedbackMsg",
  _FeedbackMsg__cdr_serialize,
  _FeedbackMsg__cdr_deserialize,
  _FeedbackMsg__get_serialized_size,
  _FeedbackMsg__max_serialized_size
};

static rosidl_message_type_support_t _FeedbackMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FeedbackMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace teb_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_teb_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<teb_msgs::msg::FeedbackMsg>()
{
  return &teb_msgs::msg::typesupport_fastrtps_cpp::_FeedbackMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teb_msgs, msg, FeedbackMsg)() {
  return &teb_msgs::msg::typesupport_fastrtps_cpp::_FeedbackMsg__handle;
}

#ifdef __cplusplus
}
#endif
