// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vesc_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vesc_msgs/msg/detail/vesc_state__struct.h"
#include "vesc_msgs/msg/detail/vesc_state__functions.h"
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


using _VescState__ros_msg_type = vesc_msgs__msg__VescState;

static bool _VescState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VescState__ros_msg_type * ros_message = static_cast<const _VescState__ros_msg_type *>(untyped_ros_message);
  // Field name: temp_fet
  {
    cdr << ros_message->temp_fet;
  }

  // Field name: temp_motor
  {
    cdr << ros_message->temp_motor;
  }

  // Field name: current_motor
  {
    cdr << ros_message->current_motor;
  }

  // Field name: current_input
  {
    cdr << ros_message->current_input;
  }

  // Field name: avg_id
  {
    cdr << ros_message->avg_id;
  }

  // Field name: avg_iq
  {
    cdr << ros_message->avg_iq;
  }

  // Field name: duty_cycle
  {
    cdr << ros_message->duty_cycle;
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: voltage_input
  {
    cdr << ros_message->voltage_input;
  }

  // Field name: charge_drawn
  {
    cdr << ros_message->charge_drawn;
  }

  // Field name: charge_regen
  {
    cdr << ros_message->charge_regen;
  }

  // Field name: energy_drawn
  {
    cdr << ros_message->energy_drawn;
  }

  // Field name: energy_regen
  {
    cdr << ros_message->energy_regen;
  }

  // Field name: displacement
  {
    cdr << ros_message->displacement;
  }

  // Field name: distance_traveled
  {
    cdr << ros_message->distance_traveled;
  }

  // Field name: fault_code
  {
    cdr << ros_message->fault_code;
  }

  // Field name: pid_pos_now
  {
    cdr << ros_message->pid_pos_now;
  }

  // Field name: controller_id
  {
    cdr << ros_message->controller_id;
  }

  // Field name: ntc_temp_mos1
  {
    cdr << ros_message->ntc_temp_mos1;
  }

  // Field name: ntc_temp_mos2
  {
    cdr << ros_message->ntc_temp_mos2;
  }

  // Field name: ntc_temp_mos3
  {
    cdr << ros_message->ntc_temp_mos3;
  }

  // Field name: avg_vd
  {
    cdr << ros_message->avg_vd;
  }

  // Field name: avg_vq
  {
    cdr << ros_message->avg_vq;
  }

  return true;
}

static bool _VescState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VescState__ros_msg_type * ros_message = static_cast<_VescState__ros_msg_type *>(untyped_ros_message);
  // Field name: temp_fet
  {
    cdr >> ros_message->temp_fet;
  }

  // Field name: temp_motor
  {
    cdr >> ros_message->temp_motor;
  }

  // Field name: current_motor
  {
    cdr >> ros_message->current_motor;
  }

  // Field name: current_input
  {
    cdr >> ros_message->current_input;
  }

  // Field name: avg_id
  {
    cdr >> ros_message->avg_id;
  }

  // Field name: avg_iq
  {
    cdr >> ros_message->avg_iq;
  }

  // Field name: duty_cycle
  {
    cdr >> ros_message->duty_cycle;
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: voltage_input
  {
    cdr >> ros_message->voltage_input;
  }

  // Field name: charge_drawn
  {
    cdr >> ros_message->charge_drawn;
  }

  // Field name: charge_regen
  {
    cdr >> ros_message->charge_regen;
  }

  // Field name: energy_drawn
  {
    cdr >> ros_message->energy_drawn;
  }

  // Field name: energy_regen
  {
    cdr >> ros_message->energy_regen;
  }

  // Field name: displacement
  {
    cdr >> ros_message->displacement;
  }

  // Field name: distance_traveled
  {
    cdr >> ros_message->distance_traveled;
  }

  // Field name: fault_code
  {
    cdr >> ros_message->fault_code;
  }

  // Field name: pid_pos_now
  {
    cdr >> ros_message->pid_pos_now;
  }

  // Field name: controller_id
  {
    cdr >> ros_message->controller_id;
  }

  // Field name: ntc_temp_mos1
  {
    cdr >> ros_message->ntc_temp_mos1;
  }

  // Field name: ntc_temp_mos2
  {
    cdr >> ros_message->ntc_temp_mos2;
  }

  // Field name: ntc_temp_mos3
  {
    cdr >> ros_message->ntc_temp_mos3;
  }

  // Field name: avg_vd
  {
    cdr >> ros_message->avg_vd;
  }

  // Field name: avg_vq
  {
    cdr >> ros_message->avg_vq;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vesc_msgs
size_t get_serialized_size_vesc_msgs__msg__VescState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VescState__ros_msg_type * ros_message = static_cast<const _VescState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name temp_fet
  {
    size_t item_size = sizeof(ros_message->temp_fet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temp_motor
  {
    size_t item_size = sizeof(ros_message->temp_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_motor
  {
    size_t item_size = sizeof(ros_message->current_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current_input
  {
    size_t item_size = sizeof(ros_message->current_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name avg_id
  {
    size_t item_size = sizeof(ros_message->avg_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name avg_iq
  {
    size_t item_size = sizeof(ros_message->avg_iq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duty_cycle
  {
    size_t item_size = sizeof(ros_message->duty_cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name voltage_input
  {
    size_t item_size = sizeof(ros_message->voltage_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name charge_drawn
  {
    size_t item_size = sizeof(ros_message->charge_drawn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name charge_regen
  {
    size_t item_size = sizeof(ros_message->charge_regen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name energy_drawn
  {
    size_t item_size = sizeof(ros_message->energy_drawn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name energy_regen
  {
    size_t item_size = sizeof(ros_message->energy_regen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name displacement
  {
    size_t item_size = sizeof(ros_message->displacement);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name distance_traveled
  {
    size_t item_size = sizeof(ros_message->distance_traveled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fault_code
  {
    size_t item_size = sizeof(ros_message->fault_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pid_pos_now
  {
    size_t item_size = sizeof(ros_message->pid_pos_now);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name controller_id
  {
    size_t item_size = sizeof(ros_message->controller_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ntc_temp_mos1
  {
    size_t item_size = sizeof(ros_message->ntc_temp_mos1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ntc_temp_mos2
  {
    size_t item_size = sizeof(ros_message->ntc_temp_mos2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ntc_temp_mos3
  {
    size_t item_size = sizeof(ros_message->ntc_temp_mos3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name avg_vd
  {
    size_t item_size = sizeof(ros_message->avg_vd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name avg_vq
  {
    size_t item_size = sizeof(ros_message->avg_vq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VescState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vesc_msgs__msg__VescState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vesc_msgs
size_t max_serialized_size_vesc_msgs__msg__VescState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: temp_fet
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: temp_motor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: current_motor
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: current_input
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: avg_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: avg_iq
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: duty_cycle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: voltage_input
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: charge_drawn
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: charge_regen
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: energy_drawn
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: energy_regen
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: displacement
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: distance_traveled
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: fault_code
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pid_pos_now
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: controller_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ntc_temp_mos1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ntc_temp_mos2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ntc_temp_mos3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: avg_vd
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: avg_vq
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _VescState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_vesc_msgs__msg__VescState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VescState = {
  "vesc_msgs::msg",
  "VescState",
  _VescState__cdr_serialize,
  _VescState__cdr_deserialize,
  _VescState__get_serialized_size,
  _VescState__max_serialized_size
};

static rosidl_message_type_support_t _VescState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VescState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vesc_msgs, msg, VescState)() {
  return &_VescState__type_support;
}

#if defined(__cplusplus)
}
#endif
