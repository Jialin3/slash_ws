// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_H_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "vesc_msgs/msg/detail/vesc_state__struct.h"

// Struct defined in msg/VescStateStamped in the package vesc_msgs.
typedef struct vesc_msgs__msg__VescStateStamped
{
  std_msgs__msg__Header header;
  vesc_msgs__msg__VescState state;
} vesc_msgs__msg__VescStateStamped;

// Struct for a sequence of vesc_msgs__msg__VescStateStamped.
typedef struct vesc_msgs__msg__VescStateStamped__Sequence
{
  vesc_msgs__msg__VescStateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vesc_msgs__msg__VescStateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__STRUCT_H_
