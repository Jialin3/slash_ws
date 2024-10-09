// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from teb_msgs:msg/TrajectoryMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_H_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_H_

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
// Member 'trajectory'
#include "teb_msgs/msg/detail/trajectory_point_msg__struct.h"

// Struct defined in msg/TrajectoryMsg in the package teb_msgs.
typedef struct teb_msgs__msg__TrajectoryMsg
{
  std_msgs__msg__Header header;
  teb_msgs__msg__TrajectoryPointMsg__Sequence trajectory;
} teb_msgs__msg__TrajectoryMsg;

// Struct for a sequence of teb_msgs__msg__TrajectoryMsg.
typedef struct teb_msgs__msg__TrajectoryMsg__Sequence
{
  teb_msgs__msg__TrajectoryMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teb_msgs__msg__TrajectoryMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__STRUCT_H_
