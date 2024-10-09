// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vesc_msgs:msg/VescImuStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_H_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_H_

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
// Member 'imu'
#include "vesc_msgs/msg/detail/vesc_imu__struct.h"

// Struct defined in msg/VescImuStamped in the package vesc_msgs.
typedef struct vesc_msgs__msg__VescImuStamped
{
  std_msgs__msg__Header header;
  vesc_msgs__msg__VescImu imu;
} vesc_msgs__msg__VescImuStamped;

// Struct for a sequence of vesc_msgs__msg__VescImuStamped.
typedef struct vesc_msgs__msg__VescImuStamped__Sequence
{
  vesc_msgs__msg__VescImuStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vesc_msgs__msg__VescImuStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU_STAMPED__STRUCT_H_
