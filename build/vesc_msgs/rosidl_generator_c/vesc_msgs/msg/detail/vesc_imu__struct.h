// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_IMU__STRUCT_H_
#define VESC_MSGS__MSG__DETAIL__VESC_IMU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ypr'
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'compass'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

// Struct defined in msg/VescImu in the package vesc_msgs.
typedef struct vesc_msgs__msg__VescImu
{
  geometry_msgs__msg__Vector3 ypr;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 compass;
  geometry_msgs__msg__Quaternion orientation;
} vesc_msgs__msg__VescImu;

// Struct for a sequence of vesc_msgs__msg__VescImu.
typedef struct vesc_msgs__msg__VescImu__Sequence
{
  vesc_msgs__msg__VescImu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vesc_msgs__msg__VescImu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VESC_MSGS__MSG__DETAIL__VESC_IMU__STRUCT_H_
