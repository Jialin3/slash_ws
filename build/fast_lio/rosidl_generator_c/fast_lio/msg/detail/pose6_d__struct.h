// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_H_
#define FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Pose6D in the package fast_lio.
typedef struct fast_lio__msg__Pose6D
{
  double offset_time;
  double acc[3];
  double gyr[3];
  double vel[3];
  double pos[3];
  double rot[9];
} fast_lio__msg__Pose6D;

// Struct for a sequence of fast_lio__msg__Pose6D.
typedef struct fast_lio__msg__Pose6D__Sequence
{
  fast_lio__msg__Pose6D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fast_lio__msg__Pose6D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FAST_LIO__MSG__DETAIL__POSE6_D__STRUCT_H_
