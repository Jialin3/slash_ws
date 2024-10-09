// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_

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
// Member 'obstacles'
#include "costmap_converter_msgs/msg/detail/obstacle_msg__struct.h"

// Struct defined in msg/ObstacleArrayMsg in the package costmap_converter_msgs.
typedef struct costmap_converter_msgs__msg__ObstacleArrayMsg
{
  std_msgs__msg__Header header;
  costmap_converter_msgs__msg__ObstacleMsg__Sequence obstacles;
} costmap_converter_msgs__msg__ObstacleArrayMsg;

// Struct for a sequence of costmap_converter_msgs__msg__ObstacleArrayMsg.
typedef struct costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence
{
  costmap_converter_msgs__msg__ObstacleArrayMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__STRUCT_H_
