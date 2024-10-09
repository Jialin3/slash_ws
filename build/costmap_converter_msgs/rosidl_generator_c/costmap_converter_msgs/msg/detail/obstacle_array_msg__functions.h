// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from costmap_converter_msgs:msg/ObstacleArrayMsg.idl
// generated code does not contain a copyright notice

#ifndef COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__FUNCTIONS_H_
#define COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "costmap_converter_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__struct.h"

/// Initialize msg/ObstacleArrayMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * costmap_converter_msgs__msg__ObstacleArrayMsg
 * )) before or use
 * costmap_converter_msgs__msg__ObstacleArrayMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__init(costmap_converter_msgs__msg__ObstacleArrayMsg * msg);

/// Finalize msg/ObstacleArrayMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
void
costmap_converter_msgs__msg__ObstacleArrayMsg__fini(costmap_converter_msgs__msg__ObstacleArrayMsg * msg);

/// Create msg/ObstacleArrayMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
costmap_converter_msgs__msg__ObstacleArrayMsg *
costmap_converter_msgs__msg__ObstacleArrayMsg__create();

/// Destroy msg/ObstacleArrayMsg message.
/**
 * It calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
void
costmap_converter_msgs__msg__ObstacleArrayMsg__destroy(costmap_converter_msgs__msg__ObstacleArrayMsg * msg);

/// Check for msg/ObstacleArrayMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__are_equal(const costmap_converter_msgs__msg__ObstacleArrayMsg * lhs, const costmap_converter_msgs__msg__ObstacleArrayMsg * rhs);

/// Copy a msg/ObstacleArrayMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__copy(
  const costmap_converter_msgs__msg__ObstacleArrayMsg * input,
  costmap_converter_msgs__msg__ObstacleArrayMsg * output);

/// Initialize array of msg/ObstacleArrayMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__init(costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * array, size_t size);

/// Finalize array of msg/ObstacleArrayMsg messages.
/**
 * It calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
void
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__fini(costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * array);

/// Create array of msg/ObstacleArrayMsg messages.
/**
 * It allocates the memory for the array and calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence *
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__create(size_t size);

/// Destroy array of msg/ObstacleArrayMsg messages.
/**
 * It calls
 * costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
void
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__destroy(costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * array);

/// Check for msg/ObstacleArrayMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__are_equal(const costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * lhs, const costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * rhs);

/// Copy an array of msg/ObstacleArrayMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_costmap_converter_msgs
bool
costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence__copy(
  const costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * input,
  costmap_converter_msgs__msg__ObstacleArrayMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // COSTMAP_CONVERTER_MSGS__MSG__DETAIL__OBSTACLE_ARRAY_MSG__FUNCTIONS_H_
