// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from teb_msgs:msg/TrajectoryMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__FUNCTIONS_H_
#define TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "teb_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "teb_msgs/msg/detail/trajectory_msg__struct.h"

/// Initialize msg/TrajectoryMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * teb_msgs__msg__TrajectoryMsg
 * )) before or use
 * teb_msgs__msg__TrajectoryMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__init(teb_msgs__msg__TrajectoryMsg * msg);

/// Finalize msg/TrajectoryMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__TrajectoryMsg__fini(teb_msgs__msg__TrajectoryMsg * msg);

/// Create msg/TrajectoryMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * teb_msgs__msg__TrajectoryMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
teb_msgs__msg__TrajectoryMsg *
teb_msgs__msg__TrajectoryMsg__create();

/// Destroy msg/TrajectoryMsg message.
/**
 * It calls
 * teb_msgs__msg__TrajectoryMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__TrajectoryMsg__destroy(teb_msgs__msg__TrajectoryMsg * msg);

/// Check for msg/TrajectoryMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__are_equal(const teb_msgs__msg__TrajectoryMsg * lhs, const teb_msgs__msg__TrajectoryMsg * rhs);

/// Copy a msg/TrajectoryMsg message.
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
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__copy(
  const teb_msgs__msg__TrajectoryMsg * input,
  teb_msgs__msg__TrajectoryMsg * output);

/// Initialize array of msg/TrajectoryMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * teb_msgs__msg__TrajectoryMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__Sequence__init(teb_msgs__msg__TrajectoryMsg__Sequence * array, size_t size);

/// Finalize array of msg/TrajectoryMsg messages.
/**
 * It calls
 * teb_msgs__msg__TrajectoryMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__TrajectoryMsg__Sequence__fini(teb_msgs__msg__TrajectoryMsg__Sequence * array);

/// Create array of msg/TrajectoryMsg messages.
/**
 * It allocates the memory for the array and calls
 * teb_msgs__msg__TrajectoryMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
teb_msgs__msg__TrajectoryMsg__Sequence *
teb_msgs__msg__TrajectoryMsg__Sequence__create(size_t size);

/// Destroy array of msg/TrajectoryMsg messages.
/**
 * It calls
 * teb_msgs__msg__TrajectoryMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__TrajectoryMsg__Sequence__destroy(teb_msgs__msg__TrajectoryMsg__Sequence * array);

/// Check for msg/TrajectoryMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__Sequence__are_equal(const teb_msgs__msg__TrajectoryMsg__Sequence * lhs, const teb_msgs__msg__TrajectoryMsg__Sequence * rhs);

/// Copy an array of msg/TrajectoryMsg messages.
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
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__TrajectoryMsg__Sequence__copy(
  const teb_msgs__msg__TrajectoryMsg__Sequence * input,
  teb_msgs__msg__TrajectoryMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TEB_MSGS__MSG__DETAIL__TRAJECTORY_MSG__FUNCTIONS_H_
