// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from fast_lio:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef FAST_LIO__MSG__DETAIL__POSE6_D__FUNCTIONS_H_
#define FAST_LIO__MSG__DETAIL__POSE6_D__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "fast_lio/msg/rosidl_generator_c__visibility_control.h"

#include "fast_lio/msg/detail/pose6_d__struct.h"

/// Initialize msg/Pose6D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * fast_lio__msg__Pose6D
 * )) before or use
 * fast_lio__msg__Pose6D__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__init(fast_lio__msg__Pose6D * msg);

/// Finalize msg/Pose6D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
void
fast_lio__msg__Pose6D__fini(fast_lio__msg__Pose6D * msg);

/// Create msg/Pose6D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * fast_lio__msg__Pose6D__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
fast_lio__msg__Pose6D *
fast_lio__msg__Pose6D__create();

/// Destroy msg/Pose6D message.
/**
 * It calls
 * fast_lio__msg__Pose6D__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
void
fast_lio__msg__Pose6D__destroy(fast_lio__msg__Pose6D * msg);

/// Check for msg/Pose6D message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__are_equal(const fast_lio__msg__Pose6D * lhs, const fast_lio__msg__Pose6D * rhs);

/// Copy a msg/Pose6D message.
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
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__copy(
  const fast_lio__msg__Pose6D * input,
  fast_lio__msg__Pose6D * output);

/// Initialize array of msg/Pose6D messages.
/**
 * It allocates the memory for the number of elements and calls
 * fast_lio__msg__Pose6D__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__Sequence__init(fast_lio__msg__Pose6D__Sequence * array, size_t size);

/// Finalize array of msg/Pose6D messages.
/**
 * It calls
 * fast_lio__msg__Pose6D__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
void
fast_lio__msg__Pose6D__Sequence__fini(fast_lio__msg__Pose6D__Sequence * array);

/// Create array of msg/Pose6D messages.
/**
 * It allocates the memory for the array and calls
 * fast_lio__msg__Pose6D__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
fast_lio__msg__Pose6D__Sequence *
fast_lio__msg__Pose6D__Sequence__create(size_t size);

/// Destroy array of msg/Pose6D messages.
/**
 * It calls
 * fast_lio__msg__Pose6D__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
void
fast_lio__msg__Pose6D__Sequence__destroy(fast_lio__msg__Pose6D__Sequence * array);

/// Check for msg/Pose6D message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__Sequence__are_equal(const fast_lio__msg__Pose6D__Sequence * lhs, const fast_lio__msg__Pose6D__Sequence * rhs);

/// Copy an array of msg/Pose6D messages.
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
ROSIDL_GENERATOR_C_PUBLIC_fast_lio
bool
fast_lio__msg__Pose6D__Sequence__copy(
  const fast_lio__msg__Pose6D__Sequence * input,
  fast_lio__msg__Pose6D__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FAST_LIO__MSG__DETAIL__POSE6_D__FUNCTIONS_H_
