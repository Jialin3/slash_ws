// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice

#ifndef TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__FUNCTIONS_H_
#define TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "teb_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "teb_msgs/msg/detail/feedback_msg__struct.h"

/// Initialize msg/FeedbackMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * teb_msgs__msg__FeedbackMsg
 * )) before or use
 * teb_msgs__msg__FeedbackMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__FeedbackMsg__init(teb_msgs__msg__FeedbackMsg * msg);

/// Finalize msg/FeedbackMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__FeedbackMsg__fini(teb_msgs__msg__FeedbackMsg * msg);

/// Create msg/FeedbackMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * teb_msgs__msg__FeedbackMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
teb_msgs__msg__FeedbackMsg *
teb_msgs__msg__FeedbackMsg__create();

/// Destroy msg/FeedbackMsg message.
/**
 * It calls
 * teb_msgs__msg__FeedbackMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__FeedbackMsg__destroy(teb_msgs__msg__FeedbackMsg * msg);

/// Check for msg/FeedbackMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__FeedbackMsg__are_equal(const teb_msgs__msg__FeedbackMsg * lhs, const teb_msgs__msg__FeedbackMsg * rhs);

/// Copy a msg/FeedbackMsg message.
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
teb_msgs__msg__FeedbackMsg__copy(
  const teb_msgs__msg__FeedbackMsg * input,
  teb_msgs__msg__FeedbackMsg * output);

/// Initialize array of msg/FeedbackMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * teb_msgs__msg__FeedbackMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__FeedbackMsg__Sequence__init(teb_msgs__msg__FeedbackMsg__Sequence * array, size_t size);

/// Finalize array of msg/FeedbackMsg messages.
/**
 * It calls
 * teb_msgs__msg__FeedbackMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__FeedbackMsg__Sequence__fini(teb_msgs__msg__FeedbackMsg__Sequence * array);

/// Create array of msg/FeedbackMsg messages.
/**
 * It allocates the memory for the array and calls
 * teb_msgs__msg__FeedbackMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
teb_msgs__msg__FeedbackMsg__Sequence *
teb_msgs__msg__FeedbackMsg__Sequence__create(size_t size);

/// Destroy array of msg/FeedbackMsg messages.
/**
 * It calls
 * teb_msgs__msg__FeedbackMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
void
teb_msgs__msg__FeedbackMsg__Sequence__destroy(teb_msgs__msg__FeedbackMsg__Sequence * array);

/// Check for msg/FeedbackMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teb_msgs
bool
teb_msgs__msg__FeedbackMsg__Sequence__are_equal(const teb_msgs__msg__FeedbackMsg__Sequence * lhs, const teb_msgs__msg__FeedbackMsg__Sequence * rhs);

/// Copy an array of msg/FeedbackMsg messages.
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
teb_msgs__msg__FeedbackMsg__Sequence__copy(
  const teb_msgs__msg__FeedbackMsg__Sequence * input,
  teb_msgs__msg__FeedbackMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TEB_MSGS__MSG__DETAIL__FEEDBACK_MSG__FUNCTIONS_H_
