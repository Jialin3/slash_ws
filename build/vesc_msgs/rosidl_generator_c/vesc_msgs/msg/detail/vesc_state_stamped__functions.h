// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vesc_msgs:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__FUNCTIONS_H_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vesc_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "vesc_msgs/msg/detail/vesc_state_stamped__struct.h"

/// Initialize msg/VescStateStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vesc_msgs__msg__VescStateStamped
 * )) before or use
 * vesc_msgs__msg__VescStateStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__init(vesc_msgs__msg__VescStateStamped * msg);

/// Finalize msg/VescStateStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescStateStamped__fini(vesc_msgs__msg__VescStateStamped * msg);

/// Create msg/VescStateStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vesc_msgs__msg__VescStateStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
vesc_msgs__msg__VescStateStamped *
vesc_msgs__msg__VescStateStamped__create();

/// Destroy msg/VescStateStamped message.
/**
 * It calls
 * vesc_msgs__msg__VescStateStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescStateStamped__destroy(vesc_msgs__msg__VescStateStamped * msg);

/// Check for msg/VescStateStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__are_equal(const vesc_msgs__msg__VescStateStamped * lhs, const vesc_msgs__msg__VescStateStamped * rhs);

/// Copy a msg/VescStateStamped message.
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
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__copy(
  const vesc_msgs__msg__VescStateStamped * input,
  vesc_msgs__msg__VescStateStamped * output);

/// Initialize array of msg/VescStateStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * vesc_msgs__msg__VescStateStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__Sequence__init(vesc_msgs__msg__VescStateStamped__Sequence * array, size_t size);

/// Finalize array of msg/VescStateStamped messages.
/**
 * It calls
 * vesc_msgs__msg__VescStateStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescStateStamped__Sequence__fini(vesc_msgs__msg__VescStateStamped__Sequence * array);

/// Create array of msg/VescStateStamped messages.
/**
 * It allocates the memory for the array and calls
 * vesc_msgs__msg__VescStateStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
vesc_msgs__msg__VescStateStamped__Sequence *
vesc_msgs__msg__VescStateStamped__Sequence__create(size_t size);

/// Destroy array of msg/VescStateStamped messages.
/**
 * It calls
 * vesc_msgs__msg__VescStateStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
void
vesc_msgs__msg__VescStateStamped__Sequence__destroy(vesc_msgs__msg__VescStateStamped__Sequence * array);

/// Check for msg/VescStateStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__Sequence__are_equal(const vesc_msgs__msg__VescStateStamped__Sequence * lhs, const vesc_msgs__msg__VescStateStamped__Sequence * rhs);

/// Copy an array of msg/VescStateStamped messages.
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
ROSIDL_GENERATOR_C_PUBLIC_vesc_msgs
bool
vesc_msgs__msg__VescStateStamped__Sequence__copy(
  const vesc_msgs__msg__VescStateStamped__Sequence * input,
  vesc_msgs__msg__VescStateStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE_STAMPED__FUNCTIONS_H_
