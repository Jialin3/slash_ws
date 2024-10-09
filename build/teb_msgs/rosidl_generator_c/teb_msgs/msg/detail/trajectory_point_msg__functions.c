// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from teb_msgs:msg/TrajectoryPointMsg.idl
// generated code does not contain a copyright notice
#include "teb_msgs/msg/detail/trajectory_point_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `time_from_start`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
teb_msgs__msg__TrajectoryPointMsg__init(teb_msgs__msg__TrajectoryPointMsg * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    teb_msgs__msg__TrajectoryPointMsg__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    teb_msgs__msg__TrajectoryPointMsg__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__init(&msg->acceleration)) {
    teb_msgs__msg__TrajectoryPointMsg__fini(msg);
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__init(&msg->time_from_start)) {
    teb_msgs__msg__TrajectoryPointMsg__fini(msg);
    return false;
  }
  return true;
}

void
teb_msgs__msg__TrajectoryPointMsg__fini(teb_msgs__msg__TrajectoryPointMsg * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Twist__fini(&msg->acceleration);
  // time_from_start
  builtin_interfaces__msg__Duration__fini(&msg->time_from_start);
}

bool
teb_msgs__msg__TrajectoryPointMsg__are_equal(const teb_msgs__msg__TrajectoryPointMsg * lhs, const teb_msgs__msg__TrajectoryPointMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->time_from_start), &(rhs->time_from_start)))
  {
    return false;
  }
  return true;
}

bool
teb_msgs__msg__TrajectoryPointMsg__copy(
  const teb_msgs__msg__TrajectoryPointMsg * input,
  teb_msgs__msg__TrajectoryPointMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Twist__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // time_from_start
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->time_from_start), &(output->time_from_start)))
  {
    return false;
  }
  return true;
}

teb_msgs__msg__TrajectoryPointMsg *
teb_msgs__msg__TrajectoryPointMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__TrajectoryPointMsg * msg = (teb_msgs__msg__TrajectoryPointMsg *)allocator.allocate(sizeof(teb_msgs__msg__TrajectoryPointMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teb_msgs__msg__TrajectoryPointMsg));
  bool success = teb_msgs__msg__TrajectoryPointMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teb_msgs__msg__TrajectoryPointMsg__destroy(teb_msgs__msg__TrajectoryPointMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teb_msgs__msg__TrajectoryPointMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teb_msgs__msg__TrajectoryPointMsg__Sequence__init(teb_msgs__msg__TrajectoryPointMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__TrajectoryPointMsg * data = NULL;

  if (size) {
    data = (teb_msgs__msg__TrajectoryPointMsg *)allocator.zero_allocate(size, sizeof(teb_msgs__msg__TrajectoryPointMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teb_msgs__msg__TrajectoryPointMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teb_msgs__msg__TrajectoryPointMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
teb_msgs__msg__TrajectoryPointMsg__Sequence__fini(teb_msgs__msg__TrajectoryPointMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      teb_msgs__msg__TrajectoryPointMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

teb_msgs__msg__TrajectoryPointMsg__Sequence *
teb_msgs__msg__TrajectoryPointMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__TrajectoryPointMsg__Sequence * array = (teb_msgs__msg__TrajectoryPointMsg__Sequence *)allocator.allocate(sizeof(teb_msgs__msg__TrajectoryPointMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teb_msgs__msg__TrajectoryPointMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teb_msgs__msg__TrajectoryPointMsg__Sequence__destroy(teb_msgs__msg__TrajectoryPointMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teb_msgs__msg__TrajectoryPointMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teb_msgs__msg__TrajectoryPointMsg__Sequence__are_equal(const teb_msgs__msg__TrajectoryPointMsg__Sequence * lhs, const teb_msgs__msg__TrajectoryPointMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teb_msgs__msg__TrajectoryPointMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teb_msgs__msg__TrajectoryPointMsg__Sequence__copy(
  const teb_msgs__msg__TrajectoryPointMsg__Sequence * input,
  teb_msgs__msg__TrajectoryPointMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teb_msgs__msg__TrajectoryPointMsg);
    teb_msgs__msg__TrajectoryPointMsg * data =
      (teb_msgs__msg__TrajectoryPointMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teb_msgs__msg__TrajectoryPointMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teb_msgs__msg__TrajectoryPointMsg__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!teb_msgs__msg__TrajectoryPointMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
