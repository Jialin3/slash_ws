// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from teb_msgs:msg/FeedbackMsg.idl
// generated code does not contain a copyright notice
#include "teb_msgs/msg/detail/feedback_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `trajectories`
#include "teb_msgs/msg/detail/trajectory_msg__functions.h"
// Member `obstacles_msg`
#include "costmap_converter_msgs/msg/detail/obstacle_array_msg__functions.h"

bool
teb_msgs__msg__FeedbackMsg__init(teb_msgs__msg__FeedbackMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    teb_msgs__msg__FeedbackMsg__fini(msg);
    return false;
  }
  // trajectories
  if (!teb_msgs__msg__TrajectoryMsg__Sequence__init(&msg->trajectories, 0)) {
    teb_msgs__msg__FeedbackMsg__fini(msg);
    return false;
  }
  // selected_trajectory_idx
  // obstacles_msg
  if (!costmap_converter_msgs__msg__ObstacleArrayMsg__init(&msg->obstacles_msg)) {
    teb_msgs__msg__FeedbackMsg__fini(msg);
    return false;
  }
  return true;
}

void
teb_msgs__msg__FeedbackMsg__fini(teb_msgs__msg__FeedbackMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trajectories
  teb_msgs__msg__TrajectoryMsg__Sequence__fini(&msg->trajectories);
  // selected_trajectory_idx
  // obstacles_msg
  costmap_converter_msgs__msg__ObstacleArrayMsg__fini(&msg->obstacles_msg);
}

bool
teb_msgs__msg__FeedbackMsg__are_equal(const teb_msgs__msg__FeedbackMsg * lhs, const teb_msgs__msg__FeedbackMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // trajectories
  if (!teb_msgs__msg__TrajectoryMsg__Sequence__are_equal(
      &(lhs->trajectories), &(rhs->trajectories)))
  {
    return false;
  }
  // selected_trajectory_idx
  if (lhs->selected_trajectory_idx != rhs->selected_trajectory_idx) {
    return false;
  }
  // obstacles_msg
  if (!costmap_converter_msgs__msg__ObstacleArrayMsg__are_equal(
      &(lhs->obstacles_msg), &(rhs->obstacles_msg)))
  {
    return false;
  }
  return true;
}

bool
teb_msgs__msg__FeedbackMsg__copy(
  const teb_msgs__msg__FeedbackMsg * input,
  teb_msgs__msg__FeedbackMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // trajectories
  if (!teb_msgs__msg__TrajectoryMsg__Sequence__copy(
      &(input->trajectories), &(output->trajectories)))
  {
    return false;
  }
  // selected_trajectory_idx
  output->selected_trajectory_idx = input->selected_trajectory_idx;
  // obstacles_msg
  if (!costmap_converter_msgs__msg__ObstacleArrayMsg__copy(
      &(input->obstacles_msg), &(output->obstacles_msg)))
  {
    return false;
  }
  return true;
}

teb_msgs__msg__FeedbackMsg *
teb_msgs__msg__FeedbackMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__FeedbackMsg * msg = (teb_msgs__msg__FeedbackMsg *)allocator.allocate(sizeof(teb_msgs__msg__FeedbackMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teb_msgs__msg__FeedbackMsg));
  bool success = teb_msgs__msg__FeedbackMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teb_msgs__msg__FeedbackMsg__destroy(teb_msgs__msg__FeedbackMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teb_msgs__msg__FeedbackMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teb_msgs__msg__FeedbackMsg__Sequence__init(teb_msgs__msg__FeedbackMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__FeedbackMsg * data = NULL;

  if (size) {
    data = (teb_msgs__msg__FeedbackMsg *)allocator.zero_allocate(size, sizeof(teb_msgs__msg__FeedbackMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teb_msgs__msg__FeedbackMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teb_msgs__msg__FeedbackMsg__fini(&data[i - 1]);
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
teb_msgs__msg__FeedbackMsg__Sequence__fini(teb_msgs__msg__FeedbackMsg__Sequence * array)
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
      teb_msgs__msg__FeedbackMsg__fini(&array->data[i]);
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

teb_msgs__msg__FeedbackMsg__Sequence *
teb_msgs__msg__FeedbackMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teb_msgs__msg__FeedbackMsg__Sequence * array = (teb_msgs__msg__FeedbackMsg__Sequence *)allocator.allocate(sizeof(teb_msgs__msg__FeedbackMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teb_msgs__msg__FeedbackMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teb_msgs__msg__FeedbackMsg__Sequence__destroy(teb_msgs__msg__FeedbackMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teb_msgs__msg__FeedbackMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teb_msgs__msg__FeedbackMsg__Sequence__are_equal(const teb_msgs__msg__FeedbackMsg__Sequence * lhs, const teb_msgs__msg__FeedbackMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teb_msgs__msg__FeedbackMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teb_msgs__msg__FeedbackMsg__Sequence__copy(
  const teb_msgs__msg__FeedbackMsg__Sequence * input,
  teb_msgs__msg__FeedbackMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teb_msgs__msg__FeedbackMsg);
    teb_msgs__msg__FeedbackMsg * data =
      (teb_msgs__msg__FeedbackMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teb_msgs__msg__FeedbackMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teb_msgs__msg__FeedbackMsg__fini(&data[i]);
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
    if (!teb_msgs__msg__FeedbackMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
