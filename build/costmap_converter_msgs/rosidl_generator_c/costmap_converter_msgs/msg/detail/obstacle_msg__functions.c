// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from costmap_converter_msgs:msg/ObstacleMsg.idl
// generated code does not contain a copyright notice
#include "costmap_converter_msgs/msg/detail/obstacle_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `polygon`
#include "geometry_msgs/msg/detail/polygon__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `velocities`
#include "geometry_msgs/msg/detail/twist_with_covariance__functions.h"

bool
costmap_converter_msgs__msg__ObstacleMsg__init(costmap_converter_msgs__msg__ObstacleMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    costmap_converter_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  // polygon
  if (!geometry_msgs__msg__Polygon__init(&msg->polygon)) {
    costmap_converter_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  // radius
  // id
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    costmap_converter_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__TwistWithCovariance__init(&msg->velocities)) {
    costmap_converter_msgs__msg__ObstacleMsg__fini(msg);
    return false;
  }
  return true;
}

void
costmap_converter_msgs__msg__ObstacleMsg__fini(costmap_converter_msgs__msg__ObstacleMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // polygon
  geometry_msgs__msg__Polygon__fini(&msg->polygon);
  // radius
  // id
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // velocities
  geometry_msgs__msg__TwistWithCovariance__fini(&msg->velocities);
}

bool
costmap_converter_msgs__msg__ObstacleMsg__are_equal(const costmap_converter_msgs__msg__ObstacleMsg * lhs, const costmap_converter_msgs__msg__ObstacleMsg * rhs)
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
  // polygon
  if (!geometry_msgs__msg__Polygon__are_equal(
      &(lhs->polygon), &(rhs->polygon)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__TwistWithCovariance__are_equal(
      &(lhs->velocities), &(rhs->velocities)))
  {
    return false;
  }
  return true;
}

bool
costmap_converter_msgs__msg__ObstacleMsg__copy(
  const costmap_converter_msgs__msg__ObstacleMsg * input,
  costmap_converter_msgs__msg__ObstacleMsg * output)
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
  // polygon
  if (!geometry_msgs__msg__Polygon__copy(
      &(input->polygon), &(output->polygon)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  // id
  output->id = input->id;
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // velocities
  if (!geometry_msgs__msg__TwistWithCovariance__copy(
      &(input->velocities), &(output->velocities)))
  {
    return false;
  }
  return true;
}

costmap_converter_msgs__msg__ObstacleMsg *
costmap_converter_msgs__msg__ObstacleMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  costmap_converter_msgs__msg__ObstacleMsg * msg = (costmap_converter_msgs__msg__ObstacleMsg *)allocator.allocate(sizeof(costmap_converter_msgs__msg__ObstacleMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(costmap_converter_msgs__msg__ObstacleMsg));
  bool success = costmap_converter_msgs__msg__ObstacleMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
costmap_converter_msgs__msg__ObstacleMsg__destroy(costmap_converter_msgs__msg__ObstacleMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    costmap_converter_msgs__msg__ObstacleMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
costmap_converter_msgs__msg__ObstacleMsg__Sequence__init(costmap_converter_msgs__msg__ObstacleMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  costmap_converter_msgs__msg__ObstacleMsg * data = NULL;

  if (size) {
    data = (costmap_converter_msgs__msg__ObstacleMsg *)allocator.zero_allocate(size, sizeof(costmap_converter_msgs__msg__ObstacleMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = costmap_converter_msgs__msg__ObstacleMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        costmap_converter_msgs__msg__ObstacleMsg__fini(&data[i - 1]);
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
costmap_converter_msgs__msg__ObstacleMsg__Sequence__fini(costmap_converter_msgs__msg__ObstacleMsg__Sequence * array)
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
      costmap_converter_msgs__msg__ObstacleMsg__fini(&array->data[i]);
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

costmap_converter_msgs__msg__ObstacleMsg__Sequence *
costmap_converter_msgs__msg__ObstacleMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  costmap_converter_msgs__msg__ObstacleMsg__Sequence * array = (costmap_converter_msgs__msg__ObstacleMsg__Sequence *)allocator.allocate(sizeof(costmap_converter_msgs__msg__ObstacleMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = costmap_converter_msgs__msg__ObstacleMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
costmap_converter_msgs__msg__ObstacleMsg__Sequence__destroy(costmap_converter_msgs__msg__ObstacleMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    costmap_converter_msgs__msg__ObstacleMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
costmap_converter_msgs__msg__ObstacleMsg__Sequence__are_equal(const costmap_converter_msgs__msg__ObstacleMsg__Sequence * lhs, const costmap_converter_msgs__msg__ObstacleMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!costmap_converter_msgs__msg__ObstacleMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
costmap_converter_msgs__msg__ObstacleMsg__Sequence__copy(
  const costmap_converter_msgs__msg__ObstacleMsg__Sequence * input,
  costmap_converter_msgs__msg__ObstacleMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(costmap_converter_msgs__msg__ObstacleMsg);
    costmap_converter_msgs__msg__ObstacleMsg * data =
      (costmap_converter_msgs__msg__ObstacleMsg *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!costmap_converter_msgs__msg__ObstacleMsg__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          costmap_converter_msgs__msg__ObstacleMsg__fini(&data[i]);
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
    if (!costmap_converter_msgs__msg__ObstacleMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
