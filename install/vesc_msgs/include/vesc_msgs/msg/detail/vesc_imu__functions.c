// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescImu.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ypr`
// Member `linear_acceleration`
// Member `angular_velocity`
// Member `compass`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
vesc_msgs__msg__VescImu__init(vesc_msgs__msg__VescImu * msg)
{
  if (!msg) {
    return false;
  }
  // ypr
  if (!geometry_msgs__msg__Vector3__init(&msg->ypr)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // compass
  if (!geometry_msgs__msg__Vector3__init(&msg->compass)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    vesc_msgs__msg__VescImu__fini(msg);
    return false;
  }
  return true;
}

void
vesc_msgs__msg__VescImu__fini(vesc_msgs__msg__VescImu * msg)
{
  if (!msg) {
    return;
  }
  // ypr
  geometry_msgs__msg__Vector3__fini(&msg->ypr);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // compass
  geometry_msgs__msg__Vector3__fini(&msg->compass);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
}

bool
vesc_msgs__msg__VescImu__are_equal(const vesc_msgs__msg__VescImu * lhs, const vesc_msgs__msg__VescImu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ypr
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->ypr), &(rhs->ypr)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_acceleration), &(rhs->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // compass
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->compass), &(rhs->compass)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  return true;
}

bool
vesc_msgs__msg__VescImu__copy(
  const vesc_msgs__msg__VescImu * input,
  vesc_msgs__msg__VescImu * output)
{
  if (!input || !output) {
    return false;
  }
  // ypr
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->ypr), &(output->ypr)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_acceleration), &(output->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // compass
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->compass), &(output->compass)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  return true;
}

vesc_msgs__msg__VescImu *
vesc_msgs__msg__VescImu__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImu * msg = (vesc_msgs__msg__VescImu *)allocator.allocate(sizeof(vesc_msgs__msg__VescImu), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescImu));
  bool success = vesc_msgs__msg__VescImu__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescImu__destroy(vesc_msgs__msg__VescImu * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vesc_msgs__msg__VescImu__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vesc_msgs__msg__VescImu__Sequence__init(vesc_msgs__msg__VescImu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImu * data = NULL;

  if (size) {
    data = (vesc_msgs__msg__VescImu *)allocator.zero_allocate(size, sizeof(vesc_msgs__msg__VescImu), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescImu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescImu__fini(&data[i - 1]);
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
vesc_msgs__msg__VescImu__Sequence__fini(vesc_msgs__msg__VescImu__Sequence * array)
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
      vesc_msgs__msg__VescImu__fini(&array->data[i]);
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

vesc_msgs__msg__VescImu__Sequence *
vesc_msgs__msg__VescImu__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImu__Sequence * array = (vesc_msgs__msg__VescImu__Sequence *)allocator.allocate(sizeof(vesc_msgs__msg__VescImu__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescImu__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescImu__Sequence__destroy(vesc_msgs__msg__VescImu__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vesc_msgs__msg__VescImu__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vesc_msgs__msg__VescImu__Sequence__are_equal(const vesc_msgs__msg__VescImu__Sequence * lhs, const vesc_msgs__msg__VescImu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vesc_msgs__msg__VescImu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vesc_msgs__msg__VescImu__Sequence__copy(
  const vesc_msgs__msg__VescImu__Sequence * input,
  vesc_msgs__msg__VescImu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vesc_msgs__msg__VescImu);
    vesc_msgs__msg__VescImu * data =
      (vesc_msgs__msg__VescImu *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vesc_msgs__msg__VescImu__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vesc_msgs__msg__VescImu__fini(&data[i]);
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
    if (!vesc_msgs__msg__VescImu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
