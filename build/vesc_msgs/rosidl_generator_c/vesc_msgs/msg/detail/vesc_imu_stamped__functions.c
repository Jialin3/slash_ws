// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescImuStamped.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_imu_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `imu`
#include "vesc_msgs/msg/detail/vesc_imu__functions.h"

bool
vesc_msgs__msg__VescImuStamped__init(vesc_msgs__msg__VescImuStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vesc_msgs__msg__VescImuStamped__fini(msg);
    return false;
  }
  // imu
  if (!vesc_msgs__msg__VescImu__init(&msg->imu)) {
    vesc_msgs__msg__VescImuStamped__fini(msg);
    return false;
  }
  return true;
}

void
vesc_msgs__msg__VescImuStamped__fini(vesc_msgs__msg__VescImuStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // imu
  vesc_msgs__msg__VescImu__fini(&msg->imu);
}

bool
vesc_msgs__msg__VescImuStamped__are_equal(const vesc_msgs__msg__VescImuStamped * lhs, const vesc_msgs__msg__VescImuStamped * rhs)
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
  // imu
  if (!vesc_msgs__msg__VescImu__are_equal(
      &(lhs->imu), &(rhs->imu)))
  {
    return false;
  }
  return true;
}

bool
vesc_msgs__msg__VescImuStamped__copy(
  const vesc_msgs__msg__VescImuStamped * input,
  vesc_msgs__msg__VescImuStamped * output)
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
  // imu
  if (!vesc_msgs__msg__VescImu__copy(
      &(input->imu), &(output->imu)))
  {
    return false;
  }
  return true;
}

vesc_msgs__msg__VescImuStamped *
vesc_msgs__msg__VescImuStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImuStamped * msg = (vesc_msgs__msg__VescImuStamped *)allocator.allocate(sizeof(vesc_msgs__msg__VescImuStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescImuStamped));
  bool success = vesc_msgs__msg__VescImuStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescImuStamped__destroy(vesc_msgs__msg__VescImuStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vesc_msgs__msg__VescImuStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vesc_msgs__msg__VescImuStamped__Sequence__init(vesc_msgs__msg__VescImuStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImuStamped * data = NULL;

  if (size) {
    data = (vesc_msgs__msg__VescImuStamped *)allocator.zero_allocate(size, sizeof(vesc_msgs__msg__VescImuStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescImuStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescImuStamped__fini(&data[i - 1]);
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
vesc_msgs__msg__VescImuStamped__Sequence__fini(vesc_msgs__msg__VescImuStamped__Sequence * array)
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
      vesc_msgs__msg__VescImuStamped__fini(&array->data[i]);
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

vesc_msgs__msg__VescImuStamped__Sequence *
vesc_msgs__msg__VescImuStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescImuStamped__Sequence * array = (vesc_msgs__msg__VescImuStamped__Sequence *)allocator.allocate(sizeof(vesc_msgs__msg__VescImuStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescImuStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescImuStamped__Sequence__destroy(vesc_msgs__msg__VescImuStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vesc_msgs__msg__VescImuStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vesc_msgs__msg__VescImuStamped__Sequence__are_equal(const vesc_msgs__msg__VescImuStamped__Sequence * lhs, const vesc_msgs__msg__VescImuStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vesc_msgs__msg__VescImuStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vesc_msgs__msg__VescImuStamped__Sequence__copy(
  const vesc_msgs__msg__VescImuStamped__Sequence * input,
  vesc_msgs__msg__VescImuStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vesc_msgs__msg__VescImuStamped);
    vesc_msgs__msg__VescImuStamped * data =
      (vesc_msgs__msg__VescImuStamped *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vesc_msgs__msg__VescImuStamped__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vesc_msgs__msg__VescImuStamped__fini(&data[i]);
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
    if (!vesc_msgs__msg__VescImuStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
