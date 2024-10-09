// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from teleop_tools_msgs:action/Increment.idl
// generated code does not contain a copyright notice
#include "teleop_tools_msgs/action/detail/increment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `increment_by`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
teleop_tools_msgs__action__Increment_Goal__init(teleop_tools_msgs__action__Increment_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // increment_by
  if (!rosidl_runtime_c__float__Sequence__init(&msg->increment_by, 0)) {
    teleop_tools_msgs__action__Increment_Goal__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_Goal__fini(teleop_tools_msgs__action__Increment_Goal * msg)
{
  if (!msg) {
    return;
  }
  // increment_by
  rosidl_runtime_c__float__Sequence__fini(&msg->increment_by);
}

bool
teleop_tools_msgs__action__Increment_Goal__are_equal(const teleop_tools_msgs__action__Increment_Goal * lhs, const teleop_tools_msgs__action__Increment_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // increment_by
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->increment_by), &(rhs->increment_by)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Goal__copy(
  const teleop_tools_msgs__action__Increment_Goal * input,
  teleop_tools_msgs__action__Increment_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // increment_by
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->increment_by), &(output->increment_by)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_Goal *
teleop_tools_msgs__action__Increment_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Goal * msg = (teleop_tools_msgs__action__Increment_Goal *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_Goal));
  bool success = teleop_tools_msgs__action__Increment_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_Goal__destroy(teleop_tools_msgs__action__Increment_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_Goal__Sequence__init(teleop_tools_msgs__action__Increment_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Goal * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_Goal *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_Goal__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_Goal__Sequence__fini(teleop_tools_msgs__action__Increment_Goal__Sequence * array)
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
      teleop_tools_msgs__action__Increment_Goal__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_Goal__Sequence *
teleop_tools_msgs__action__Increment_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Goal__Sequence * array = (teleop_tools_msgs__action__Increment_Goal__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_Goal__Sequence__destroy(teleop_tools_msgs__action__Increment_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_Goal__Sequence__are_equal(const teleop_tools_msgs__action__Increment_Goal__Sequence * lhs, const teleop_tools_msgs__action__Increment_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Goal__Sequence__copy(
  const teleop_tools_msgs__action__Increment_Goal__Sequence * input,
  teleop_tools_msgs__action__Increment_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_Goal);
    teleop_tools_msgs__action__Increment_Goal * data =
      (teleop_tools_msgs__action__Increment_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_Goal__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
teleop_tools_msgs__action__Increment_Result__init(teleop_tools_msgs__action__Increment_Result * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
teleop_tools_msgs__action__Increment_Result__fini(teleop_tools_msgs__action__Increment_Result * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
teleop_tools_msgs__action__Increment_Result__are_equal(const teleop_tools_msgs__action__Increment_Result * lhs, const teleop_tools_msgs__action__Increment_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Result__copy(
  const teleop_tools_msgs__action__Increment_Result * input,
  teleop_tools_msgs__action__Increment_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

teleop_tools_msgs__action__Increment_Result *
teleop_tools_msgs__action__Increment_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Result * msg = (teleop_tools_msgs__action__Increment_Result *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_Result));
  bool success = teleop_tools_msgs__action__Increment_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_Result__destroy(teleop_tools_msgs__action__Increment_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_Result__Sequence__init(teleop_tools_msgs__action__Increment_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Result * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_Result *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_Result__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_Result__Sequence__fini(teleop_tools_msgs__action__Increment_Result__Sequence * array)
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
      teleop_tools_msgs__action__Increment_Result__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_Result__Sequence *
teleop_tools_msgs__action__Increment_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Result__Sequence * array = (teleop_tools_msgs__action__Increment_Result__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_Result__Sequence__destroy(teleop_tools_msgs__action__Increment_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_Result__Sequence__are_equal(const teleop_tools_msgs__action__Increment_Result__Sequence * lhs, const teleop_tools_msgs__action__Increment_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Result__Sequence__copy(
  const teleop_tools_msgs__action__Increment_Result__Sequence * input,
  teleop_tools_msgs__action__Increment_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_Result);
    teleop_tools_msgs__action__Increment_Result * data =
      (teleop_tools_msgs__action__Increment_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_Result__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
teleop_tools_msgs__action__Increment_Feedback__init(teleop_tools_msgs__action__Increment_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
teleop_tools_msgs__action__Increment_Feedback__fini(teleop_tools_msgs__action__Increment_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
teleop_tools_msgs__action__Increment_Feedback__are_equal(const teleop_tools_msgs__action__Increment_Feedback * lhs, const teleop_tools_msgs__action__Increment_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Feedback__copy(
  const teleop_tools_msgs__action__Increment_Feedback * input,
  teleop_tools_msgs__action__Increment_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

teleop_tools_msgs__action__Increment_Feedback *
teleop_tools_msgs__action__Increment_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Feedback * msg = (teleop_tools_msgs__action__Increment_Feedback *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_Feedback));
  bool success = teleop_tools_msgs__action__Increment_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_Feedback__destroy(teleop_tools_msgs__action__Increment_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_Feedback__Sequence__init(teleop_tools_msgs__action__Increment_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Feedback * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_Feedback *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_Feedback__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_Feedback__Sequence__fini(teleop_tools_msgs__action__Increment_Feedback__Sequence * array)
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
      teleop_tools_msgs__action__Increment_Feedback__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_Feedback__Sequence *
teleop_tools_msgs__action__Increment_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_Feedback__Sequence * array = (teleop_tools_msgs__action__Increment_Feedback__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_Feedback__Sequence__destroy(teleop_tools_msgs__action__Increment_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_Feedback__Sequence__are_equal(const teleop_tools_msgs__action__Increment_Feedback__Sequence * lhs, const teleop_tools_msgs__action__Increment_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_Feedback__Sequence__copy(
  const teleop_tools_msgs__action__Increment_Feedback__Sequence * input,
  teleop_tools_msgs__action__Increment_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_Feedback);
    teleop_tools_msgs__action__Increment_Feedback * data =
      (teleop_tools_msgs__action__Increment_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_Feedback__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"

bool
teleop_tools_msgs__action__Increment_SendGoal_Request__init(teleop_tools_msgs__action__Increment_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    teleop_tools_msgs__action__Increment_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!teleop_tools_msgs__action__Increment_Goal__init(&msg->goal)) {
    teleop_tools_msgs__action__Increment_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Request__fini(teleop_tools_msgs__action__Increment_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  teleop_tools_msgs__action__Increment_Goal__fini(&msg->goal);
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Request__are_equal(const teleop_tools_msgs__action__Increment_SendGoal_Request * lhs, const teleop_tools_msgs__action__Increment_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!teleop_tools_msgs__action__Increment_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Request__copy(
  const teleop_tools_msgs__action__Increment_SendGoal_Request * input,
  teleop_tools_msgs__action__Increment_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!teleop_tools_msgs__action__Increment_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_SendGoal_Request *
teleop_tools_msgs__action__Increment_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Request * msg = (teleop_tools_msgs__action__Increment_SendGoal_Request *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request));
  bool success = teleop_tools_msgs__action__Increment_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Request__destroy(teleop_tools_msgs__action__Increment_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__init(teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Request * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_SendGoal_Request *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_SendGoal_Request__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__fini(teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * array)
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
      teleop_tools_msgs__action__Increment_SendGoal_Request__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence *
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * array = (teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__destroy(teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__are_equal(const teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * lhs, const teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence__copy(
  const teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * input,
  teleop_tools_msgs__action__Increment_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_SendGoal_Request);
    teleop_tools_msgs__action__Increment_SendGoal_Request * data =
      (teleop_tools_msgs__action__Increment_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_SendGoal_Request__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
teleop_tools_msgs__action__Increment_SendGoal_Response__init(teleop_tools_msgs__action__Increment_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    teleop_tools_msgs__action__Increment_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Response__fini(teleop_tools_msgs__action__Increment_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Response__are_equal(const teleop_tools_msgs__action__Increment_SendGoal_Response * lhs, const teleop_tools_msgs__action__Increment_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Response__copy(
  const teleop_tools_msgs__action__Increment_SendGoal_Response * input,
  teleop_tools_msgs__action__Increment_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_SendGoal_Response *
teleop_tools_msgs__action__Increment_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Response * msg = (teleop_tools_msgs__action__Increment_SendGoal_Response *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response));
  bool success = teleop_tools_msgs__action__Increment_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Response__destroy(teleop_tools_msgs__action__Increment_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__init(teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Response * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_SendGoal_Response *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_SendGoal_Response__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__fini(teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * array)
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
      teleop_tools_msgs__action__Increment_SendGoal_Response__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence *
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * array = (teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__destroy(teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__are_equal(const teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * lhs, const teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence__copy(
  const teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * input,
  teleop_tools_msgs__action__Increment_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_SendGoal_Response);
    teleop_tools_msgs__action__Increment_SendGoal_Response * data =
      (teleop_tools_msgs__action__Increment_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_SendGoal_Response__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
teleop_tools_msgs__action__Increment_GetResult_Request__init(teleop_tools_msgs__action__Increment_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    teleop_tools_msgs__action__Increment_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_GetResult_Request__fini(teleop_tools_msgs__action__Increment_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
teleop_tools_msgs__action__Increment_GetResult_Request__are_equal(const teleop_tools_msgs__action__Increment_GetResult_Request * lhs, const teleop_tools_msgs__action__Increment_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_GetResult_Request__copy(
  const teleop_tools_msgs__action__Increment_GetResult_Request * input,
  teleop_tools_msgs__action__Increment_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_GetResult_Request *
teleop_tools_msgs__action__Increment_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Request * msg = (teleop_tools_msgs__action__Increment_GetResult_Request *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_GetResult_Request));
  bool success = teleop_tools_msgs__action__Increment_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_GetResult_Request__destroy(teleop_tools_msgs__action__Increment_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__init(teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Request * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_GetResult_Request *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_GetResult_Request__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__fini(teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * array)
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
      teleop_tools_msgs__action__Increment_GetResult_Request__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_GetResult_Request__Sequence *
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * array = (teleop_tools_msgs__action__Increment_GetResult_Request__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__destroy(teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__are_equal(const teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * lhs, const teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_GetResult_Request__Sequence__copy(
  const teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * input,
  teleop_tools_msgs__action__Increment_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_GetResult_Request);
    teleop_tools_msgs__action__Increment_GetResult_Request * data =
      (teleop_tools_msgs__action__Increment_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_GetResult_Request__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"

bool
teleop_tools_msgs__action__Increment_GetResult_Response__init(teleop_tools_msgs__action__Increment_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!teleop_tools_msgs__action__Increment_Result__init(&msg->result)) {
    teleop_tools_msgs__action__Increment_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_GetResult_Response__fini(teleop_tools_msgs__action__Increment_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  teleop_tools_msgs__action__Increment_Result__fini(&msg->result);
}

bool
teleop_tools_msgs__action__Increment_GetResult_Response__are_equal(const teleop_tools_msgs__action__Increment_GetResult_Response * lhs, const teleop_tools_msgs__action__Increment_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!teleop_tools_msgs__action__Increment_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_GetResult_Response__copy(
  const teleop_tools_msgs__action__Increment_GetResult_Response * input,
  teleop_tools_msgs__action__Increment_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!teleop_tools_msgs__action__Increment_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_GetResult_Response *
teleop_tools_msgs__action__Increment_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Response * msg = (teleop_tools_msgs__action__Increment_GetResult_Response *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_GetResult_Response));
  bool success = teleop_tools_msgs__action__Increment_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_GetResult_Response__destroy(teleop_tools_msgs__action__Increment_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__init(teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Response * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_GetResult_Response *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_GetResult_Response__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__fini(teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * array)
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
      teleop_tools_msgs__action__Increment_GetResult_Response__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_GetResult_Response__Sequence *
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * array = (teleop_tools_msgs__action__Increment_GetResult_Response__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__destroy(teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__are_equal(const teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * lhs, const teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_GetResult_Response__Sequence__copy(
  const teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * input,
  teleop_tools_msgs__action__Increment_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_GetResult_Response);
    teleop_tools_msgs__action__Increment_GetResult_Response * data =
      (teleop_tools_msgs__action__Increment_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_GetResult_Response__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "teleop_tools_msgs/action/detail/increment__functions.h"

bool
teleop_tools_msgs__action__Increment_FeedbackMessage__init(teleop_tools_msgs__action__Increment_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    teleop_tools_msgs__action__Increment_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!teleop_tools_msgs__action__Increment_Feedback__init(&msg->feedback)) {
    teleop_tools_msgs__action__Increment_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
teleop_tools_msgs__action__Increment_FeedbackMessage__fini(teleop_tools_msgs__action__Increment_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  teleop_tools_msgs__action__Increment_Feedback__fini(&msg->feedback);
}

bool
teleop_tools_msgs__action__Increment_FeedbackMessage__are_equal(const teleop_tools_msgs__action__Increment_FeedbackMessage * lhs, const teleop_tools_msgs__action__Increment_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!teleop_tools_msgs__action__Increment_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_FeedbackMessage__copy(
  const teleop_tools_msgs__action__Increment_FeedbackMessage * input,
  teleop_tools_msgs__action__Increment_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!teleop_tools_msgs__action__Increment_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

teleop_tools_msgs__action__Increment_FeedbackMessage *
teleop_tools_msgs__action__Increment_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_FeedbackMessage * msg = (teleop_tools_msgs__action__Increment_FeedbackMessage *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage));
  bool success = teleop_tools_msgs__action__Increment_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teleop_tools_msgs__action__Increment_FeedbackMessage__destroy(teleop_tools_msgs__action__Increment_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teleop_tools_msgs__action__Increment_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__init(teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_FeedbackMessage * data = NULL;

  if (size) {
    data = (teleop_tools_msgs__action__Increment_FeedbackMessage *)allocator.zero_allocate(size, sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teleop_tools_msgs__action__Increment_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teleop_tools_msgs__action__Increment_FeedbackMessage__fini(&data[i - 1]);
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
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__fini(teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * array)
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
      teleop_tools_msgs__action__Increment_FeedbackMessage__fini(&array->data[i]);
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

teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence *
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * array = (teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence *)allocator.allocate(sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__destroy(teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__are_equal(const teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * lhs, const teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teleop_tools_msgs__action__Increment_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence__copy(
  const teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * input,
  teleop_tools_msgs__action__Increment_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teleop_tools_msgs__action__Increment_FeedbackMessage);
    teleop_tools_msgs__action__Increment_FeedbackMessage * data =
      (teleop_tools_msgs__action__Increment_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teleop_tools_msgs__action__Increment_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          teleop_tools_msgs__action__Increment_FeedbackMessage__fini(&data[i]);
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
    if (!teleop_tools_msgs__action__Increment_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
