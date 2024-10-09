// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice
#include "vesc_msgs/msg/detail/vesc_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vesc_msgs__msg__VescState__init(vesc_msgs__msg__VescState * msg)
{
  if (!msg) {
    return false;
  }
  // temp_fet
  // temp_motor
  // current_motor
  // current_input
  // avg_id
  // avg_iq
  // duty_cycle
  // speed
  // voltage_input
  // charge_drawn
  // charge_regen
  // energy_drawn
  // energy_regen
  // displacement
  // distance_traveled
  // fault_code
  // pid_pos_now
  // controller_id
  // ntc_temp_mos1
  // ntc_temp_mos2
  // ntc_temp_mos3
  // avg_vd
  // avg_vq
  return true;
}

void
vesc_msgs__msg__VescState__fini(vesc_msgs__msg__VescState * msg)
{
  if (!msg) {
    return;
  }
  // temp_fet
  // temp_motor
  // current_motor
  // current_input
  // avg_id
  // avg_iq
  // duty_cycle
  // speed
  // voltage_input
  // charge_drawn
  // charge_regen
  // energy_drawn
  // energy_regen
  // displacement
  // distance_traveled
  // fault_code
  // pid_pos_now
  // controller_id
  // ntc_temp_mos1
  // ntc_temp_mos2
  // ntc_temp_mos3
  // avg_vd
  // avg_vq
}

bool
vesc_msgs__msg__VescState__are_equal(const vesc_msgs__msg__VescState * lhs, const vesc_msgs__msg__VescState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // temp_fet
  if (lhs->temp_fet != rhs->temp_fet) {
    return false;
  }
  // temp_motor
  if (lhs->temp_motor != rhs->temp_motor) {
    return false;
  }
  // current_motor
  if (lhs->current_motor != rhs->current_motor) {
    return false;
  }
  // current_input
  if (lhs->current_input != rhs->current_input) {
    return false;
  }
  // avg_id
  if (lhs->avg_id != rhs->avg_id) {
    return false;
  }
  // avg_iq
  if (lhs->avg_iq != rhs->avg_iq) {
    return false;
  }
  // duty_cycle
  if (lhs->duty_cycle != rhs->duty_cycle) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // voltage_input
  if (lhs->voltage_input != rhs->voltage_input) {
    return false;
  }
  // charge_drawn
  if (lhs->charge_drawn != rhs->charge_drawn) {
    return false;
  }
  // charge_regen
  if (lhs->charge_regen != rhs->charge_regen) {
    return false;
  }
  // energy_drawn
  if (lhs->energy_drawn != rhs->energy_drawn) {
    return false;
  }
  // energy_regen
  if (lhs->energy_regen != rhs->energy_regen) {
    return false;
  }
  // displacement
  if (lhs->displacement != rhs->displacement) {
    return false;
  }
  // distance_traveled
  if (lhs->distance_traveled != rhs->distance_traveled) {
    return false;
  }
  // fault_code
  if (lhs->fault_code != rhs->fault_code) {
    return false;
  }
  // pid_pos_now
  if (lhs->pid_pos_now != rhs->pid_pos_now) {
    return false;
  }
  // controller_id
  if (lhs->controller_id != rhs->controller_id) {
    return false;
  }
  // ntc_temp_mos1
  if (lhs->ntc_temp_mos1 != rhs->ntc_temp_mos1) {
    return false;
  }
  // ntc_temp_mos2
  if (lhs->ntc_temp_mos2 != rhs->ntc_temp_mos2) {
    return false;
  }
  // ntc_temp_mos3
  if (lhs->ntc_temp_mos3 != rhs->ntc_temp_mos3) {
    return false;
  }
  // avg_vd
  if (lhs->avg_vd != rhs->avg_vd) {
    return false;
  }
  // avg_vq
  if (lhs->avg_vq != rhs->avg_vq) {
    return false;
  }
  return true;
}

bool
vesc_msgs__msg__VescState__copy(
  const vesc_msgs__msg__VescState * input,
  vesc_msgs__msg__VescState * output)
{
  if (!input || !output) {
    return false;
  }
  // temp_fet
  output->temp_fet = input->temp_fet;
  // temp_motor
  output->temp_motor = input->temp_motor;
  // current_motor
  output->current_motor = input->current_motor;
  // current_input
  output->current_input = input->current_input;
  // avg_id
  output->avg_id = input->avg_id;
  // avg_iq
  output->avg_iq = input->avg_iq;
  // duty_cycle
  output->duty_cycle = input->duty_cycle;
  // speed
  output->speed = input->speed;
  // voltage_input
  output->voltage_input = input->voltage_input;
  // charge_drawn
  output->charge_drawn = input->charge_drawn;
  // charge_regen
  output->charge_regen = input->charge_regen;
  // energy_drawn
  output->energy_drawn = input->energy_drawn;
  // energy_regen
  output->energy_regen = input->energy_regen;
  // displacement
  output->displacement = input->displacement;
  // distance_traveled
  output->distance_traveled = input->distance_traveled;
  // fault_code
  output->fault_code = input->fault_code;
  // pid_pos_now
  output->pid_pos_now = input->pid_pos_now;
  // controller_id
  output->controller_id = input->controller_id;
  // ntc_temp_mos1
  output->ntc_temp_mos1 = input->ntc_temp_mos1;
  // ntc_temp_mos2
  output->ntc_temp_mos2 = input->ntc_temp_mos2;
  // ntc_temp_mos3
  output->ntc_temp_mos3 = input->ntc_temp_mos3;
  // avg_vd
  output->avg_vd = input->avg_vd;
  // avg_vq
  output->avg_vq = input->avg_vq;
  return true;
}

vesc_msgs__msg__VescState *
vesc_msgs__msg__VescState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescState * msg = (vesc_msgs__msg__VescState *)allocator.allocate(sizeof(vesc_msgs__msg__VescState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vesc_msgs__msg__VescState));
  bool success = vesc_msgs__msg__VescState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vesc_msgs__msg__VescState__destroy(vesc_msgs__msg__VescState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vesc_msgs__msg__VescState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vesc_msgs__msg__VescState__Sequence__init(vesc_msgs__msg__VescState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescState * data = NULL;

  if (size) {
    data = (vesc_msgs__msg__VescState *)allocator.zero_allocate(size, sizeof(vesc_msgs__msg__VescState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vesc_msgs__msg__VescState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vesc_msgs__msg__VescState__fini(&data[i - 1]);
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
vesc_msgs__msg__VescState__Sequence__fini(vesc_msgs__msg__VescState__Sequence * array)
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
      vesc_msgs__msg__VescState__fini(&array->data[i]);
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

vesc_msgs__msg__VescState__Sequence *
vesc_msgs__msg__VescState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vesc_msgs__msg__VescState__Sequence * array = (vesc_msgs__msg__VescState__Sequence *)allocator.allocate(sizeof(vesc_msgs__msg__VescState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vesc_msgs__msg__VescState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vesc_msgs__msg__VescState__Sequence__destroy(vesc_msgs__msg__VescState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vesc_msgs__msg__VescState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vesc_msgs__msg__VescState__Sequence__are_equal(const vesc_msgs__msg__VescState__Sequence * lhs, const vesc_msgs__msg__VescState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vesc_msgs__msg__VescState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vesc_msgs__msg__VescState__Sequence__copy(
  const vesc_msgs__msg__VescState__Sequence * input,
  vesc_msgs__msg__VescState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vesc_msgs__msg__VescState);
    vesc_msgs__msg__VescState * data =
      (vesc_msgs__msg__VescState *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vesc_msgs__msg__VescState__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          vesc_msgs__msg__VescState__fini(&data[i]);
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
    if (!vesc_msgs__msg__VescState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
