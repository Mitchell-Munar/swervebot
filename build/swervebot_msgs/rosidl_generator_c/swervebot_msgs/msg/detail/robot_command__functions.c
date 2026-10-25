// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swervebot_msgs:msg/RobotCommand.idl
// generated code does not contain a copyright notice
#include "swervebot_msgs/msg/detail/robot_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
swervebot_msgs__msg__RobotCommand__init(swervebot_msgs__msg__RobotCommand * msg)
{
  if (!msg) {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    swervebot_msgs__msg__RobotCommand__fini(msg);
    return false;
  }
  // mode
  return true;
}

void
swervebot_msgs__msg__RobotCommand__fini(swervebot_msgs__msg__RobotCommand * msg)
{
  if (!msg) {
    return;
  }
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // mode
}

bool
swervebot_msgs__msg__RobotCommand__are_equal(const swervebot_msgs__msg__RobotCommand * lhs, const swervebot_msgs__msg__RobotCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
swervebot_msgs__msg__RobotCommand__copy(
  const swervebot_msgs__msg__RobotCommand * input,
  swervebot_msgs__msg__RobotCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // mode
  output->mode = input->mode;
  return true;
}

swervebot_msgs__msg__RobotCommand *
swervebot_msgs__msg__RobotCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swervebot_msgs__msg__RobotCommand * msg = (swervebot_msgs__msg__RobotCommand *)allocator.allocate(sizeof(swervebot_msgs__msg__RobotCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swervebot_msgs__msg__RobotCommand));
  bool success = swervebot_msgs__msg__RobotCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swervebot_msgs__msg__RobotCommand__destroy(swervebot_msgs__msg__RobotCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swervebot_msgs__msg__RobotCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swervebot_msgs__msg__RobotCommand__Sequence__init(swervebot_msgs__msg__RobotCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swervebot_msgs__msg__RobotCommand * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(swervebot_msgs__msg__RobotCommand)) {
      return false;
    }
    data = (swervebot_msgs__msg__RobotCommand *)allocator.zero_allocate(size, sizeof(swervebot_msgs__msg__RobotCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swervebot_msgs__msg__RobotCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swervebot_msgs__msg__RobotCommand__fini(&data[i - 1]);
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
swervebot_msgs__msg__RobotCommand__Sequence__fini(swervebot_msgs__msg__RobotCommand__Sequence * array)
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
      swervebot_msgs__msg__RobotCommand__fini(&array->data[i]);
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

swervebot_msgs__msg__RobotCommand__Sequence *
swervebot_msgs__msg__RobotCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swervebot_msgs__msg__RobotCommand__Sequence * array = (swervebot_msgs__msg__RobotCommand__Sequence *)allocator.allocate(sizeof(swervebot_msgs__msg__RobotCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swervebot_msgs__msg__RobotCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swervebot_msgs__msg__RobotCommand__Sequence__destroy(swervebot_msgs__msg__RobotCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swervebot_msgs__msg__RobotCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swervebot_msgs__msg__RobotCommand__Sequence__are_equal(const swervebot_msgs__msg__RobotCommand__Sequence * lhs, const swervebot_msgs__msg__RobotCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swervebot_msgs__msg__RobotCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swervebot_msgs__msg__RobotCommand__Sequence__copy(
  const swervebot_msgs__msg__RobotCommand__Sequence * input,
  swervebot_msgs__msg__RobotCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(swervebot_msgs__msg__RobotCommand)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(swervebot_msgs__msg__RobotCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swervebot_msgs__msg__RobotCommand * data =
      (swervebot_msgs__msg__RobotCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swervebot_msgs__msg__RobotCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swervebot_msgs__msg__RobotCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swervebot_msgs__msg__RobotCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
