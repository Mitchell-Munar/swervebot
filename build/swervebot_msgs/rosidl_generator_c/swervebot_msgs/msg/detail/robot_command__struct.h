// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swervebot_msgs:msg/RobotCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "swervebot_msgs/msg/robot_command.h"


#ifndef SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__STRUCT_H_
#define SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'ACKERMANN'.
enum
{
  swervebot_msgs__msg__RobotCommand__ACKERMANN = 0
};

/// Constant 'CRAB_WALK'.
enum
{
  swervebot_msgs__msg__RobotCommand__CRAB_WALK = 1
};

/// Constant 'ZERO_TURN'.
enum
{
  swervebot_msgs__msg__RobotCommand__ZERO_TURN = 2
};

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/RobotCommand in the package swervebot_msgs.
typedef struct swervebot_msgs__msg__RobotCommand
{
  geometry_msgs__msg__Twist twist;
  uint8_t mode;
} swervebot_msgs__msg__RobotCommand;

// Struct for a sequence of swervebot_msgs__msg__RobotCommand.
typedef struct swervebot_msgs__msg__RobotCommand__Sequence
{
  swervebot_msgs__msg__RobotCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swervebot_msgs__msg__RobotCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__STRUCT_H_
