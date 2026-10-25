// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from swervebot_msgs:msg/RobotCommand.idl
// generated code does not contain a copyright notice

#ifndef SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "swervebot_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "swervebot_msgs/msg/detail/robot_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace swervebot_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
cdr_serialize(
  const swervebot_msgs::msg::RobotCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  swervebot_msgs::msg::RobotCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
get_serialized_size(
  const swervebot_msgs::msg::RobotCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
max_serialized_size_RobotCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
cdr_serialize_key(
  const swervebot_msgs::msg::RobotCommand & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
get_serialized_size_key(
  const swervebot_msgs::msg::RobotCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
max_serialized_size_key_RobotCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
has_buffer_fields_RobotCommand();

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace swervebot_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swervebot_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, swervebot_msgs, msg, RobotCommand)();

#ifdef __cplusplus
}
#endif

#endif  // SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
