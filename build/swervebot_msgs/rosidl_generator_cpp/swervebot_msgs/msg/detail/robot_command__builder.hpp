// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swervebot_msgs:msg/RobotCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "swervebot_msgs/msg/robot_command.hpp"


#ifndef SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
#define SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swervebot_msgs/msg/detail/robot_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swervebot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotCommand_mode
{
public:
  explicit Init_RobotCommand_mode(::swervebot_msgs::msg::RobotCommand & msg)
  : msg_(msg)
  {}
  ::swervebot_msgs::msg::RobotCommand mode(::swervebot_msgs::msg::RobotCommand::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swervebot_msgs::msg::RobotCommand msg_;
};

class Init_RobotCommand_twist
{
public:
  Init_RobotCommand_twist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommand_mode twist(::swervebot_msgs::msg::RobotCommand::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_RobotCommand_mode(msg_);
  }

private:
  ::swervebot_msgs::msg::RobotCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swervebot_msgs::msg::RobotCommand>()
{
  return swervebot_msgs::msg::builder::Init_RobotCommand_twist();
}

}  // namespace swervebot_msgs

#endif  // SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__BUILDER_HPP_
