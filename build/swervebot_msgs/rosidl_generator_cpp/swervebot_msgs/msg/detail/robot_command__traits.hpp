// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swervebot_msgs:msg/RobotCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "swervebot_msgs/msg/robot_command.hpp"


#ifndef SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__TRAITS_HPP_
#define SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <array>
#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "swervebot_msgs/msg/detail/robot_command__struct.hpp"
#include "rosidl_runtime_cpp/buffer__traits.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace swervebot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

template<typename T, std::enable_if_t<std::is_same_v<std::decay_t<T>, swervebot_msgs::msg::RobotCommand>, int> = 0>
constexpr auto as_tuple_ref(T && msg)
{
  return std::forward_as_tuple(
    std::forward<T>(msg).twist,
    std::forward<T>(msg).mode);
}

}  // namespace msg

}  // namespace swervebot_msgs

namespace rosidl_generator_traits
{

template<>
constexpr const char * data_type<swervebot_msgs::msg::RobotCommand>()
{
  return "swervebot_msgs::msg::RobotCommand";
}

template<>
constexpr const char * name<swervebot_msgs::msg::RobotCommand>()
{
  return "swervebot_msgs/msg/RobotCommand";
}

template<>
struct has_fixed_size<swervebot_msgs::msg::RobotCommand>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<swervebot_msgs::msg::RobotCommand>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<swervebot_msgs::msg::RobotCommand>
  : std::true_type {};

template<>
struct MessageTraits<swervebot_msgs::msg::RobotCommand>
{
  static constexpr std::size_t member_count = 2;
  static constexpr std::array<std::string_view, member_count> member_names = {
    "twist",
    "mode",
  };
};

}  // namespace rosidl_generator_traits

#endif  // SWERVEBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND__TRAITS_HPP_
