// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:msg/RobotMsg.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__ROBOT_MSG__TRAITS_HPP_
#define XARM_MSGS__MSG__DETAIL__ROBOT_MSG__TRAITS_HPP_

#include "xarm_msgs/msg/detail/robot_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::msg::RobotMsg>()
{
  return "xarm_msgs::msg::RobotMsg";
}

template<>
inline const char * name<xarm_msgs::msg::RobotMsg>()
{
  return "xarm_msgs/msg/RobotMsg";
}

template<>
struct has_fixed_size<xarm_msgs::msg::RobotMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::msg::RobotMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::msg::RobotMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__MSG__DETAIL__ROBOT_MSG__TRAITS_HPP_
