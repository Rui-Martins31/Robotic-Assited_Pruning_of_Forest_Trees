// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:msg/MoveVelocity.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__TRAITS_HPP_
#define XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__TRAITS_HPP_

#include "xarm_msgs/msg/detail/move_velocity__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::msg::MoveVelocity>()
{
  return "xarm_msgs::msg::MoveVelocity";
}

template<>
inline const char * name<xarm_msgs::msg::MoveVelocity>()
{
  return "xarm_msgs/msg/MoveVelocity";
}

template<>
struct has_fixed_size<xarm_msgs::msg::MoveVelocity>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::msg::MoveVelocity>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::msg::MoveVelocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__MSG__DETAIL__MOVE_VELOCITY__TRAITS_HPP_
