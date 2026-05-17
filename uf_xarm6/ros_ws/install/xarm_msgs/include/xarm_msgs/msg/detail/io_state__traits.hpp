// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_
#define XARM_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_

#include "xarm_msgs/msg/detail/io_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::msg::IOState>()
{
  return "xarm_msgs::msg::IOState";
}

template<>
inline const char * name<xarm_msgs::msg::IOState>()
{
  return "xarm_msgs/msg/IOState";
}

template<>
struct has_fixed_size<xarm_msgs::msg::IOState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::msg::IOState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::msg::IOState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__MSG__DETAIL__IO_STATE__TRAITS_HPP_
