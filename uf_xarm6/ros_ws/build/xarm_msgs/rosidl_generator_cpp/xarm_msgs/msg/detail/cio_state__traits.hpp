// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:msg/CIOState.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__CIO_STATE__TRAITS_HPP_
#define XARM_MSGS__MSG__DETAIL__CIO_STATE__TRAITS_HPP_

#include "xarm_msgs/msg/detail/cio_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::msg::CIOState>()
{
  return "xarm_msgs::msg::CIOState";
}

template<>
inline const char * name<xarm_msgs::msg::CIOState>()
{
  return "xarm_msgs/msg/CIOState";
}

template<>
struct has_fixed_size<xarm_msgs::msg::CIOState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<xarm_msgs::msg::CIOState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<xarm_msgs::msg::CIOState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__MSG__DETAIL__CIO_STATE__TRAITS_HPP_
