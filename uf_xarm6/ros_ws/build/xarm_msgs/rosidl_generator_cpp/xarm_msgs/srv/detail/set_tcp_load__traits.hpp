// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/SetTcpLoad.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__SET_TCP_LOAD__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__SET_TCP_LOAD__TRAITS_HPP_

#include "xarm_msgs/srv/detail/set_tcp_load__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetTcpLoad_Request>()
{
  return "xarm_msgs::srv::SetTcpLoad_Request";
}

template<>
inline const char * name<xarm_msgs::srv::SetTcpLoad_Request>()
{
  return "xarm_msgs/srv/SetTcpLoad_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetTcpLoad_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::SetTcpLoad_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::SetTcpLoad_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetTcpLoad_Response>()
{
  return "xarm_msgs::srv::SetTcpLoad_Response";
}

template<>
inline const char * name<xarm_msgs::srv::SetTcpLoad_Response>()
{
  return "xarm_msgs/srv/SetTcpLoad_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetTcpLoad_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::SetTcpLoad_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::SetTcpLoad_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetTcpLoad>()
{
  return "xarm_msgs::srv::SetTcpLoad";
}

template<>
inline const char * name<xarm_msgs::srv::SetTcpLoad>()
{
  return "xarm_msgs/srv/SetTcpLoad";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetTcpLoad>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::SetTcpLoad_Request>::value &&
    has_fixed_size<xarm_msgs::srv::SetTcpLoad_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::SetTcpLoad>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::SetTcpLoad_Request>::value &&
    has_bounded_size<xarm_msgs::srv::SetTcpLoad_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::SetTcpLoad>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::SetTcpLoad_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::SetTcpLoad_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__SET_TCP_LOAD__TRAITS_HPP_
