// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/SetInt32ByType.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__SET_INT32_BY_TYPE__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__SET_INT32_BY_TYPE__TRAITS_HPP_

#include "xarm_msgs/srv/detail/set_int32_by_type__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetInt32ByType_Request>()
{
  return "xarm_msgs::srv::SetInt32ByType_Request";
}

template<>
inline const char * name<xarm_msgs::srv::SetInt32ByType_Request>()
{
  return "xarm_msgs/srv/SetInt32ByType_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetInt32ByType_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::SetInt32ByType_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::SetInt32ByType_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetInt32ByType_Response>()
{
  return "xarm_msgs::srv::SetInt32ByType_Response";
}

template<>
inline const char * name<xarm_msgs::srv::SetInt32ByType_Response>()
{
  return "xarm_msgs/srv/SetInt32ByType_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetInt32ByType_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::SetInt32ByType_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::SetInt32ByType_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::SetInt32ByType>()
{
  return "xarm_msgs::srv::SetInt32ByType";
}

template<>
inline const char * name<xarm_msgs::srv::SetInt32ByType>()
{
  return "xarm_msgs/srv/SetInt32ByType";
}

template<>
struct has_fixed_size<xarm_msgs::srv::SetInt32ByType>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::SetInt32ByType_Request>::value &&
    has_fixed_size<xarm_msgs::srv::SetInt32ByType_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::SetInt32ByType>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::SetInt32ByType_Request>::value &&
    has_bounded_size<xarm_msgs::srv::SetInt32ByType_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::SetInt32ByType>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::SetInt32ByType_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::SetInt32ByType_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__SET_INT32_BY_TYPE__TRAITS_HPP_
