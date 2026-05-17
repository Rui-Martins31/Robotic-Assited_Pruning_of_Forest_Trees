// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/PlanSingleStraight.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__PLAN_SINGLE_STRAIGHT__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__PLAN_SINGLE_STRAIGHT__TRAITS_HPP_

#include "xarm_msgs/srv/detail/plan_single_straight__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanSingleStraight_Request>()
{
  return "xarm_msgs::srv::PlanSingleStraight_Request";
}

template<>
inline const char * name<xarm_msgs::srv::PlanSingleStraight_Request>()
{
  return "xarm_msgs/srv/PlanSingleStraight_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanSingleStraight_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanSingleStraight_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<xarm_msgs::srv::PlanSingleStraight_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanSingleStraight_Response>()
{
  return "xarm_msgs::srv::PlanSingleStraight_Response";
}

template<>
inline const char * name<xarm_msgs::srv::PlanSingleStraight_Response>()
{
  return "xarm_msgs/srv/PlanSingleStraight_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanSingleStraight_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanSingleStraight_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::PlanSingleStraight_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanSingleStraight>()
{
  return "xarm_msgs::srv::PlanSingleStraight";
}

template<>
inline const char * name<xarm_msgs::srv::PlanSingleStraight>()
{
  return "xarm_msgs/srv/PlanSingleStraight";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanSingleStraight>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::PlanSingleStraight_Request>::value &&
    has_fixed_size<xarm_msgs::srv::PlanSingleStraight_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanSingleStraight>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::PlanSingleStraight_Request>::value &&
    has_bounded_size<xarm_msgs::srv::PlanSingleStraight_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::PlanSingleStraight>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::PlanSingleStraight_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::PlanSingleStraight_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__PLAN_SINGLE_STRAIGHT__TRAITS_HPP_
