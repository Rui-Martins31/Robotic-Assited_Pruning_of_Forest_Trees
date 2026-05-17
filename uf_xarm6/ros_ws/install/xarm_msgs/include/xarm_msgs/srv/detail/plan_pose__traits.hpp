// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/PlanPose.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__PLAN_POSE__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__PLAN_POSE__TRAITS_HPP_

#include "xarm_msgs/srv/detail/plan_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'target'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanPose_Request>()
{
  return "xarm_msgs::srv::PlanPose_Request";
}

template<>
inline const char * name<xarm_msgs::srv::PlanPose_Request>()
{
  return "xarm_msgs/srv/PlanPose_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<xarm_msgs::srv::PlanPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanPose_Response>()
{
  return "xarm_msgs::srv::PlanPose_Response";
}

template<>
inline const char * name<xarm_msgs::srv::PlanPose_Response>()
{
  return "xarm_msgs/srv/PlanPose_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanPose_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::PlanPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::PlanPose>()
{
  return "xarm_msgs::srv::PlanPose";
}

template<>
inline const char * name<xarm_msgs::srv::PlanPose>()
{
  return "xarm_msgs/srv/PlanPose";
}

template<>
struct has_fixed_size<xarm_msgs::srv::PlanPose>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::PlanPose_Request>::value &&
    has_fixed_size<xarm_msgs::srv::PlanPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::PlanPose>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::PlanPose_Request>::value &&
    has_bounded_size<xarm_msgs::srv::PlanPose_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::PlanPose>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::PlanPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::PlanPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__PLAN_POSE__TRAITS_HPP_
