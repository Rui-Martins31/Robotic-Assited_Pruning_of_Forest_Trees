// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/VacuumGripperCtrl.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__VACUUM_GRIPPER_CTRL__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__VACUUM_GRIPPER_CTRL__TRAITS_HPP_

#include "xarm_msgs/srv/detail/vacuum_gripper_ctrl__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::VacuumGripperCtrl_Request>()
{
  return "xarm_msgs::srv::VacuumGripperCtrl_Request";
}

template<>
inline const char * name<xarm_msgs::srv::VacuumGripperCtrl_Request>()
{
  return "xarm_msgs/srv/VacuumGripperCtrl_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::VacuumGripperCtrl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::VacuumGripperCtrl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::VacuumGripperCtrl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::VacuumGripperCtrl_Response>()
{
  return "xarm_msgs::srv::VacuumGripperCtrl_Response";
}

template<>
inline const char * name<xarm_msgs::srv::VacuumGripperCtrl_Response>()
{
  return "xarm_msgs/srv/VacuumGripperCtrl_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::VacuumGripperCtrl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::VacuumGripperCtrl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::VacuumGripperCtrl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::VacuumGripperCtrl>()
{
  return "xarm_msgs::srv::VacuumGripperCtrl";
}

template<>
inline const char * name<xarm_msgs::srv::VacuumGripperCtrl>()
{
  return "xarm_msgs/srv/VacuumGripperCtrl";
}

template<>
struct has_fixed_size<xarm_msgs::srv::VacuumGripperCtrl>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::VacuumGripperCtrl_Request>::value &&
    has_fixed_size<xarm_msgs::srv::VacuumGripperCtrl_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::VacuumGripperCtrl>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::VacuumGripperCtrl_Request>::value &&
    has_bounded_size<xarm_msgs::srv::VacuumGripperCtrl_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::VacuumGripperCtrl>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::VacuumGripperCtrl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::VacuumGripperCtrl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__VACUUM_GRIPPER_CTRL__TRAITS_HPP_
