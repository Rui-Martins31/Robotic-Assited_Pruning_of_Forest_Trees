// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/RobotiqMove.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__ROBOTIQ_MOVE__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__ROBOTIQ_MOVE__TRAITS_HPP_

#include "xarm_msgs/srv/detail/robotiq_move__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::RobotiqMove_Request>()
{
  return "xarm_msgs::srv::RobotiqMove_Request";
}

template<>
inline const char * name<xarm_msgs::srv::RobotiqMove_Request>()
{
  return "xarm_msgs/srv/RobotiqMove_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::RobotiqMove_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::RobotiqMove_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::RobotiqMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::RobotiqMove_Response>()
{
  return "xarm_msgs::srv::RobotiqMove_Response";
}

template<>
inline const char * name<xarm_msgs::srv::RobotiqMove_Response>()
{
  return "xarm_msgs/srv/RobotiqMove_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::RobotiqMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::RobotiqMove_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::RobotiqMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::RobotiqMove>()
{
  return "xarm_msgs::srv::RobotiqMove";
}

template<>
inline const char * name<xarm_msgs::srv::RobotiqMove>()
{
  return "xarm_msgs/srv/RobotiqMove";
}

template<>
struct has_fixed_size<xarm_msgs::srv::RobotiqMove>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::RobotiqMove_Request>::value &&
    has_fixed_size<xarm_msgs::srv::RobotiqMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::RobotiqMove>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::RobotiqMove_Request>::value &&
    has_bounded_size<xarm_msgs::srv::RobotiqMove_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::RobotiqMove>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::RobotiqMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::RobotiqMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__ROBOTIQ_MOVE__TRAITS_HPP_
