// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xarm_msgs:srv/BioGripperEnable.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__BIO_GRIPPER_ENABLE__TRAITS_HPP_
#define XARM_MSGS__SRV__DETAIL__BIO_GRIPPER_ENABLE__TRAITS_HPP_

#include "xarm_msgs/srv/detail/bio_gripper_enable__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::BioGripperEnable_Request>()
{
  return "xarm_msgs::srv::BioGripperEnable_Request";
}

template<>
inline const char * name<xarm_msgs::srv::BioGripperEnable_Request>()
{
  return "xarm_msgs/srv/BioGripperEnable_Request";
}

template<>
struct has_fixed_size<xarm_msgs::srv::BioGripperEnable_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xarm_msgs::srv::BioGripperEnable_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xarm_msgs::srv::BioGripperEnable_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::BioGripperEnable_Response>()
{
  return "xarm_msgs::srv::BioGripperEnable_Response";
}

template<>
inline const char * name<xarm_msgs::srv::BioGripperEnable_Response>()
{
  return "xarm_msgs/srv/BioGripperEnable_Response";
}

template<>
struct has_fixed_size<xarm_msgs::srv::BioGripperEnable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xarm_msgs::srv::BioGripperEnable_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xarm_msgs::srv::BioGripperEnable_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xarm_msgs::srv::BioGripperEnable>()
{
  return "xarm_msgs::srv::BioGripperEnable";
}

template<>
inline const char * name<xarm_msgs::srv::BioGripperEnable>()
{
  return "xarm_msgs/srv/BioGripperEnable";
}

template<>
struct has_fixed_size<xarm_msgs::srv::BioGripperEnable>
  : std::integral_constant<
    bool,
    has_fixed_size<xarm_msgs::srv::BioGripperEnable_Request>::value &&
    has_fixed_size<xarm_msgs::srv::BioGripperEnable_Response>::value
  >
{
};

template<>
struct has_bounded_size<xarm_msgs::srv::BioGripperEnable>
  : std::integral_constant<
    bool,
    has_bounded_size<xarm_msgs::srv::BioGripperEnable_Request>::value &&
    has_bounded_size<xarm_msgs::srv::BioGripperEnable_Response>::value
  >
{
};

template<>
struct is_service<xarm_msgs::srv::BioGripperEnable>
  : std::true_type
{
};

template<>
struct is_service_request<xarm_msgs::srv::BioGripperEnable_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xarm_msgs::srv::BioGripperEnable_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XARM_MSGS__SRV__DETAIL__BIO_GRIPPER_ENABLE__TRAITS_HPP_
