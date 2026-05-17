// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xarm_msgs:msg/RobotMsg.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__ROBOT_MSG__STRUCT_H_
#define XARM_MSGS__MSG__DETAIL__ROBOT_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'angle'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/RobotMsg in the package xarm_msgs.
typedef struct xarm_msgs__msg__RobotMsg
{
  std_msgs__msg__Header header;
  int16_t state;
  int16_t mode;
  int16_t cmdnum;
  int16_t mt_brake;
  int16_t mt_able;
  int16_t err;
  int16_t warn;
  rosidl_runtime_c__float__Sequence angle;
  float pose[6];
  float offset[6];
} xarm_msgs__msg__RobotMsg;

// Struct for a sequence of xarm_msgs__msg__RobotMsg.
typedef struct xarm_msgs__msg__RobotMsg__Sequence
{
  xarm_msgs__msg__RobotMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__msg__RobotMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__MSG__DETAIL__ROBOT_MSG__STRUCT_H_
