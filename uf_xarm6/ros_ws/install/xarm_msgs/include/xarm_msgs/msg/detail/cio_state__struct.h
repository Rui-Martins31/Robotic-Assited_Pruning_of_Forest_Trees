// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xarm_msgs:msg/CIOState.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__MSG__DETAIL__CIO_STATE__STRUCT_H_
#define XARM_MSGS__MSG__DETAIL__CIO_STATE__STRUCT_H_

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

// Struct defined in msg/CIOState in the package xarm_msgs.
typedef struct xarm_msgs__msg__CIOState
{
  std_msgs__msg__Header header;
  int16_t state;
  int16_t code;
  uint16_t input_digitals[2];
  uint16_t output_digitals[2];
  float input_analogs[2];
  float output_analogs[2];
  uint8_t input_conf[16];
  uint8_t output_conf[16];
} xarm_msgs__msg__CIOState;

// Struct for a sequence of xarm_msgs__msg__CIOState.
typedef struct xarm_msgs__msg__CIOState__Sequence
{
  xarm_msgs__msg__CIOState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__msg__CIOState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__MSG__DETAIL__CIO_STATE__STRUCT_H_
