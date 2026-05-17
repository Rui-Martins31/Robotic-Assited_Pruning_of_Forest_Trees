// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:msg/IOState.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/msg/detail/io_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/msg/detail/io_state__struct.h"
#include "xarm_msgs/msg/detail/io_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _IOState__ros_msg_type = xarm_msgs__msg__IOState;

static bool _IOState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IOState__ros_msg_type * ros_message = static_cast<const _IOState__ros_msg_type *>(untyped_ros_message);
  // Field name: digital_1
  {
    cdr << ros_message->digital_1;
  }

  // Field name: digital_2
  {
    cdr << ros_message->digital_2;
  }

  // Field name: analog_1
  {
    cdr << ros_message->analog_1;
  }

  // Field name: analog_2
  {
    cdr << ros_message->analog_2;
  }

  return true;
}

static bool _IOState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IOState__ros_msg_type * ros_message = static_cast<_IOState__ros_msg_type *>(untyped_ros_message);
  // Field name: digital_1
  {
    cdr >> ros_message->digital_1;
  }

  // Field name: digital_2
  {
    cdr >> ros_message->digital_2;
  }

  // Field name: analog_1
  {
    cdr >> ros_message->analog_1;
  }

  // Field name: analog_2
  {
    cdr >> ros_message->analog_2;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__msg__IOState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IOState__ros_msg_type * ros_message = static_cast<const _IOState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name digital_1
  {
    size_t item_size = sizeof(ros_message->digital_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name digital_2
  {
    size_t item_size = sizeof(ros_message->digital_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name analog_1
  {
    size_t item_size = sizeof(ros_message->analog_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name analog_2
  {
    size_t item_size = sizeof(ros_message->analog_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IOState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__msg__IOState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__msg__IOState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: digital_1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: digital_2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: analog_1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: analog_2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _IOState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__msg__IOState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IOState = {
  "xarm_msgs::msg",
  "IOState",
  _IOState__cdr_serialize,
  _IOState__cdr_deserialize,
  _IOState__get_serialized_size,
  _IOState__max_serialized_size
};

static rosidl_message_type_support_t _IOState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IOState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, msg, IOState)() {
  return &_IOState__type_support;
}

#if defined(__cplusplus)
}
#endif
