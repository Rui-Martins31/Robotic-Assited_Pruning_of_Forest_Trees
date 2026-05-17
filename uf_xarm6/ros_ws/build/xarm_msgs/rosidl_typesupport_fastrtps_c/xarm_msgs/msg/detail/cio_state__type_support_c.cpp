// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:msg/CIOState.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/msg/detail/cio_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/msg/detail/cio_state__struct.h"
#include "xarm_msgs/msg/detail/cio_state__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xarm_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xarm_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_xarm_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _CIOState__ros_msg_type = xarm_msgs__msg__CIOState;

static bool _CIOState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CIOState__ros_msg_type * ros_message = static_cast<const _CIOState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: state
  {
    cdr << ros_message->state;
  }

  // Field name: code
  {
    cdr << ros_message->code;
  }

  // Field name: input_digitals
  {
    size_t size = 2;
    auto array_ptr = ros_message->input_digitals;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: output_digitals
  {
    size_t size = 2;
    auto array_ptr = ros_message->output_digitals;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: input_analogs
  {
    size_t size = 2;
    auto array_ptr = ros_message->input_analogs;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: output_analogs
  {
    size_t size = 2;
    auto array_ptr = ros_message->output_analogs;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: input_conf
  {
    size_t size = 16;
    auto array_ptr = ros_message->input_conf;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: output_conf
  {
    size_t size = 16;
    auto array_ptr = ros_message->output_conf;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _CIOState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CIOState__ros_msg_type * ros_message = static_cast<_CIOState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: state
  {
    cdr >> ros_message->state;
  }

  // Field name: code
  {
    cdr >> ros_message->code;
  }

  // Field name: input_digitals
  {
    size_t size = 2;
    auto array_ptr = ros_message->input_digitals;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: output_digitals
  {
    size_t size = 2;
    auto array_ptr = ros_message->output_digitals;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: input_analogs
  {
    size_t size = 2;
    auto array_ptr = ros_message->input_analogs;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: output_analogs
  {
    size_t size = 2;
    auto array_ptr = ros_message->output_analogs;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: input_conf
  {
    size_t size = 16;
    auto array_ptr = ros_message->input_conf;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: output_conf
  {
    size_t size = 16;
    auto array_ptr = ros_message->output_conf;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__msg__CIOState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CIOState__ros_msg_type * ros_message = static_cast<const _CIOState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name state
  {
    size_t item_size = sizeof(ros_message->state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name code
  {
    size_t item_size = sizeof(ros_message->code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input_digitals
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->input_digitals;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_digitals
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->output_digitals;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input_analogs
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->input_analogs;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_analogs
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->output_analogs;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input_conf
  {
    size_t array_size = 16;
    auto array_ptr = ros_message->input_conf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_conf
  {
    size_t array_size = 16;
    auto array_ptr = ros_message->output_conf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CIOState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__msg__CIOState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__msg__CIOState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: code
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: input_digitals
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: output_digitals
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: input_analogs
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: output_analogs
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: input_conf
  {
    size_t array_size = 16;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: output_conf
  {
    size_t array_size = 16;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _CIOState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__msg__CIOState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CIOState = {
  "xarm_msgs::msg",
  "CIOState",
  _CIOState__cdr_serialize,
  _CIOState__cdr_deserialize,
  _CIOState__get_serialized_size,
  _CIOState__max_serialized_size
};

static rosidl_message_type_support_t _CIOState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CIOState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, msg, CIOState)() {
  return &_CIOState__type_support;
}

#if defined(__cplusplus)
}
#endif
