// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:msg/RobotMsg.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/msg/detail/robot_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/msg/detail/robot_msg__struct.h"
#include "xarm_msgs/msg/detail/robot_msg__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // angle
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // angle
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


using _RobotMsg__ros_msg_type = xarm_msgs__msg__RobotMsg;

static bool _RobotMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RobotMsg__ros_msg_type * ros_message = static_cast<const _RobotMsg__ros_msg_type *>(untyped_ros_message);
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

  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  // Field name: cmdnum
  {
    cdr << ros_message->cmdnum;
  }

  // Field name: mt_brake
  {
    cdr << ros_message->mt_brake;
  }

  // Field name: mt_able
  {
    cdr << ros_message->mt_able;
  }

  // Field name: err
  {
    cdr << ros_message->err;
  }

  // Field name: warn
  {
    cdr << ros_message->warn;
  }

  // Field name: angle
  {
    size_t size = ros_message->angle.size;
    auto array_ptr = ros_message->angle.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: pose
  {
    size_t size = 6;
    auto array_ptr = ros_message->pose;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: offset
  {
    size_t size = 6;
    auto array_ptr = ros_message->offset;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _RobotMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RobotMsg__ros_msg_type * ros_message = static_cast<_RobotMsg__ros_msg_type *>(untyped_ros_message);
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

  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  // Field name: cmdnum
  {
    cdr >> ros_message->cmdnum;
  }

  // Field name: mt_brake
  {
    cdr >> ros_message->mt_brake;
  }

  // Field name: mt_able
  {
    cdr >> ros_message->mt_able;
  }

  // Field name: err
  {
    cdr >> ros_message->err;
  }

  // Field name: warn
  {
    cdr >> ros_message->warn;
  }

  // Field name: angle
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->angle.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->angle);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->angle, size)) {
      return "failed to create array for field 'angle'";
    }
    auto array_ptr = ros_message->angle.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: pose
  {
    size_t size = 6;
    auto array_ptr = ros_message->pose;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: offset
  {
    size_t size = 6;
    auto array_ptr = ros_message->offset;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__msg__RobotMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotMsg__ros_msg_type * ros_message = static_cast<const _RobotMsg__ros_msg_type *>(untyped_ros_message);
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
  // field.name mode
  {
    size_t item_size = sizeof(ros_message->mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cmdnum
  {
    size_t item_size = sizeof(ros_message->cmdnum);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mt_brake
  {
    size_t item_size = sizeof(ros_message->mt_brake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mt_able
  {
    size_t item_size = sizeof(ros_message->mt_able);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name err
  {
    size_t item_size = sizeof(ros_message->err);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name warn
  {
    size_t item_size = sizeof(ros_message->warn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angle
  {
    size_t array_size = ros_message->angle.size;
    auto array_ptr = ros_message->angle.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pose
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->pose;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name offset
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->offset;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RobotMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__msg__RobotMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__msg__RobotMsg(
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
  // member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: cmdnum
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: mt_brake
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: mt_able
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: err
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: warn
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: angle
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pose
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: offset
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _RobotMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__msg__RobotMsg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_RobotMsg = {
  "xarm_msgs::msg",
  "RobotMsg",
  _RobotMsg__cdr_serialize,
  _RobotMsg__cdr_deserialize,
  _RobotMsg__get_serialized_size,
  _RobotMsg__max_serialized_size
};

static rosidl_message_type_support_t _RobotMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobotMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, msg, RobotMsg)() {
  return &_RobotMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
