// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:srv/FtForceParams.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/srv/detail/ft_force_params__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/srv/detail/ft_force_params__struct.h"
#include "xarm_msgs/srv/detail/ft_force_params__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // c_axis, kd, ki, kp, limits, ref, xe_limit
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // c_axis, kd, ki, kp, limits, ref, xe_limit

// forward declare type support functions


using _FtForceParams_Request__ros_msg_type = xarm_msgs__srv__FtForceParams_Request;

static bool _FtForceParams_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FtForceParams_Request__ros_msg_type * ros_message = static_cast<const _FtForceParams_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: coord
  {
    cdr << ros_message->coord;
  }

  // Field name: c_axis
  {
    size_t size = ros_message->c_axis.size;
    auto array_ptr = ros_message->c_axis.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: ref
  {
    size_t size = ros_message->ref.size;
    auto array_ptr = ros_message->ref.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: limits
  {
    size_t size = ros_message->limits.size;
    auto array_ptr = ros_message->limits.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: kp
  {
    size_t size = ros_message->kp.size;
    auto array_ptr = ros_message->kp.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: ki
  {
    size_t size = ros_message->ki.size;
    auto array_ptr = ros_message->ki.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: kd
  {
    size_t size = ros_message->kd.size;
    auto array_ptr = ros_message->kd.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: xe_limit
  {
    size_t size = ros_message->xe_limit.size;
    auto array_ptr = ros_message->xe_limit.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _FtForceParams_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FtForceParams_Request__ros_msg_type * ros_message = static_cast<_FtForceParams_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: coord
  {
    cdr >> ros_message->coord;
  }

  // Field name: c_axis
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->c_axis.data) {
      rosidl_runtime_c__int16__Sequence__fini(&ros_message->c_axis);
    }
    if (!rosidl_runtime_c__int16__Sequence__init(&ros_message->c_axis, size)) {
      return "failed to create array for field 'c_axis'";
    }
    auto array_ptr = ros_message->c_axis.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: ref
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->ref.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->ref);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->ref, size)) {
      return "failed to create array for field 'ref'";
    }
    auto array_ptr = ros_message->ref.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: limits
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->limits.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->limits);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->limits, size)) {
      return "failed to create array for field 'limits'";
    }
    auto array_ptr = ros_message->limits.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: kp
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->kp.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->kp);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->kp, size)) {
      return "failed to create array for field 'kp'";
    }
    auto array_ptr = ros_message->kp.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: ki
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->ki.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->ki);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->ki, size)) {
      return "failed to create array for field 'ki'";
    }
    auto array_ptr = ros_message->ki.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: kd
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->kd.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->kd);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->kd, size)) {
      return "failed to create array for field 'kd'";
    }
    auto array_ptr = ros_message->kd.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: xe_limit
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->xe_limit.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->xe_limit);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->xe_limit, size)) {
      return "failed to create array for field 'xe_limit'";
    }
    auto array_ptr = ros_message->xe_limit.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__srv__FtForceParams_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FtForceParams_Request__ros_msg_type * ros_message = static_cast<const _FtForceParams_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name coord
  {
    size_t item_size = sizeof(ros_message->coord);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name c_axis
  {
    size_t array_size = ros_message->c_axis.size;
    auto array_ptr = ros_message->c_axis.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ref
  {
    size_t array_size = ros_message->ref.size;
    auto array_ptr = ros_message->ref.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name limits
  {
    size_t array_size = ros_message->limits.size;
    auto array_ptr = ros_message->limits.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp
  {
    size_t array_size = ros_message->kp.size;
    auto array_ptr = ros_message->kp.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ki
  {
    size_t array_size = ros_message->ki.size;
    auto array_ptr = ros_message->ki.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kd
  {
    size_t array_size = ros_message->kd.size;
    auto array_ptr = ros_message->kd.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name xe_limit
  {
    size_t array_size = ros_message->xe_limit.size;
    auto array_ptr = ros_message->xe_limit.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FtForceParams_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__srv__FtForceParams_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__srv__FtForceParams_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: coord
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: c_axis
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: ref
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: limits
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: ki
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kd
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: xe_limit
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _FtForceParams_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__srv__FtForceParams_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_FtForceParams_Request = {
  "xarm_msgs::srv",
  "FtForceParams_Request",
  _FtForceParams_Request__cdr_serialize,
  _FtForceParams_Request__cdr_deserialize,
  _FtForceParams_Request__get_serialized_size,
  _FtForceParams_Request__max_serialized_size
};

static rosidl_message_type_support_t _FtForceParams_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FtForceParams_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, FtForceParams_Request)() {
  return &_FtForceParams_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "xarm_msgs/srv/detail/ft_force_params__struct.h"
// already included above
// #include "xarm_msgs/srv/detail/ft_force_params__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "rosidl_runtime_c/string.h"  // message
#include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _FtForceParams_Response__ros_msg_type = xarm_msgs__srv__FtForceParams_Response;

static bool _FtForceParams_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FtForceParams_Response__ros_msg_type * ros_message = static_cast<const _FtForceParams_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ret
  {
    cdr << ros_message->ret;
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _FtForceParams_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FtForceParams_Response__ros_msg_type * ros_message = static_cast<_FtForceParams_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: ret
  {
    cdr >> ros_message->ret;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__srv__FtForceParams_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FtForceParams_Response__ros_msg_type * ros_message = static_cast<const _FtForceParams_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ret
  {
    size_t item_size = sizeof(ros_message->ret);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _FtForceParams_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__srv__FtForceParams_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__srv__FtForceParams_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: ret
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _FtForceParams_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__srv__FtForceParams_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_FtForceParams_Response = {
  "xarm_msgs::srv",
  "FtForceParams_Response",
  _FtForceParams_Response__cdr_serialize,
  _FtForceParams_Response__cdr_deserialize,
  _FtForceParams_Response__get_serialized_size,
  _FtForceParams_Response__max_serialized_size
};

static rosidl_message_type_support_t _FtForceParams_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FtForceParams_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, FtForceParams_Response)() {
  return &_FtForceParams_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/srv/ft_force_params.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t FtForceParams__callbacks = {
  "xarm_msgs::srv",
  "FtForceParams",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, FtForceParams_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, FtForceParams_Response)(),
};

static rosidl_service_type_support_t FtForceParams__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &FtForceParams__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, FtForceParams)() {
  return &FtForceParams__handle;
}

#if defined(__cplusplus)
}
#endif
