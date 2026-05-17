// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from xarm_msgs:srv/PlanJoint.idl
// generated code does not contain a copyright notice
#include "xarm_msgs/srv/detail/plan_joint__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "xarm_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "xarm_msgs/srv/detail/plan_joint__struct.h"
#include "xarm_msgs/srv/detail/plan_joint__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // target
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // target

// forward declare type support functions


using _PlanJoint_Request__ros_msg_type = xarm_msgs__srv__PlanJoint_Request;

static bool _PlanJoint_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PlanJoint_Request__ros_msg_type * ros_message = static_cast<const _PlanJoint_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target
  {
    size_t size = ros_message->target.size;
    auto array_ptr = ros_message->target.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _PlanJoint_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PlanJoint_Request__ros_msg_type * ros_message = static_cast<_PlanJoint_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: target
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->target.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->target);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->target, size)) {
      return "failed to create array for field 'target'";
    }
    auto array_ptr = ros_message->target.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__srv__PlanJoint_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlanJoint_Request__ros_msg_type * ros_message = static_cast<const _PlanJoint_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name target
  {
    size_t array_size = ros_message->target.size;
    auto array_ptr = ros_message->target.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PlanJoint_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__srv__PlanJoint_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__srv__PlanJoint_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: target
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PlanJoint_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__srv__PlanJoint_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PlanJoint_Request = {
  "xarm_msgs::srv",
  "PlanJoint_Request",
  _PlanJoint_Request__cdr_serialize,
  _PlanJoint_Request__cdr_deserialize,
  _PlanJoint_Request__get_serialized_size,
  _PlanJoint_Request__max_serialized_size
};

static rosidl_message_type_support_t _PlanJoint_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlanJoint_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, PlanJoint_Request)() {
  return &_PlanJoint_Request__type_support;
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
// #include "xarm_msgs/srv/detail/plan_joint__struct.h"
// already included above
// #include "xarm_msgs/srv/detail/plan_joint__functions.h"
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


// forward declare type support functions


using _PlanJoint_Response__ros_msg_type = xarm_msgs__srv__PlanJoint_Response;

static bool _PlanJoint_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PlanJoint_Response__ros_msg_type * ros_message = static_cast<const _PlanJoint_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _PlanJoint_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PlanJoint_Response__ros_msg_type * ros_message = static_cast<_PlanJoint_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t get_serialized_size_xarm_msgs__srv__PlanJoint_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlanJoint_Response__ros_msg_type * ros_message = static_cast<const _PlanJoint_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PlanJoint_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_xarm_msgs__srv__PlanJoint_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_xarm_msgs
size_t max_serialized_size_xarm_msgs__srv__PlanJoint_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _PlanJoint_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_xarm_msgs__srv__PlanJoint_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PlanJoint_Response = {
  "xarm_msgs::srv",
  "PlanJoint_Response",
  _PlanJoint_Response__cdr_serialize,
  _PlanJoint_Response__cdr_deserialize,
  _PlanJoint_Response__get_serialized_size,
  _PlanJoint_Response__max_serialized_size
};

static rosidl_message_type_support_t _PlanJoint_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlanJoint_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, PlanJoint_Response)() {
  return &_PlanJoint_Response__type_support;
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
#include "xarm_msgs/srv/plan_joint.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t PlanJoint__callbacks = {
  "xarm_msgs::srv",
  "PlanJoint",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, PlanJoint_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, PlanJoint_Response)(),
};

static rosidl_service_type_support_t PlanJoint__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &PlanJoint__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xarm_msgs, srv, PlanJoint)() {
  return &PlanJoint__handle;
}

#if defined(__cplusplus)
}
#endif
