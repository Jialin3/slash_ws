// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from teleop_tools_msgs:action/Increment.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "teleop_tools_msgs/action/detail/increment__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_Goal_type_support_ids_t;

static const _Increment_Goal_type_support_ids_t _Increment_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_Goal_type_support_symbol_names_t _Increment_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_Goal)),
  }
};

typedef struct _Increment_Goal_type_support_data_t
{
  void * data[2];
} _Increment_Goal_type_support_data_t;

static _Increment_Goal_type_support_data_t _Increment_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_Goal_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_Goal>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_Goal)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_Result_type_support_ids_t;

static const _Increment_Result_type_support_ids_t _Increment_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_Result_type_support_symbol_names_t _Increment_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_Result)),
  }
};

typedef struct _Increment_Result_type_support_data_t
{
  void * data[2];
} _Increment_Result_type_support_data_t;

static _Increment_Result_type_support_data_t _Increment_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_Result_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_Result>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_Result)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_Feedback_type_support_ids_t;

static const _Increment_Feedback_type_support_ids_t _Increment_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_Feedback_type_support_symbol_names_t _Increment_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_Feedback)),
  }
};

typedef struct _Increment_Feedback_type_support_data_t
{
  void * data[2];
} _Increment_Feedback_type_support_data_t;

static _Increment_Feedback_type_support_data_t _Increment_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_Feedback_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_Feedback>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_Feedback)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_SendGoal_Request_type_support_ids_t;

static const _Increment_SendGoal_Request_type_support_ids_t _Increment_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_SendGoal_Request_type_support_symbol_names_t _Increment_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_SendGoal_Request)),
  }
};

typedef struct _Increment_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Increment_SendGoal_Request_type_support_data_t;

static _Increment_SendGoal_Request_type_support_data_t _Increment_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_SendGoal_Request_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal_Request>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_SendGoal_Request)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_SendGoal_Response_type_support_ids_t;

static const _Increment_SendGoal_Response_type_support_ids_t _Increment_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_SendGoal_Response_type_support_symbol_names_t _Increment_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_SendGoal_Response)),
  }
};

typedef struct _Increment_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Increment_SendGoal_Response_type_support_data_t;

static _Increment_SendGoal_Response_type_support_data_t _Increment_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_SendGoal_Response_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal_Response>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_SendGoal_Response)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_SendGoal_type_support_ids_t;

static const _Increment_SendGoal_type_support_ids_t _Increment_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_SendGoal_type_support_symbol_names_t _Increment_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_SendGoal)),
  }
};

typedef struct _Increment_SendGoal_type_support_data_t
{
  void * data[2];
} _Increment_SendGoal_type_support_data_t;

static _Increment_SendGoal_type_support_data_t _Increment_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_SendGoal_service_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Increment_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Increment_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Increment_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<teleop_tools_msgs::action::Increment_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_GetResult_Request_type_support_ids_t;

static const _Increment_GetResult_Request_type_support_ids_t _Increment_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_GetResult_Request_type_support_symbol_names_t _Increment_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_GetResult_Request)),
  }
};

typedef struct _Increment_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Increment_GetResult_Request_type_support_data_t;

static _Increment_GetResult_Request_type_support_data_t _Increment_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_GetResult_Request_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_GetResult_Request>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_GetResult_Request)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_GetResult_Response_type_support_ids_t;

static const _Increment_GetResult_Response_type_support_ids_t _Increment_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_GetResult_Response_type_support_symbol_names_t _Increment_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_GetResult_Response)),
  }
};

typedef struct _Increment_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Increment_GetResult_Response_type_support_data_t;

static _Increment_GetResult_Response_type_support_data_t _Increment_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_GetResult_Response_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_GetResult_Response>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_GetResult_Response)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_GetResult_type_support_ids_t;

static const _Increment_GetResult_type_support_ids_t _Increment_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_GetResult_type_support_symbol_names_t _Increment_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_GetResult)),
  }
};

typedef struct _Increment_GetResult_type_support_data_t
{
  void * data[2];
} _Increment_GetResult_type_support_data_t;

static _Increment_GetResult_type_support_data_t _Increment_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_GetResult_service_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Increment_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Increment_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Increment_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<teleop_tools_msgs::action::Increment_GetResult>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<teleop_tools_msgs::action::Increment_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Increment_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Increment_FeedbackMessage_type_support_ids_t;

static const _Increment_FeedbackMessage_type_support_ids_t _Increment_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Increment_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Increment_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Increment_FeedbackMessage_type_support_symbol_names_t _Increment_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teleop_tools_msgs, action, Increment_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teleop_tools_msgs, action, Increment_FeedbackMessage)),
  }
};

typedef struct _Increment_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Increment_FeedbackMessage_type_support_data_t;

static _Increment_FeedbackMessage_type_support_data_t _Increment_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Increment_FeedbackMessage_message_typesupport_map = {
  2,
  "teleop_tools_msgs",
  &_Increment_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Increment_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Increment_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Increment_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Increment_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teleop_tools_msgs::action::Increment_FeedbackMessage>()
{
  return &::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment_FeedbackMessage)() {
  return get_message_type_support_handle<teleop_tools_msgs::action::Increment_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "teleop_tools_msgs/action/detail/increment__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace teleop_tools_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t Increment_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace teleop_tools_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<teleop_tools_msgs::action::Increment>()
{
  using ::teleop_tools_msgs::action::rosidl_typesupport_cpp::Increment_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  Increment_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::teleop_tools_msgs::action::Increment::Impl::SendGoalService>();
  Increment_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::teleop_tools_msgs::action::Increment::Impl::GetResultService>();
  Increment_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::teleop_tools_msgs::action::Increment::Impl::CancelGoalService>();
  Increment_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::teleop_tools_msgs::action::Increment::Impl::FeedbackMessage>();
  Increment_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::teleop_tools_msgs::action::Increment::Impl::GoalStatusMessage>();
  return &Increment_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, teleop_tools_msgs, action, Increment)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<teleop_tools_msgs::action::Increment>();
}

#ifdef __cplusplus
}
#endif
