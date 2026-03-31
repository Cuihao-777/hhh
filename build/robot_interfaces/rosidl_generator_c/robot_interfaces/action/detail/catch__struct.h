// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:action/Catch.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__ACTION__DETAIL__CATCH__STRUCT_H_
#define ROBOT_INTERFACES__ACTION__DETAIL__CATCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_Goal
{
  geometry_msgs__msg__Pose target_pose;
  int32_t action_type;
} robot_interfaces__action__Catch_Goal;

// Struct for a sequence of robot_interfaces__action__Catch_Goal.
typedef struct robot_interfaces__action__Catch_Goal__Sequence
{
  robot_interfaces__action__Catch_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_Result
{
  bool success;
  rosidl_runtime_c__String reason;
} robot_interfaces__action__Catch_Result;

// Struct for a sequence of robot_interfaces__action__Catch_Result.
typedef struct robot_interfaces__action__Catch_Result__Sequence
{
  robot_interfaces__action__Catch_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_state'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_Feedback
{
  rosidl_runtime_c__String current_state;
  float progress_percentage;
} robot_interfaces__action__Catch_Feedback;

// Struct for a sequence of robot_interfaces__action__Catch_Feedback.
typedef struct robot_interfaces__action__Catch_Feedback__Sequence
{
  robot_interfaces__action__Catch_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robot_interfaces/action/detail/catch__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_interfaces__action__Catch_Goal goal;
} robot_interfaces__action__Catch_SendGoal_Request;

// Struct for a sequence of robot_interfaces__action__Catch_SendGoal_Request.
typedef struct robot_interfaces__action__Catch_SendGoal_Request__Sequence
{
  robot_interfaces__action__Catch_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robot_interfaces__action__Catch_SendGoal_Response;

// Struct for a sequence of robot_interfaces__action__Catch_SendGoal_Response.
typedef struct robot_interfaces__action__Catch_SendGoal_Response__Sequence
{
  robot_interfaces__action__Catch_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robot_interfaces__action__Catch_GetResult_Request;

// Struct for a sequence of robot_interfaces__action__Catch_GetResult_Request.
typedef struct robot_interfaces__action__Catch_GetResult_Request__Sequence
{
  robot_interfaces__action__Catch_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_interfaces/action/detail/catch__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_GetResult_Response
{
  int8_t status;
  robot_interfaces__action__Catch_Result result;
} robot_interfaces__action__Catch_GetResult_Response;

// Struct for a sequence of robot_interfaces__action__Catch_GetResult_Response.
typedef struct robot_interfaces__action__Catch_GetResult_Response__Sequence
{
  robot_interfaces__action__Catch_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robot_interfaces/action/detail/catch__struct.h"

/// Struct defined in action/Catch in the package robot_interfaces.
typedef struct robot_interfaces__action__Catch_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_interfaces__action__Catch_Feedback feedback;
} robot_interfaces__action__Catch_FeedbackMessage;

// Struct for a sequence of robot_interfaces__action__Catch_FeedbackMessage.
typedef struct robot_interfaces__action__Catch_FeedbackMessage__Sequence
{
  robot_interfaces__action__Catch_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Catch_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__ACTION__DETAIL__CATCH__STRUCT_H_
