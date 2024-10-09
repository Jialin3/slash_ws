// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teleop_tools_msgs:action/Increment.idl
// generated code does not contain a copyright notice

#ifndef TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__BUILDER_HPP_
#define TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__BUILDER_HPP_

#include "teleop_tools_msgs/action/detail/increment__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_Goal_increment_by
{
public:
  Init_Increment_Goal_increment_by()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::teleop_tools_msgs::action::Increment_Goal increment_by(::teleop_tools_msgs::action::Increment_Goal::_increment_by_type arg)
  {
    msg_.increment_by = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_Goal>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_Goal_increment_by();
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_Result>()
{
  return ::teleop_tools_msgs::action::Increment_Result(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_Feedback>()
{
  return ::teleop_tools_msgs::action::Increment_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_SendGoal_Request_goal
{
public:
  explicit Init_Increment_SendGoal_Request_goal(::teleop_tools_msgs::action::Increment_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::teleop_tools_msgs::action::Increment_SendGoal_Request goal(::teleop_tools_msgs::action::Increment_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_SendGoal_Request msg_;
};

class Init_Increment_SendGoal_Request_goal_id
{
public:
  Init_Increment_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Increment_SendGoal_Request_goal goal_id(::teleop_tools_msgs::action::Increment_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Increment_SendGoal_Request_goal(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_SendGoal_Request>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_SendGoal_Request_goal_id();
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_SendGoal_Response_stamp
{
public:
  explicit Init_Increment_SendGoal_Response_stamp(::teleop_tools_msgs::action::Increment_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::teleop_tools_msgs::action::Increment_SendGoal_Response stamp(::teleop_tools_msgs::action::Increment_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_SendGoal_Response msg_;
};

class Init_Increment_SendGoal_Response_accepted
{
public:
  Init_Increment_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Increment_SendGoal_Response_stamp accepted(::teleop_tools_msgs::action::Increment_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Increment_SendGoal_Response_stamp(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_SendGoal_Response>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_SendGoal_Response_accepted();
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_GetResult_Request_goal_id
{
public:
  Init_Increment_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::teleop_tools_msgs::action::Increment_GetResult_Request goal_id(::teleop_tools_msgs::action::Increment_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_GetResult_Request>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_GetResult_Request_goal_id();
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_GetResult_Response_result
{
public:
  explicit Init_Increment_GetResult_Response_result(::teleop_tools_msgs::action::Increment_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::teleop_tools_msgs::action::Increment_GetResult_Response result(::teleop_tools_msgs::action::Increment_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_GetResult_Response msg_;
};

class Init_Increment_GetResult_Response_status
{
public:
  Init_Increment_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Increment_GetResult_Response_result status(::teleop_tools_msgs::action::Increment_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Increment_GetResult_Response_result(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_GetResult_Response>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_GetResult_Response_status();
}

}  // namespace teleop_tools_msgs


namespace teleop_tools_msgs
{

namespace action
{

namespace builder
{

class Init_Increment_FeedbackMessage_feedback
{
public:
  explicit Init_Increment_FeedbackMessage_feedback(::teleop_tools_msgs::action::Increment_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::teleop_tools_msgs::action::Increment_FeedbackMessage feedback(::teleop_tools_msgs::action::Increment_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_FeedbackMessage msg_;
};

class Init_Increment_FeedbackMessage_goal_id
{
public:
  Init_Increment_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Increment_FeedbackMessage_feedback goal_id(::teleop_tools_msgs::action::Increment_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Increment_FeedbackMessage_feedback(msg_);
  }

private:
  ::teleop_tools_msgs::action::Increment_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::teleop_tools_msgs::action::Increment_FeedbackMessage>()
{
  return teleop_tools_msgs::action::builder::Init_Increment_FeedbackMessage_goal_id();
}

}  // namespace teleop_tools_msgs

#endif  // TELEOP_TOOLS_MSGS__ACTION__DETAIL__INCREMENT__BUILDER_HPP_
