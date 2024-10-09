// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_

#include "vesc_msgs/msg/detail/vesc_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vesc_msgs
{

namespace msg
{

namespace builder
{

class Init_VescState_avg_vq
{
public:
  explicit Init_VescState_avg_vq(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  ::vesc_msgs::msg::VescState avg_vq(::vesc_msgs::msg::VescState::_avg_vq_type arg)
  {
    msg_.avg_vq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_avg_vd
{
public:
  explicit Init_VescState_avg_vd(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_avg_vq avg_vd(::vesc_msgs::msg::VescState::_avg_vd_type arg)
  {
    msg_.avg_vd = std::move(arg);
    return Init_VescState_avg_vq(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_ntc_temp_mos3
{
public:
  explicit Init_VescState_ntc_temp_mos3(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_avg_vd ntc_temp_mos3(::vesc_msgs::msg::VescState::_ntc_temp_mos3_type arg)
  {
    msg_.ntc_temp_mos3 = std::move(arg);
    return Init_VescState_avg_vd(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_ntc_temp_mos2
{
public:
  explicit Init_VescState_ntc_temp_mos2(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_ntc_temp_mos3 ntc_temp_mos2(::vesc_msgs::msg::VescState::_ntc_temp_mos2_type arg)
  {
    msg_.ntc_temp_mos2 = std::move(arg);
    return Init_VescState_ntc_temp_mos3(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_ntc_temp_mos1
{
public:
  explicit Init_VescState_ntc_temp_mos1(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_ntc_temp_mos2 ntc_temp_mos1(::vesc_msgs::msg::VescState::_ntc_temp_mos1_type arg)
  {
    msg_.ntc_temp_mos1 = std::move(arg);
    return Init_VescState_ntc_temp_mos2(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_controller_id
{
public:
  explicit Init_VescState_controller_id(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_ntc_temp_mos1 controller_id(::vesc_msgs::msg::VescState::_controller_id_type arg)
  {
    msg_.controller_id = std::move(arg);
    return Init_VescState_ntc_temp_mos1(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_pid_pos_now
{
public:
  explicit Init_VescState_pid_pos_now(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_controller_id pid_pos_now(::vesc_msgs::msg::VescState::_pid_pos_now_type arg)
  {
    msg_.pid_pos_now = std::move(arg);
    return Init_VescState_controller_id(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_fault_code
{
public:
  explicit Init_VescState_fault_code(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_pid_pos_now fault_code(::vesc_msgs::msg::VescState::_fault_code_type arg)
  {
    msg_.fault_code = std::move(arg);
    return Init_VescState_pid_pos_now(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_distance_traveled
{
public:
  explicit Init_VescState_distance_traveled(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_fault_code distance_traveled(::vesc_msgs::msg::VescState::_distance_traveled_type arg)
  {
    msg_.distance_traveled = std::move(arg);
    return Init_VescState_fault_code(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_displacement
{
public:
  explicit Init_VescState_displacement(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_distance_traveled displacement(::vesc_msgs::msg::VescState::_displacement_type arg)
  {
    msg_.displacement = std::move(arg);
    return Init_VescState_distance_traveled(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_energy_regen
{
public:
  explicit Init_VescState_energy_regen(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_displacement energy_regen(::vesc_msgs::msg::VescState::_energy_regen_type arg)
  {
    msg_.energy_regen = std::move(arg);
    return Init_VescState_displacement(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_energy_drawn
{
public:
  explicit Init_VescState_energy_drawn(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_energy_regen energy_drawn(::vesc_msgs::msg::VescState::_energy_drawn_type arg)
  {
    msg_.energy_drawn = std::move(arg);
    return Init_VescState_energy_regen(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_charge_regen
{
public:
  explicit Init_VescState_charge_regen(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_energy_drawn charge_regen(::vesc_msgs::msg::VescState::_charge_regen_type arg)
  {
    msg_.charge_regen = std::move(arg);
    return Init_VescState_energy_drawn(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_charge_drawn
{
public:
  explicit Init_VescState_charge_drawn(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_charge_regen charge_drawn(::vesc_msgs::msg::VescState::_charge_drawn_type arg)
  {
    msg_.charge_drawn = std::move(arg);
    return Init_VescState_charge_regen(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_voltage_input
{
public:
  explicit Init_VescState_voltage_input(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_charge_drawn voltage_input(::vesc_msgs::msg::VescState::_voltage_input_type arg)
  {
    msg_.voltage_input = std::move(arg);
    return Init_VescState_charge_drawn(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_speed
{
public:
  explicit Init_VescState_speed(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_voltage_input speed(::vesc_msgs::msg::VescState::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_VescState_voltage_input(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_duty_cycle
{
public:
  explicit Init_VescState_duty_cycle(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_speed duty_cycle(::vesc_msgs::msg::VescState::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return Init_VescState_speed(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_avg_iq
{
public:
  explicit Init_VescState_avg_iq(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_duty_cycle avg_iq(::vesc_msgs::msg::VescState::_avg_iq_type arg)
  {
    msg_.avg_iq = std::move(arg);
    return Init_VescState_duty_cycle(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_avg_id
{
public:
  explicit Init_VescState_avg_id(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_avg_iq avg_id(::vesc_msgs::msg::VescState::_avg_id_type arg)
  {
    msg_.avg_id = std::move(arg);
    return Init_VescState_avg_iq(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_current_input
{
public:
  explicit Init_VescState_current_input(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_avg_id current_input(::vesc_msgs::msg::VescState::_current_input_type arg)
  {
    msg_.current_input = std::move(arg);
    return Init_VescState_avg_id(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_current_motor
{
public:
  explicit Init_VescState_current_motor(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_current_input current_motor(::vesc_msgs::msg::VescState::_current_motor_type arg)
  {
    msg_.current_motor = std::move(arg);
    return Init_VescState_current_input(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_temp_motor
{
public:
  explicit Init_VescState_temp_motor(::vesc_msgs::msg::VescState & msg)
  : msg_(msg)
  {}
  Init_VescState_current_motor temp_motor(::vesc_msgs::msg::VescState::_temp_motor_type arg)
  {
    msg_.temp_motor = std::move(arg);
    return Init_VescState_current_motor(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

class Init_VescState_temp_fet
{
public:
  Init_VescState_temp_fet()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescState_temp_motor temp_fet(::vesc_msgs::msg::VescState::_temp_fet_type arg)
  {
    msg_.temp_fet = std::move(arg);
    return Init_VescState_temp_motor(msg_);
  }

private:
  ::vesc_msgs::msg::VescState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_msgs::msg::VescState>()
{
  return vesc_msgs::msg::builder::Init_VescState_temp_fet();
}

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__BUILDER_HPP_
