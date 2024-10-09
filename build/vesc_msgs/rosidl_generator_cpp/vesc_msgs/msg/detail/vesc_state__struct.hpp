// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice

#ifndef VESC_MSGS__MSG__DETAIL__VESC_STATE__STRUCT_HPP_
#define VESC_MSGS__MSG__DETAIL__VESC_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__vesc_msgs__msg__VescState __attribute__((deprecated))
#else
# define DEPRECATED__vesc_msgs__msg__VescState __declspec(deprecated)
#endif

namespace vesc_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VescState_
{
  using Type = VescState_<ContainerAllocator>;

  explicit VescState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temp_fet = 0.0;
      this->temp_motor = 0.0;
      this->current_motor = 0.0;
      this->current_input = 0.0;
      this->avg_id = 0.0;
      this->avg_iq = 0.0;
      this->duty_cycle = 0.0;
      this->speed = 0.0;
      this->voltage_input = 0.0;
      this->charge_drawn = 0.0;
      this->charge_regen = 0.0;
      this->energy_drawn = 0.0;
      this->energy_regen = 0.0;
      this->displacement = 0l;
      this->distance_traveled = 0l;
      this->fault_code = 0l;
      this->pid_pos_now = 0.0;
      this->controller_id = 0l;
      this->ntc_temp_mos1 = 0.0;
      this->ntc_temp_mos2 = 0.0;
      this->ntc_temp_mos3 = 0.0;
      this->avg_vd = 0.0;
      this->avg_vq = 0.0;
    }
  }

  explicit VescState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temp_fet = 0.0;
      this->temp_motor = 0.0;
      this->current_motor = 0.0;
      this->current_input = 0.0;
      this->avg_id = 0.0;
      this->avg_iq = 0.0;
      this->duty_cycle = 0.0;
      this->speed = 0.0;
      this->voltage_input = 0.0;
      this->charge_drawn = 0.0;
      this->charge_regen = 0.0;
      this->energy_drawn = 0.0;
      this->energy_regen = 0.0;
      this->displacement = 0l;
      this->distance_traveled = 0l;
      this->fault_code = 0l;
      this->pid_pos_now = 0.0;
      this->controller_id = 0l;
      this->ntc_temp_mos1 = 0.0;
      this->ntc_temp_mos2 = 0.0;
      this->ntc_temp_mos3 = 0.0;
      this->avg_vd = 0.0;
      this->avg_vq = 0.0;
    }
  }

  // field types and members
  using _temp_fet_type =
    double;
  _temp_fet_type temp_fet;
  using _temp_motor_type =
    double;
  _temp_motor_type temp_motor;
  using _current_motor_type =
    double;
  _current_motor_type current_motor;
  using _current_input_type =
    double;
  _current_input_type current_input;
  using _avg_id_type =
    double;
  _avg_id_type avg_id;
  using _avg_iq_type =
    double;
  _avg_iq_type avg_iq;
  using _duty_cycle_type =
    double;
  _duty_cycle_type duty_cycle;
  using _speed_type =
    double;
  _speed_type speed;
  using _voltage_input_type =
    double;
  _voltage_input_type voltage_input;
  using _charge_drawn_type =
    double;
  _charge_drawn_type charge_drawn;
  using _charge_regen_type =
    double;
  _charge_regen_type charge_regen;
  using _energy_drawn_type =
    double;
  _energy_drawn_type energy_drawn;
  using _energy_regen_type =
    double;
  _energy_regen_type energy_regen;
  using _displacement_type =
    int32_t;
  _displacement_type displacement;
  using _distance_traveled_type =
    int32_t;
  _distance_traveled_type distance_traveled;
  using _fault_code_type =
    int32_t;
  _fault_code_type fault_code;
  using _pid_pos_now_type =
    double;
  _pid_pos_now_type pid_pos_now;
  using _controller_id_type =
    int32_t;
  _controller_id_type controller_id;
  using _ntc_temp_mos1_type =
    double;
  _ntc_temp_mos1_type ntc_temp_mos1;
  using _ntc_temp_mos2_type =
    double;
  _ntc_temp_mos2_type ntc_temp_mos2;
  using _ntc_temp_mos3_type =
    double;
  _ntc_temp_mos3_type ntc_temp_mos3;
  using _avg_vd_type =
    double;
  _avg_vd_type avg_vd;
  using _avg_vq_type =
    double;
  _avg_vq_type avg_vq;

  // setters for named parameter idiom
  Type & set__temp_fet(
    const double & _arg)
  {
    this->temp_fet = _arg;
    return *this;
  }
  Type & set__temp_motor(
    const double & _arg)
  {
    this->temp_motor = _arg;
    return *this;
  }
  Type & set__current_motor(
    const double & _arg)
  {
    this->current_motor = _arg;
    return *this;
  }
  Type & set__current_input(
    const double & _arg)
  {
    this->current_input = _arg;
    return *this;
  }
  Type & set__avg_id(
    const double & _arg)
  {
    this->avg_id = _arg;
    return *this;
  }
  Type & set__avg_iq(
    const double & _arg)
  {
    this->avg_iq = _arg;
    return *this;
  }
  Type & set__duty_cycle(
    const double & _arg)
  {
    this->duty_cycle = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__voltage_input(
    const double & _arg)
  {
    this->voltage_input = _arg;
    return *this;
  }
  Type & set__charge_drawn(
    const double & _arg)
  {
    this->charge_drawn = _arg;
    return *this;
  }
  Type & set__charge_regen(
    const double & _arg)
  {
    this->charge_regen = _arg;
    return *this;
  }
  Type & set__energy_drawn(
    const double & _arg)
  {
    this->energy_drawn = _arg;
    return *this;
  }
  Type & set__energy_regen(
    const double & _arg)
  {
    this->energy_regen = _arg;
    return *this;
  }
  Type & set__displacement(
    const int32_t & _arg)
  {
    this->displacement = _arg;
    return *this;
  }
  Type & set__distance_traveled(
    const int32_t & _arg)
  {
    this->distance_traveled = _arg;
    return *this;
  }
  Type & set__fault_code(
    const int32_t & _arg)
  {
    this->fault_code = _arg;
    return *this;
  }
  Type & set__pid_pos_now(
    const double & _arg)
  {
    this->pid_pos_now = _arg;
    return *this;
  }
  Type & set__controller_id(
    const int32_t & _arg)
  {
    this->controller_id = _arg;
    return *this;
  }
  Type & set__ntc_temp_mos1(
    const double & _arg)
  {
    this->ntc_temp_mos1 = _arg;
    return *this;
  }
  Type & set__ntc_temp_mos2(
    const double & _arg)
  {
    this->ntc_temp_mos2 = _arg;
    return *this;
  }
  Type & set__ntc_temp_mos3(
    const double & _arg)
  {
    this->ntc_temp_mos3 = _arg;
    return *this;
  }
  Type & set__avg_vd(
    const double & _arg)
  {
    this->avg_vd = _arg;
    return *this;
  }
  Type & set__avg_vq(
    const double & _arg)
  {
    this->avg_vq = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t FAULT_CODE_NONE =
    0;
  static constexpr int32_t FAULT_CODE_OVER_VOLTAGE =
    1;
  static constexpr int32_t FAULT_CODE_UNDER_VOLTAGE =
    2;
  static constexpr int32_t FAULT_CODE_DRV8302 =
    3;
  static constexpr int32_t FAULT_CODE_ABS_OVER_CURRENT =
    4;
  static constexpr int32_t FAULT_CODE_OVER_TEMP_FET =
    5;
  static constexpr int32_t FAULT_CODE_OVER_TEMP_MOTOR =
    6;

  // pointer types
  using RawPtr =
    vesc_msgs::msg::VescState_<ContainerAllocator> *;
  using ConstRawPtr =
    const vesc_msgs::msg::VescState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vesc_msgs::msg::VescState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vesc_msgs::msg::VescState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vesc_msgs::msg::VescState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vesc_msgs::msg::VescState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vesc_msgs__msg__VescState
    std::shared_ptr<vesc_msgs::msg::VescState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vesc_msgs__msg__VescState
    std::shared_ptr<vesc_msgs::msg::VescState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VescState_ & other) const
  {
    if (this->temp_fet != other.temp_fet) {
      return false;
    }
    if (this->temp_motor != other.temp_motor) {
      return false;
    }
    if (this->current_motor != other.current_motor) {
      return false;
    }
    if (this->current_input != other.current_input) {
      return false;
    }
    if (this->avg_id != other.avg_id) {
      return false;
    }
    if (this->avg_iq != other.avg_iq) {
      return false;
    }
    if (this->duty_cycle != other.duty_cycle) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->voltage_input != other.voltage_input) {
      return false;
    }
    if (this->charge_drawn != other.charge_drawn) {
      return false;
    }
    if (this->charge_regen != other.charge_regen) {
      return false;
    }
    if (this->energy_drawn != other.energy_drawn) {
      return false;
    }
    if (this->energy_regen != other.energy_regen) {
      return false;
    }
    if (this->displacement != other.displacement) {
      return false;
    }
    if (this->distance_traveled != other.distance_traveled) {
      return false;
    }
    if (this->fault_code != other.fault_code) {
      return false;
    }
    if (this->pid_pos_now != other.pid_pos_now) {
      return false;
    }
    if (this->controller_id != other.controller_id) {
      return false;
    }
    if (this->ntc_temp_mos1 != other.ntc_temp_mos1) {
      return false;
    }
    if (this->ntc_temp_mos2 != other.ntc_temp_mos2) {
      return false;
    }
    if (this->ntc_temp_mos3 != other.ntc_temp_mos3) {
      return false;
    }
    if (this->avg_vd != other.avg_vd) {
      return false;
    }
    if (this->avg_vq != other.avg_vq) {
      return false;
    }
    return true;
  }
  bool operator!=(const VescState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VescState_

// alias to use template instance with default allocator
using VescState =
  vesc_msgs::msg::VescState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_NONE;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_OVER_VOLTAGE;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_UNDER_VOLTAGE;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_DRV8302;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_ABS_OVER_CURRENT;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_OVER_TEMP_FET;
template<typename ContainerAllocator>
constexpr int32_t VescState_<ContainerAllocator>::FAULT_CODE_OVER_TEMP_MOTOR;

}  // namespace msg

}  // namespace vesc_msgs

#endif  // VESC_MSGS__MSG__DETAIL__VESC_STATE__STRUCT_HPP_
