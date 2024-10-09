// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vesc_msgs:msg/VescState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "vesc_msgs/msg/detail/vesc_state__struct.h"
#include "vesc_msgs/msg/detail/vesc_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vesc_msgs__msg__vesc_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("vesc_msgs.msg._vesc_state.VescState", full_classname_dest, 35) == 0);
  }
  vesc_msgs__msg__VescState * ros_message = _ros_message;
  {  // temp_fet
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_fet");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temp_fet = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temp_motor
    PyObject * field = PyObject_GetAttrString(_pymsg, "temp_motor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temp_motor = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_motor
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_motor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_motor = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_input");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_input = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // avg_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "avg_id");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->avg_id = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // avg_iq
    PyObject * field = PyObject_GetAttrString(_pymsg, "avg_iq");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->avg_iq = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // duty_cycle
    PyObject * field = PyObject_GetAttrString(_pymsg, "duty_cycle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duty_cycle = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // voltage_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "voltage_input");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->voltage_input = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // charge_drawn
    PyObject * field = PyObject_GetAttrString(_pymsg, "charge_drawn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->charge_drawn = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // charge_regen
    PyObject * field = PyObject_GetAttrString(_pymsg, "charge_regen");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->charge_regen = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // energy_drawn
    PyObject * field = PyObject_GetAttrString(_pymsg, "energy_drawn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->energy_drawn = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // energy_regen
    PyObject * field = PyObject_GetAttrString(_pymsg, "energy_regen");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->energy_regen = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // displacement
    PyObject * field = PyObject_GetAttrString(_pymsg, "displacement");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->displacement = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // distance_traveled
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_traveled");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->distance_traveled = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // fault_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fault_code = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // pid_pos_now
    PyObject * field = PyObject_GetAttrString(_pymsg, "pid_pos_now");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pid_pos_now = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // controller_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "controller_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->controller_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // ntc_temp_mos1
    PyObject * field = PyObject_GetAttrString(_pymsg, "ntc_temp_mos1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ntc_temp_mos1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ntc_temp_mos2
    PyObject * field = PyObject_GetAttrString(_pymsg, "ntc_temp_mos2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ntc_temp_mos2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ntc_temp_mos3
    PyObject * field = PyObject_GetAttrString(_pymsg, "ntc_temp_mos3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ntc_temp_mos3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // avg_vd
    PyObject * field = PyObject_GetAttrString(_pymsg, "avg_vd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->avg_vd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // avg_vq
    PyObject * field = PyObject_GetAttrString(_pymsg, "avg_vq");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->avg_vq = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vesc_msgs__msg__vesc_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VescState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vesc_msgs.msg._vesc_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VescState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vesc_msgs__msg__VescState * ros_message = (vesc_msgs__msg__VescState *)raw_ros_message;
  {  // temp_fet
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temp_fet);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temp_fet", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temp_motor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temp_motor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temp_motor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_motor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_motor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_motor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_input
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // avg_id
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->avg_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "avg_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // avg_iq
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->avg_iq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "avg_iq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duty_cycle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duty_cycle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duty_cycle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // voltage_input
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->voltage_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "voltage_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charge_drawn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->charge_drawn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charge_drawn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charge_regen
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->charge_regen);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charge_regen", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // energy_drawn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->energy_drawn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "energy_drawn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // energy_regen
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->energy_regen);
    {
      int rc = PyObject_SetAttrString(_pymessage, "energy_regen", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // displacement
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->displacement);
    {
      int rc = PyObject_SetAttrString(_pymessage, "displacement", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_traveled
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->distance_traveled);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_traveled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_code
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->fault_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pid_pos_now
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pid_pos_now);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pid_pos_now", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // controller_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->controller_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "controller_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ntc_temp_mos1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ntc_temp_mos1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ntc_temp_mos1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ntc_temp_mos2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ntc_temp_mos2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ntc_temp_mos2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ntc_temp_mos3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ntc_temp_mos3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ntc_temp_mos3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // avg_vd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->avg_vd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "avg_vd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // avg_vq
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->avg_vq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "avg_vq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
