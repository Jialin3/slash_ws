# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vesc_msgs:msg/VescState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VescState(type):
    """Metaclass of message 'VescState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FAULT_CODE_NONE': 0,
        'FAULT_CODE_OVER_VOLTAGE': 1,
        'FAULT_CODE_UNDER_VOLTAGE': 2,
        'FAULT_CODE_DRV8302': 3,
        'FAULT_CODE_ABS_OVER_CURRENT': 4,
        'FAULT_CODE_OVER_TEMP_FET': 5,
        'FAULT_CODE_OVER_TEMP_MOTOR': 6,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vesc_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vesc_msgs.msg.VescState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vesc_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vesc_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vesc_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vesc_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vesc_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FAULT_CODE_NONE': cls.__constants['FAULT_CODE_NONE'],
            'FAULT_CODE_OVER_VOLTAGE': cls.__constants['FAULT_CODE_OVER_VOLTAGE'],
            'FAULT_CODE_UNDER_VOLTAGE': cls.__constants['FAULT_CODE_UNDER_VOLTAGE'],
            'FAULT_CODE_DRV8302': cls.__constants['FAULT_CODE_DRV8302'],
            'FAULT_CODE_ABS_OVER_CURRENT': cls.__constants['FAULT_CODE_ABS_OVER_CURRENT'],
            'FAULT_CODE_OVER_TEMP_FET': cls.__constants['FAULT_CODE_OVER_TEMP_FET'],
            'FAULT_CODE_OVER_TEMP_MOTOR': cls.__constants['FAULT_CODE_OVER_TEMP_MOTOR'],
        }

    @property
    def FAULT_CODE_NONE(self):
        """Message constant 'FAULT_CODE_NONE'."""
        return Metaclass_VescState.__constants['FAULT_CODE_NONE']

    @property
    def FAULT_CODE_OVER_VOLTAGE(self):
        """Message constant 'FAULT_CODE_OVER_VOLTAGE'."""
        return Metaclass_VescState.__constants['FAULT_CODE_OVER_VOLTAGE']

    @property
    def FAULT_CODE_UNDER_VOLTAGE(self):
        """Message constant 'FAULT_CODE_UNDER_VOLTAGE'."""
        return Metaclass_VescState.__constants['FAULT_CODE_UNDER_VOLTAGE']

    @property
    def FAULT_CODE_DRV8302(self):
        """Message constant 'FAULT_CODE_DRV8302'."""
        return Metaclass_VescState.__constants['FAULT_CODE_DRV8302']

    @property
    def FAULT_CODE_ABS_OVER_CURRENT(self):
        """Message constant 'FAULT_CODE_ABS_OVER_CURRENT'."""
        return Metaclass_VescState.__constants['FAULT_CODE_ABS_OVER_CURRENT']

    @property
    def FAULT_CODE_OVER_TEMP_FET(self):
        """Message constant 'FAULT_CODE_OVER_TEMP_FET'."""
        return Metaclass_VescState.__constants['FAULT_CODE_OVER_TEMP_FET']

    @property
    def FAULT_CODE_OVER_TEMP_MOTOR(self):
        """Message constant 'FAULT_CODE_OVER_TEMP_MOTOR'."""
        return Metaclass_VescState.__constants['FAULT_CODE_OVER_TEMP_MOTOR']


class VescState(metaclass=Metaclass_VescState):
    """
    Message class 'VescState'.

    Constants:
      FAULT_CODE_NONE
      FAULT_CODE_OVER_VOLTAGE
      FAULT_CODE_UNDER_VOLTAGE
      FAULT_CODE_DRV8302
      FAULT_CODE_ABS_OVER_CURRENT
      FAULT_CODE_OVER_TEMP_FET
      FAULT_CODE_OVER_TEMP_MOTOR
    """

    __slots__ = [
        '_temp_fet',
        '_temp_motor',
        '_current_motor',
        '_current_input',
        '_avg_id',
        '_avg_iq',
        '_duty_cycle',
        '_speed',
        '_voltage_input',
        '_charge_drawn',
        '_charge_regen',
        '_energy_drawn',
        '_energy_regen',
        '_displacement',
        '_distance_traveled',
        '_fault_code',
        '_pid_pos_now',
        '_controller_id',
        '_ntc_temp_mos1',
        '_ntc_temp_mos2',
        '_ntc_temp_mos3',
        '_avg_vd',
        '_avg_vq',
    ]

    _fields_and_field_types = {
        'temp_fet': 'double',
        'temp_motor': 'double',
        'current_motor': 'double',
        'current_input': 'double',
        'avg_id': 'double',
        'avg_iq': 'double',
        'duty_cycle': 'double',
        'speed': 'double',
        'voltage_input': 'double',
        'charge_drawn': 'double',
        'charge_regen': 'double',
        'energy_drawn': 'double',
        'energy_regen': 'double',
        'displacement': 'int32',
        'distance_traveled': 'int32',
        'fault_code': 'int32',
        'pid_pos_now': 'double',
        'controller_id': 'int32',
        'ntc_temp_mos1': 'double',
        'ntc_temp_mos2': 'double',
        'ntc_temp_mos3': 'double',
        'avg_vd': 'double',
        'avg_vq': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.temp_fet = kwargs.get('temp_fet', float())
        self.temp_motor = kwargs.get('temp_motor', float())
        self.current_motor = kwargs.get('current_motor', float())
        self.current_input = kwargs.get('current_input', float())
        self.avg_id = kwargs.get('avg_id', float())
        self.avg_iq = kwargs.get('avg_iq', float())
        self.duty_cycle = kwargs.get('duty_cycle', float())
        self.speed = kwargs.get('speed', float())
        self.voltage_input = kwargs.get('voltage_input', float())
        self.charge_drawn = kwargs.get('charge_drawn', float())
        self.charge_regen = kwargs.get('charge_regen', float())
        self.energy_drawn = kwargs.get('energy_drawn', float())
        self.energy_regen = kwargs.get('energy_regen', float())
        self.displacement = kwargs.get('displacement', int())
        self.distance_traveled = kwargs.get('distance_traveled', int())
        self.fault_code = kwargs.get('fault_code', int())
        self.pid_pos_now = kwargs.get('pid_pos_now', float())
        self.controller_id = kwargs.get('controller_id', int())
        self.ntc_temp_mos1 = kwargs.get('ntc_temp_mos1', float())
        self.ntc_temp_mos2 = kwargs.get('ntc_temp_mos2', float())
        self.ntc_temp_mos3 = kwargs.get('ntc_temp_mos3', float())
        self.avg_vd = kwargs.get('avg_vd', float())
        self.avg_vq = kwargs.get('avg_vq', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.temp_fet != other.temp_fet:
            return False
        if self.temp_motor != other.temp_motor:
            return False
        if self.current_motor != other.current_motor:
            return False
        if self.current_input != other.current_input:
            return False
        if self.avg_id != other.avg_id:
            return False
        if self.avg_iq != other.avg_iq:
            return False
        if self.duty_cycle != other.duty_cycle:
            return False
        if self.speed != other.speed:
            return False
        if self.voltage_input != other.voltage_input:
            return False
        if self.charge_drawn != other.charge_drawn:
            return False
        if self.charge_regen != other.charge_regen:
            return False
        if self.energy_drawn != other.energy_drawn:
            return False
        if self.energy_regen != other.energy_regen:
            return False
        if self.displacement != other.displacement:
            return False
        if self.distance_traveled != other.distance_traveled:
            return False
        if self.fault_code != other.fault_code:
            return False
        if self.pid_pos_now != other.pid_pos_now:
            return False
        if self.controller_id != other.controller_id:
            return False
        if self.ntc_temp_mos1 != other.ntc_temp_mos1:
            return False
        if self.ntc_temp_mos2 != other.ntc_temp_mos2:
            return False
        if self.ntc_temp_mos3 != other.ntc_temp_mos3:
            return False
        if self.avg_vd != other.avg_vd:
            return False
        if self.avg_vq != other.avg_vq:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def temp_fet(self):
        """Message field 'temp_fet'."""
        return self._temp_fet

    @temp_fet.setter
    def temp_fet(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temp_fet' field must be of type 'float'"
        self._temp_fet = value

    @property
    def temp_motor(self):
        """Message field 'temp_motor'."""
        return self._temp_motor

    @temp_motor.setter
    def temp_motor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temp_motor' field must be of type 'float'"
        self._temp_motor = value

    @property
    def current_motor(self):
        """Message field 'current_motor'."""
        return self._current_motor

    @current_motor.setter
    def current_motor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_motor' field must be of type 'float'"
        self._current_motor = value

    @property
    def current_input(self):
        """Message field 'current_input'."""
        return self._current_input

    @current_input.setter
    def current_input(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_input' field must be of type 'float'"
        self._current_input = value

    @property
    def avg_id(self):
        """Message field 'avg_id'."""
        return self._avg_id

    @avg_id.setter
    def avg_id(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'avg_id' field must be of type 'float'"
        self._avg_id = value

    @property
    def avg_iq(self):
        """Message field 'avg_iq'."""
        return self._avg_iq

    @avg_iq.setter
    def avg_iq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'avg_iq' field must be of type 'float'"
        self._avg_iq = value

    @property
    def duty_cycle(self):
        """Message field 'duty_cycle'."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'duty_cycle' field must be of type 'float'"
        self._duty_cycle = value

    @property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
        self._speed = value

    @property
    def voltage_input(self):
        """Message field 'voltage_input'."""
        return self._voltage_input

    @voltage_input.setter
    def voltage_input(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage_input' field must be of type 'float'"
        self._voltage_input = value

    @property
    def charge_drawn(self):
        """Message field 'charge_drawn'."""
        return self._charge_drawn

    @charge_drawn.setter
    def charge_drawn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charge_drawn' field must be of type 'float'"
        self._charge_drawn = value

    @property
    def charge_regen(self):
        """Message field 'charge_regen'."""
        return self._charge_regen

    @charge_regen.setter
    def charge_regen(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'charge_regen' field must be of type 'float'"
        self._charge_regen = value

    @property
    def energy_drawn(self):
        """Message field 'energy_drawn'."""
        return self._energy_drawn

    @energy_drawn.setter
    def energy_drawn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'energy_drawn' field must be of type 'float'"
        self._energy_drawn = value

    @property
    def energy_regen(self):
        """Message field 'energy_regen'."""
        return self._energy_regen

    @energy_regen.setter
    def energy_regen(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'energy_regen' field must be of type 'float'"
        self._energy_regen = value

    @property
    def displacement(self):
        """Message field 'displacement'."""
        return self._displacement

    @displacement.setter
    def displacement(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'displacement' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'displacement' field must be an integer in [-2147483648, 2147483647]"
        self._displacement = value

    @property
    def distance_traveled(self):
        """Message field 'distance_traveled'."""
        return self._distance_traveled

    @distance_traveled.setter
    def distance_traveled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'distance_traveled' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'distance_traveled' field must be an integer in [-2147483648, 2147483647]"
        self._distance_traveled = value

    @property
    def fault_code(self):
        """Message field 'fault_code'."""
        return self._fault_code

    @fault_code.setter
    def fault_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fault_code' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'fault_code' field must be an integer in [-2147483648, 2147483647]"
        self._fault_code = value

    @property
    def pid_pos_now(self):
        """Message field 'pid_pos_now'."""
        return self._pid_pos_now

    @pid_pos_now.setter
    def pid_pos_now(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pid_pos_now' field must be of type 'float'"
        self._pid_pos_now = value

    @property
    def controller_id(self):
        """Message field 'controller_id'."""
        return self._controller_id

    @controller_id.setter
    def controller_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'controller_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'controller_id' field must be an integer in [-2147483648, 2147483647]"
        self._controller_id = value

    @property
    def ntc_temp_mos1(self):
        """Message field 'ntc_temp_mos1'."""
        return self._ntc_temp_mos1

    @ntc_temp_mos1.setter
    def ntc_temp_mos1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ntc_temp_mos1' field must be of type 'float'"
        self._ntc_temp_mos1 = value

    @property
    def ntc_temp_mos2(self):
        """Message field 'ntc_temp_mos2'."""
        return self._ntc_temp_mos2

    @ntc_temp_mos2.setter
    def ntc_temp_mos2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ntc_temp_mos2' field must be of type 'float'"
        self._ntc_temp_mos2 = value

    @property
    def ntc_temp_mos3(self):
        """Message field 'ntc_temp_mos3'."""
        return self._ntc_temp_mos3

    @ntc_temp_mos3.setter
    def ntc_temp_mos3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ntc_temp_mos3' field must be of type 'float'"
        self._ntc_temp_mos3 = value

    @property
    def avg_vd(self):
        """Message field 'avg_vd'."""
        return self._avg_vd

    @avg_vd.setter
    def avg_vd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'avg_vd' field must be of type 'float'"
        self._avg_vd = value

    @property
    def avg_vq(self):
        """Message field 'avg_vq'."""
        return self._avg_vq

    @avg_vq.setter
    def avg_vq(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'avg_vq' field must be of type 'float'"
        self._avg_vq = value
