# generated from rosidl_generator_py/resource/_idl.py.em
# with input from fast_lio:msg/Pose6D.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'acc'
# Member 'gyr'
# Member 'vel'
# Member 'pos'
# Member 'rot'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Pose6D(type):
    """Metaclass of message 'Pose6D'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('fast_lio')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'fast_lio.msg.Pose6D')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pose6_d
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pose6_d
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pose6_d
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pose6_d
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pose6_d

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Pose6D(metaclass=Metaclass_Pose6D):
    """Message class 'Pose6D'."""

    __slots__ = [
        '_offset_time',
        '_acc',
        '_gyr',
        '_vel',
        '_pos',
        '_rot',
    ]

    _fields_and_field_types = {
        'offset_time': 'double',
        'acc': 'double[3]',
        'gyr': 'double[3]',
        'vel': 'double[3]',
        'pos': 'double[3]',
        'rot': 'double[9]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.offset_time = kwargs.get('offset_time', float())
        if 'acc' not in kwargs:
            self.acc = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.acc = numpy.array(kwargs.get('acc'), dtype=numpy.float64)
            assert self.acc.shape == (3, )
        if 'gyr' not in kwargs:
            self.gyr = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.gyr = numpy.array(kwargs.get('gyr'), dtype=numpy.float64)
            assert self.gyr.shape == (3, )
        if 'vel' not in kwargs:
            self.vel = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.vel = numpy.array(kwargs.get('vel'), dtype=numpy.float64)
            assert self.vel.shape == (3, )
        if 'pos' not in kwargs:
            self.pos = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.pos = numpy.array(kwargs.get('pos'), dtype=numpy.float64)
            assert self.pos.shape == (3, )
        if 'rot' not in kwargs:
            self.rot = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.rot = numpy.array(kwargs.get('rot'), dtype=numpy.float64)
            assert self.rot.shape == (9, )

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
        if self.offset_time != other.offset_time:
            return False
        if all(self.acc != other.acc):
            return False
        if all(self.gyr != other.gyr):
            return False
        if all(self.vel != other.vel):
            return False
        if all(self.pos != other.pos):
            return False
        if all(self.rot != other.rot):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def offset_time(self):
        """Message field 'offset_time'."""
        return self._offset_time

    @offset_time.setter
    def offset_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offset_time' field must be of type 'float'"
        self._offset_time = value

    @property
    def acc(self):
        """Message field 'acc'."""
        return self._acc

    @acc.setter
    def acc(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'acc' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'acc' numpy.ndarray() must have a size of 3"
            self._acc = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'acc' field must be a set or sequence with length 3 and each value of type 'float'"
        self._acc = numpy.array(value, dtype=numpy.float64)

    @property
    def gyr(self):
        """Message field 'gyr'."""
        return self._gyr

    @gyr.setter
    def gyr(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'gyr' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'gyr' numpy.ndarray() must have a size of 3"
            self._gyr = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'gyr' field must be a set or sequence with length 3 and each value of type 'float'"
        self._gyr = numpy.array(value, dtype=numpy.float64)

    @property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'vel' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'vel' numpy.ndarray() must have a size of 3"
            self._vel = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'vel' field must be a set or sequence with length 3 and each value of type 'float'"
        self._vel = numpy.array(value, dtype=numpy.float64)

    @property
    def pos(self):
        """Message field 'pos'."""
        return self._pos

    @pos.setter
    def pos(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'pos' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'pos' numpy.ndarray() must have a size of 3"
            self._pos = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'pos' field must be a set or sequence with length 3 and each value of type 'float'"
        self._pos = numpy.array(value, dtype=numpy.float64)

    @property
    def rot(self):
        """Message field 'rot'."""
        return self._rot

    @rot.setter
    def rot(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'rot' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'rot' numpy.ndarray() must have a size of 9"
            self._rot = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'rot' field must be a set or sequence with length 9 and each value of type 'float'"
        self._rot = numpy.array(value, dtype=numpy.float64)
