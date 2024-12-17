# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_hardware:msg/MotorData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorData(type):
    """Metaclass of message 'MotorData'."""

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
            module = import_type_support('robot_hardware')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_hardware.msg.MotorData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorData(metaclass=Metaclass_MotorData):
    """Message class 'MotorData'."""

    __slots__ = [
        '_real_left_speed_rpm',
        '_real_right_speed_rpm',
        '_real_left_neg_flag',
        '_real_right_neg_flag',
        '_expect_left_speed_rpm',
        '_expect_right_speed_rpm',
        '_expect_left_neg_flag',
        '_expect_right_neg_flag',
    ]

    _fields_and_field_types = {
        'real_left_speed_rpm': 'int16',
        'real_right_speed_rpm': 'int16',
        'real_left_neg_flag': 'uint8',
        'real_right_neg_flag': 'uint8',
        'expect_left_speed_rpm': 'int16',
        'expect_right_speed_rpm': 'int16',
        'expect_left_neg_flag': 'uint8',
        'expect_right_neg_flag': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.real_left_speed_rpm = kwargs.get('real_left_speed_rpm', int())
        self.real_right_speed_rpm = kwargs.get('real_right_speed_rpm', int())
        self.real_left_neg_flag = kwargs.get('real_left_neg_flag', int())
        self.real_right_neg_flag = kwargs.get('real_right_neg_flag', int())
        self.expect_left_speed_rpm = kwargs.get('expect_left_speed_rpm', int())
        self.expect_right_speed_rpm = kwargs.get('expect_right_speed_rpm', int())
        self.expect_left_neg_flag = kwargs.get('expect_left_neg_flag', int())
        self.expect_right_neg_flag = kwargs.get('expect_right_neg_flag', int())

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
        if self.real_left_speed_rpm != other.real_left_speed_rpm:
            return False
        if self.real_right_speed_rpm != other.real_right_speed_rpm:
            return False
        if self.real_left_neg_flag != other.real_left_neg_flag:
            return False
        if self.real_right_neg_flag != other.real_right_neg_flag:
            return False
        if self.expect_left_speed_rpm != other.expect_left_speed_rpm:
            return False
        if self.expect_right_speed_rpm != other.expect_right_speed_rpm:
            return False
        if self.expect_left_neg_flag != other.expect_left_neg_flag:
            return False
        if self.expect_right_neg_flag != other.expect_right_neg_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def real_left_speed_rpm(self):
        """Message field 'real_left_speed_rpm'."""
        return self._real_left_speed_rpm

    @real_left_speed_rpm.setter
    def real_left_speed_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'real_left_speed_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'real_left_speed_rpm' field must be an integer in [-32768, 32767]"
        self._real_left_speed_rpm = value

    @builtins.property
    def real_right_speed_rpm(self):
        """Message field 'real_right_speed_rpm'."""
        return self._real_right_speed_rpm

    @real_right_speed_rpm.setter
    def real_right_speed_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'real_right_speed_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'real_right_speed_rpm' field must be an integer in [-32768, 32767]"
        self._real_right_speed_rpm = value

    @builtins.property
    def real_left_neg_flag(self):
        """Message field 'real_left_neg_flag'."""
        return self._real_left_neg_flag

    @real_left_neg_flag.setter
    def real_left_neg_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'real_left_neg_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'real_left_neg_flag' field must be an unsigned integer in [0, 255]"
        self._real_left_neg_flag = value

    @builtins.property
    def real_right_neg_flag(self):
        """Message field 'real_right_neg_flag'."""
        return self._real_right_neg_flag

    @real_right_neg_flag.setter
    def real_right_neg_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'real_right_neg_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'real_right_neg_flag' field must be an unsigned integer in [0, 255]"
        self._real_right_neg_flag = value

    @builtins.property
    def expect_left_speed_rpm(self):
        """Message field 'expect_left_speed_rpm'."""
        return self._expect_left_speed_rpm

    @expect_left_speed_rpm.setter
    def expect_left_speed_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'expect_left_speed_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'expect_left_speed_rpm' field must be an integer in [-32768, 32767]"
        self._expect_left_speed_rpm = value

    @builtins.property
    def expect_right_speed_rpm(self):
        """Message field 'expect_right_speed_rpm'."""
        return self._expect_right_speed_rpm

    @expect_right_speed_rpm.setter
    def expect_right_speed_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'expect_right_speed_rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'expect_right_speed_rpm' field must be an integer in [-32768, 32767]"
        self._expect_right_speed_rpm = value

    @builtins.property
    def expect_left_neg_flag(self):
        """Message field 'expect_left_neg_flag'."""
        return self._expect_left_neg_flag

    @expect_left_neg_flag.setter
    def expect_left_neg_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'expect_left_neg_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'expect_left_neg_flag' field must be an unsigned integer in [0, 255]"
        self._expect_left_neg_flag = value

    @builtins.property
    def expect_right_neg_flag(self):
        """Message field 'expect_right_neg_flag'."""
        return self._expect_right_neg_flag

    @expect_right_neg_flag.setter
    def expect_right_neg_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'expect_right_neg_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'expect_right_neg_flag' field must be an unsigned integer in [0, 255]"
        self._expect_right_neg_flag = value
