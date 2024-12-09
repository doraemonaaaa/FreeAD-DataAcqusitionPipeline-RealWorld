# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_hardware:msg/HardwareData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HardwareData(type):
    """Metaclass of message 'HardwareData'."""

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
                'robot_hardware.msg.HardwareData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hardware_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hardware_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hardware_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hardware_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hardware_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HardwareData(metaclass=Metaclass_HardwareData):
    """Message class 'HardwareData'."""

    __slots__ = [
        '_real_left_speed_rpm',
        '_real_right_speed_rpm',
        '_real_left_neg_flag',
        '_real_right_neg_flag',
        '_expect_left_speed_rpm',
        '_expect_right_speed_rpm',
        '_expect_left_neg_flag',
        '_expect_right_neg_flag',
        '_imu_orientation_x',
        '_imu_orientation_y',
        '_imu_orientation_z',
        '_imu_orientation_w',
        '_imu_linear_acceleration_x',
        '_imu_linear_acceleration_y',
        '_imu_linear_acceleration_z',
        '_imu_angular_velocity_x',
        '_imu_angular_velocity_y',
        '_imu_angular_velocity_z',
        '_magnetic_field_x',
        '_magnetic_field_y',
        '_magnetic_field_z',
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
        'imu_orientation_x': 'double',
        'imu_orientation_y': 'double',
        'imu_orientation_z': 'double',
        'imu_orientation_w': 'double',
        'imu_linear_acceleration_x': 'double',
        'imu_linear_acceleration_y': 'double',
        'imu_linear_acceleration_z': 'double',
        'imu_angular_velocity_x': 'double',
        'imu_angular_velocity_y': 'double',
        'imu_angular_velocity_z': 'double',
        'magnetic_field_x': 'double',
        'magnetic_field_y': 'double',
        'magnetic_field_z': 'double',
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
        self.imu_orientation_x = kwargs.get('imu_orientation_x', float())
        self.imu_orientation_y = kwargs.get('imu_orientation_y', float())
        self.imu_orientation_z = kwargs.get('imu_orientation_z', float())
        self.imu_orientation_w = kwargs.get('imu_orientation_w', float())
        self.imu_linear_acceleration_x = kwargs.get('imu_linear_acceleration_x', float())
        self.imu_linear_acceleration_y = kwargs.get('imu_linear_acceleration_y', float())
        self.imu_linear_acceleration_z = kwargs.get('imu_linear_acceleration_z', float())
        self.imu_angular_velocity_x = kwargs.get('imu_angular_velocity_x', float())
        self.imu_angular_velocity_y = kwargs.get('imu_angular_velocity_y', float())
        self.imu_angular_velocity_z = kwargs.get('imu_angular_velocity_z', float())
        self.magnetic_field_x = kwargs.get('magnetic_field_x', float())
        self.magnetic_field_y = kwargs.get('magnetic_field_y', float())
        self.magnetic_field_z = kwargs.get('magnetic_field_z', float())

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
        if self.imu_orientation_x != other.imu_orientation_x:
            return False
        if self.imu_orientation_y != other.imu_orientation_y:
            return False
        if self.imu_orientation_z != other.imu_orientation_z:
            return False
        if self.imu_orientation_w != other.imu_orientation_w:
            return False
        if self.imu_linear_acceleration_x != other.imu_linear_acceleration_x:
            return False
        if self.imu_linear_acceleration_y != other.imu_linear_acceleration_y:
            return False
        if self.imu_linear_acceleration_z != other.imu_linear_acceleration_z:
            return False
        if self.imu_angular_velocity_x != other.imu_angular_velocity_x:
            return False
        if self.imu_angular_velocity_y != other.imu_angular_velocity_y:
            return False
        if self.imu_angular_velocity_z != other.imu_angular_velocity_z:
            return False
        if self.magnetic_field_x != other.magnetic_field_x:
            return False
        if self.magnetic_field_y != other.magnetic_field_y:
            return False
        if self.magnetic_field_z != other.magnetic_field_z:
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

    @builtins.property
    def imu_orientation_x(self):
        """Message field 'imu_orientation_x'."""
        return self._imu_orientation_x

    @imu_orientation_x.setter
    def imu_orientation_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_orientation_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_orientation_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_orientation_x = value

    @builtins.property
    def imu_orientation_y(self):
        """Message field 'imu_orientation_y'."""
        return self._imu_orientation_y

    @imu_orientation_y.setter
    def imu_orientation_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_orientation_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_orientation_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_orientation_y = value

    @builtins.property
    def imu_orientation_z(self):
        """Message field 'imu_orientation_z'."""
        return self._imu_orientation_z

    @imu_orientation_z.setter
    def imu_orientation_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_orientation_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_orientation_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_orientation_z = value

    @builtins.property
    def imu_orientation_w(self):
        """Message field 'imu_orientation_w'."""
        return self._imu_orientation_w

    @imu_orientation_w.setter
    def imu_orientation_w(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_orientation_w' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_orientation_w' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_orientation_w = value

    @builtins.property
    def imu_linear_acceleration_x(self):
        """Message field 'imu_linear_acceleration_x'."""
        return self._imu_linear_acceleration_x

    @imu_linear_acceleration_x.setter
    def imu_linear_acceleration_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_linear_acceleration_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_linear_acceleration_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_linear_acceleration_x = value

    @builtins.property
    def imu_linear_acceleration_y(self):
        """Message field 'imu_linear_acceleration_y'."""
        return self._imu_linear_acceleration_y

    @imu_linear_acceleration_y.setter
    def imu_linear_acceleration_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_linear_acceleration_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_linear_acceleration_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_linear_acceleration_y = value

    @builtins.property
    def imu_linear_acceleration_z(self):
        """Message field 'imu_linear_acceleration_z'."""
        return self._imu_linear_acceleration_z

    @imu_linear_acceleration_z.setter
    def imu_linear_acceleration_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_linear_acceleration_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_linear_acceleration_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_linear_acceleration_z = value

    @builtins.property
    def imu_angular_velocity_x(self):
        """Message field 'imu_angular_velocity_x'."""
        return self._imu_angular_velocity_x

    @imu_angular_velocity_x.setter
    def imu_angular_velocity_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_angular_velocity_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_angular_velocity_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_angular_velocity_x = value

    @builtins.property
    def imu_angular_velocity_y(self):
        """Message field 'imu_angular_velocity_y'."""
        return self._imu_angular_velocity_y

    @imu_angular_velocity_y.setter
    def imu_angular_velocity_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_angular_velocity_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_angular_velocity_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_angular_velocity_y = value

    @builtins.property
    def imu_angular_velocity_z(self):
        """Message field 'imu_angular_velocity_z'."""
        return self._imu_angular_velocity_z

    @imu_angular_velocity_z.setter
    def imu_angular_velocity_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_angular_velocity_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'imu_angular_velocity_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._imu_angular_velocity_z = value

    @builtins.property
    def magnetic_field_x(self):
        """Message field 'magnetic_field_x'."""
        return self._magnetic_field_x

    @magnetic_field_x.setter
    def magnetic_field_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'magnetic_field_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'magnetic_field_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._magnetic_field_x = value

    @builtins.property
    def magnetic_field_y(self):
        """Message field 'magnetic_field_y'."""
        return self._magnetic_field_y

    @magnetic_field_y.setter
    def magnetic_field_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'magnetic_field_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'magnetic_field_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._magnetic_field_y = value

    @builtins.property
    def magnetic_field_z(self):
        """Message field 'magnetic_field_z'."""
        return self._magnetic_field_z

    @magnetic_field_z.setter
    def magnetic_field_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'magnetic_field_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'magnetic_field_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._magnetic_field_z = value
