# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosslt_msgs:msg\Int32Tracked.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Int32Tracked(type):
    """Metaclass of message 'Int32Tracked'."""

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
            module = import_type_support('rosslt_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosslt_msgs.msg.Int32Tracked')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__int32_tracked
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__int32_tracked
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__int32_tracked
            cls._TYPE_SUPPORT = module.type_support_msg__msg__int32_tracked
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__int32_tracked

            from rosslt_msgs.msg import LocationHeader
            if LocationHeader.__class__._TYPE_SUPPORT is None:
                LocationHeader.__class__.__import_type_support__()

            from std_msgs.msg import Int32
            if Int32.__class__._TYPE_SUPPORT is None:
                Int32.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Int32Tracked(metaclass=Metaclass_Int32Tracked):
    """Message class 'Int32Tracked'."""

    __slots__ = [
        '_location',
        '_data',
    ]

    _fields_and_field_types = {
        'location': 'rosslt_msgs/LocationHeader',
        'data': 'std_msgs/Int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rosslt_msgs', 'msg'], 'LocationHeader'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rosslt_msgs.msg import LocationHeader
        self.location = kwargs.get('location', LocationHeader())
        from std_msgs.msg import Int32
        self.data = kwargs.get('data', Int32())

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
        if self.location != other.location:
            return False
        if self.data != other.data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def location(self):
        """Message field 'location'."""
        return self._location

    @location.setter
    def location(self, value):
        if __debug__:
            from rosslt_msgs.msg import LocationHeader
            assert \
                isinstance(value, LocationHeader), \
                "The 'location' field must be a sub message of type 'LocationHeader'"
        self._location = value

    @property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if __debug__:
            from std_msgs.msg import Int32
            assert \
                isinstance(value, Int32), \
                "The 'data' field must be a sub message of type 'Int32'"
        self._data = value