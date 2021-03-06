# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosslt_msgs:msg\Location.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Location(type):
    """Metaclass of message 'Location'."""

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
                'rosslt_msgs.msg.Location')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__location
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__location
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__location
            cls._TYPE_SUPPORT = module.type_support_msg__msg__location
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__location

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Location(metaclass=Metaclass_Location):
    """Message class 'Location'."""

    __slots__ = [
        '_source_node',
        '_location_id',
        '_expression',
    ]

    _fields_and_field_types = {
        'source_node': 'string',
        'location_id': 'int32',
        'expression': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.source_node = kwargs.get('source_node', str())
        self.location_id = kwargs.get('location_id', int())
        self.expression = kwargs.get('expression', str())

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
        if self.source_node != other.source_node:
            return False
        if self.location_id != other.location_id:
            return False
        if self.expression != other.expression:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def source_node(self):
        """Message field 'source_node'."""
        return self._source_node

    @source_node.setter
    def source_node(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'source_node' field must be of type 'str'"
        self._source_node = value

    @property
    def location_id(self):
        """Message field 'location_id'."""
        return self._location_id

    @location_id.setter
    def location_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'location_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'location_id' field must be an integer in [-2147483648, 2147483647]"
        self._location_id = value

    @property
    def expression(self):
        """Message field 'expression'."""
        return self._expression

    @expression.setter
    def expression(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'expression' field must be of type 'str'"
        self._expression = value
