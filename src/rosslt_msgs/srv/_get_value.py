# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosslt_msgs:srv\GetValue.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetValue_Request(type):
    """Metaclass of message 'GetValue_Request'."""

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
                'rosslt_msgs.srv.GetValue_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_value__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_value__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_value__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_value__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_value__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetValue_Request(metaclass=Metaclass_GetValue_Request):
    """Message class 'GetValue_Request'."""

    __slots__ = [
        '_location_id',
    ]

    _fields_and_field_types = {
        'location_id': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.location_id = kwargs.get('location_id', int())

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
        if self.location_id != other.location_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_GetValue_Response(type):
    """Metaclass of message 'GetValue_Response'."""

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
                'rosslt_msgs.srv.GetValue_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_value__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_value__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_value__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_value__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_value__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetValue_Response(metaclass=Metaclass_GetValue_Response):
    """Message class 'GetValue_Response'."""

    __slots__ = [
        '_valid_id',
        '_current_value',
    ]

    _fields_and_field_types = {
        'valid_id': 'boolean',
        'current_value': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.valid_id = kwargs.get('valid_id', bool())
        self.current_value = kwargs.get('current_value', str())

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
        if self.valid_id != other.valid_id:
            return False
        if self.current_value != other.current_value:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def valid_id(self):
        """Message field 'valid_id'."""
        return self._valid_id

    @valid_id.setter
    def valid_id(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid_id' field must be of type 'bool'"
        self._valid_id = value

    @property
    def current_value(self):
        """Message field 'current_value'."""
        return self._current_value

    @current_value.setter
    def current_value(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_value' field must be of type 'str'"
        self._current_value = value


class Metaclass_GetValue(type):
    """Metaclass of service 'GetValue'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rosslt_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosslt_msgs.srv.GetValue')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_value

            from src.rosslt_msgs.srv import _get_value
            if _get_value.Metaclass_GetValue_Request._TYPE_SUPPORT is None:
                _get_value.Metaclass_GetValue_Request.__import_type_support__()
            if _get_value.Metaclass_GetValue_Response._TYPE_SUPPORT is None:
                _get_value.Metaclass_GetValue_Response.__import_type_support__()


class GetValue(metaclass=Metaclass_GetValue):
    from src.rosslt_msgs.srv._get_value import GetValue_Request as Request
    from src.rosslt_msgs.srv._get_value import GetValue_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
