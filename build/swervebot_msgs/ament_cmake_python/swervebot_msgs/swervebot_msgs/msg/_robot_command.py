# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swervebot_msgs:msg/RobotCommand.idl
# generated code does not contain a copyright notice

from __future__ import annotations

import collections.abc
import os
import typing

import rosidl_pycommon.interface_base_classes

if typing.TYPE_CHECKING:
    from ctypes import Structure

    class PyCapsule(Structure):
        pass  # don't need to define the full structure


# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
ros_python_check_fields = os.getenv('ROS_PYTHON_CHECK_FIELDS', default='')


if typing.TYPE_CHECKING:
    import geometry_msgs.msg  # noqa: E402, I100, I201, I300


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotCommand(rosidl_pycommon.interface_base_classes.MessageTypeSupportMeta):
    """Metaclass of message 'RobotCommand'."""

    _CREATE_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_FROM_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_TO_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _DESTROY_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _TYPE_SUPPORT: typing.ClassVar[typing.Optional[PyCapsule]] = None

    class RobotCommandConstants(typing.TypedDict):
        ACKERMANN: typing.Literal[0]
        CRAB_WALK: typing.Literal[1]
        ZERO_TURN: typing.Literal[2]

    __constants: RobotCommandConstants = {
        'ACKERMANN': 0,
        'CRAB_WALK': 1,
        'ZERO_TURN': 2,
    }

    @classmethod
    def __import_type_support__(cls) -> None:
        try:
            from rosidl_generator_py import import_type_support  # type: ignore[attr-defined]
            module = import_type_support('swervebot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swervebot_msgs.msg.RobotCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_command

            from geometry_msgs.msg import Twist
            if Twist._TYPE_SUPPORT is None:
                Twist.__import_type_support__()

    @classmethod
    def __prepare__(metacls, name: str, bases: tuple[type[typing.Any], ...], /, **kwds: typing.Any) -> collections.abc.MutableMapping[str, object]:
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ACKERMANN': metacls.__constants['ACKERMANN'],
            'CRAB_WALK': metacls.__constants['CRAB_WALK'],
            'ZERO_TURN': metacls.__constants['ZERO_TURN'],
        }

    @property
    def ACKERMANN(self) -> typing.Literal[0]:
        """Message constant 'ACKERMANN'."""
        return Metaclass_RobotCommand.__constants['ACKERMANN']

    @property
    def CRAB_WALK(self) -> typing.Literal[1]:
        """Message constant 'CRAB_WALK'."""
        return Metaclass_RobotCommand.__constants['CRAB_WALK']

    @property
    def ZERO_TURN(self) -> typing.Literal[2]:
        """Message constant 'ZERO_TURN'."""
        return Metaclass_RobotCommand.__constants['ZERO_TURN']


class RobotCommand(rosidl_pycommon.interface_base_classes.BaseMessage, metaclass=Metaclass_RobotCommand):
    """
    Message class 'RobotCommand'.

    Constants:
      ACKERMANN
      CRAB_WALK
      ZERO_TURN
    """

    __slots__ = [
        '_twist',
        '_mode',
        '_check_fields',
    ]

    _fields_and_field_types: dict[str, str] = {
        'twist': 'geometry_msgs/Twist',
        'mode': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES: tuple[rosidl_parser.definition.AbstractType, ...] = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Twist'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, *,
                 twist: typing.Optional[geometry_msgs.msg.Twist] = None,  # noqa: E501
                 mode: typing.Optional[int] = None,  # noqa: E501
                 check_fields: typing.Optional[bool] = None) -> None:
        if check_fields is not None:
            self._check_fields = check_fields
        else:
            self._check_fields = ros_python_check_fields == '1'
        from geometry_msgs.msg import Twist
        self.twist = twist if twist is not None else Twist()
        self.mode = mode if mode is not None else int()

    def __repr__(self) -> str:
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args: list[str] = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    from rosidl_buffer import Buffer as _RosidlBuffer
                    if not isinstance(field, _RosidlBuffer):
                        if self._check_fields:
                            assert fieldstr.startswith('array(')
                        prefix = "array('X', "
                        suffix = ')'
                        fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotCommand):
            return False
        if self.twist != other.twist:
            return False
        if self.mode != other.mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def twist(self) -> geometry_msgs.msg.Twist:
        """Message field 'twist'."""
        return self._twist

    @twist.setter
    def twist(self, value: geometry_msgs.msg.Twist) -> None:
        from geometry_msgs.msg import Twist

        if self._check_fields:
            if False:  # Done for templating alignment
                pass
            else:
                assert \
                    isinstance(value, Twist), \
                    "The 'twist' field must be a sub message of type 'Twist'"

        self._twist = value

    @builtins.property
    def mode(self) -> int:
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value: int) -> None:

        if self._check_fields:
            if False:  # Done for templating alignment
                pass
            else:
                assert \
                    isinstance(value, int), \
                    "The 'mode' field must be of type 'int'"
                assert value >= 0 and value < 256, \
                    "The 'mode' field must be an unsigned integer in [0, 255]"

        self._mode = value
