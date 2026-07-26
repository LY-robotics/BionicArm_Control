"""Exceptions raised by end-effector drivers."""


class GripperError(Exception):
    """Base exception for gripper operations."""


class GripperConnectionError(GripperError):
    """The gripper transport is unavailable."""


class GripperCommunicationError(GripperError):
    """A CAN request failed or timed out."""


class GripperProtocolError(GripperError):
    """A Gloria-M payload is malformed or inconsistent."""


class GripperConfigurationError(GripperError, ValueError):
    """A gripper ID, limit, calibration, or command is invalid."""


__all__ = [
    "GripperCommunicationError",
    "GripperConfigurationError",
    "GripperConnectionError",
    "GripperError",
    "GripperProtocolError",
]
