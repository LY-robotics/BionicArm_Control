"""Explicit placeholder for an end effector that is not installed."""

from __future__ import annotations

from .errors import GripperConnectionError
from .models import GripperState


class UnavailableGripper:
    """Keep the dual-side API stable when one gripper is not configured."""

    available = False
    connected = False
    enabled = False

    def __init__(self, *, name: str, reason: str = "gripper is not configured") -> None:
        self.name = name
        self.unavailable_reason = reason
        self.state = GripperState()

    def _raise_unavailable(self) -> None:
        raise GripperConnectionError(f"{self.name}: {self.unavailable_reason}")

    def connect(self) -> None:
        self._raise_unavailable()

    def disconnect(self, *, disable: bool = True) -> None:
        del disable

    close = disconnect

    def enable(self) -> None:
        self._raise_unavailable()

    def disable(self) -> None:
        self._raise_unavailable()

    def clear_faults(self) -> None:
        self._raise_unavailable()

    def set_zero(self, *, confirm: bool = False) -> None:
        del confirm
        self._raise_unavailable()

    def move_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        del position_rad, velocity_rad_s, poll
        self._raise_unavailable()

    def move_normalized(
        self,
        opening_fraction: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        del opening_fraction, velocity_rad_s, poll
        self._raise_unavailable()

    def open_fingers(
        self,
        velocity_rad_s: float = 0.2,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        del velocity_rad_s, poll
        self._raise_unavailable()

    def close_fingers(
        self,
        velocity_rad_s: float = 0.2,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        del velocity_rad_s, poll
        self._raise_unavailable()

    def refresh_state(self) -> GripperState:
        self._raise_unavailable()


__all__ = ["UnavailableGripper"]
