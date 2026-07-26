"""Replaceable end-effector interface used by the application layer."""

from __future__ import annotations

from typing import Protocol

from .models import GripperState


class GripperHardware(Protocol):
    """Common API implemented by real and simulated grippers."""

    name: str
    available: bool
    connected: bool

    def connect(self) -> None: ...
    def disconnect(self, *, disable: bool = True) -> None: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def clear_faults(self) -> None: ...
    def set_zero(self, *, confirm: bool = False) -> None: ...
    def move_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None: ...
    def move_normalized(
        self,
        opening_fraction: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None: ...
    def open_fingers(
        self,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None: ...
    def close_fingers(
        self,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None: ...
    def refresh_state(self) -> GripperState: ...


__all__ = ["GripperHardware"]
