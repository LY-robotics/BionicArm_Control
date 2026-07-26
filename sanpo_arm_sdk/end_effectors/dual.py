"""Side-by-side control of two independent gripper objects."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Callable

from .base import GripperHardware
from .models import GripperState


@dataclass(frozen=True)
class DualGripperResult:
    left_success: bool
    right_success: bool
    left_error: str = ""
    right_error: str = ""
    left_available: bool = True
    right_available: bool = True

    @property
    def success(self) -> bool:
        left_ok = not self.left_available or self.left_success
        right_ok = not self.right_available or self.right_success
        return left_ok and right_ok

    @property
    def any_available(self) -> bool:
        return self.left_available or self.right_available

    def side_available(self, side: str) -> bool:
        if side == "left":
            return self.left_available
        if side == "right":
            return self.right_available
        raise ValueError("side must be left or right")


class DualGripperController:
    """Coordinate two grippers without depending on either arm controller."""

    def __init__(
        self,
        left: GripperHardware,
        right: GripperHardware,
    ) -> None:
        self.left = left
        self.right = right

    @staticmethod
    def _capture(function: Callable[[], object]) -> tuple[bool, str, object]:
        try:
            return True, "", function()
        except Exception as exc:
            return False, str(exc), None

    @staticmethod
    def _available(gripper: GripperHardware) -> bool:
        return bool(getattr(gripper, "available", True))

    @classmethod
    def _capture_side(
        cls,
        gripper: GripperHardware,
        function: Callable[[], object],
    ) -> tuple[bool, str, object, bool]:
        available = cls._available(gripper)
        if not available:
            reason = str(
                getattr(gripper, "unavailable_reason", "gripper is not configured")
            )
            return True, reason, None, False
        success, error, value = cls._capture(function)
        return success, error, value, True

    def _run_both(
        self,
        left_call: Callable[[], object],
        right_call: Callable[[], object],
    ) -> tuple[DualGripperResult, tuple[object, object]]:
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(
                self._capture_side,
                self.left,
                left_call,
            )
            right_future = executor.submit(
                self._capture_side,
                self.right,
                right_call,
            )
            left_ok, left_error, left_value, left_available = left_future.result()
            (
                right_ok,
                right_error,
                right_value,
                right_available,
            ) = right_future.result()
        return (
            DualGripperResult(
                left_success=left_ok,
                right_success=right_ok,
                left_error=left_error,
                right_error=right_error,
                left_available=left_available,
                right_available=right_available,
            ),
            (left_value, right_value),
        )

    def connect(self) -> DualGripperResult:
        return self._run_both(self.left.connect, self.right.connect)[0]

    def disconnect(self, *, disable: bool = True) -> DualGripperResult:
        return self._run_both(
            lambda: self.left.disconnect(disable=disable),
            lambda: self.right.disconnect(disable=disable),
        )[0]

    close = disconnect

    def enable_both(self) -> DualGripperResult:
        return self._run_both(self.left.enable, self.right.enable)[0]

    def disable_both(self) -> DualGripperResult:
        return self._run_both(self.left.disable, self.right.disable)[0]

    def clear_faults_both(self) -> DualGripperResult:
        return self._run_both(
            self.left.clear_faults,
            self.right.clear_faults,
        )[0]

    def set_zero_both(self, *, confirm: bool = False) -> DualGripperResult:
        return self._run_both(
            lambda: self.left.set_zero(confirm=confirm),
            lambda: self.right.set_zero(confirm=confirm),
        )[0]

    def move_both(
        self,
        left_opening: float,
        right_opening: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> tuple[DualGripperResult, dict[str, GripperState | None]]:
        result, values = self._run_both(
            lambda: self.left.move_normalized(
                left_opening,
                velocity_rad_s,
                poll=poll,
            ),
            lambda: self.right.move_normalized(
                right_opening,
                velocity_rad_s,
                poll=poll,
            ),
        )
        return result, {"left": values[0], "right": values[1]}

    def refresh_both(
        self,
    ) -> tuple[DualGripperResult, dict[str, GripperState | None]]:
        result, values = self._run_both(
            self.left.refresh_state,
            self.right.refresh_state,
        )
        return result, {"left": values[0], "right": values[1]}


__all__ = ["DualGripperController", "DualGripperResult"]
