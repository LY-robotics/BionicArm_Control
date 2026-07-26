"""In-memory gripper implementing the same API as GloriaGripper."""

from __future__ import annotations

import math
import time

from .errors import GripperConfigurationError, GripperConnectionError
from .models import GloriaGripperConfig, GripperState


class SimulatedGripper:
    available = True

    def __init__(
        self,
        *,
        name: str = "simulated_gripper",
        config: GloriaGripperConfig | None = None,
    ) -> None:
        self.name = name
        self.config = config or GloriaGripperConfig()
        self.connected = False
        self.enabled = False
        self.state = self._state(self.config.calibration.open_position_rad)

    def _state(self, position_rad: float) -> GripperState:
        return GripperState(
            motor_id=self.config.motor_id,
            status_code=1 if self.enabled else 0,
            position_rad=position_rad,
            velocity_rad_s=0.0,
            torque_nm=0.0,
            mos_temperature_c=25,
            rotor_temperature_c=25,
            opening_fraction=self.config.calibration.fraction_from_position(
                position_rad
            ),
            updated_at=time.monotonic(),
        )

    def _require_connected(self) -> None:
        if not self.connected:
            raise GripperConnectionError(f"{self.name} 尚未连接")

    def connect(self) -> None:
        self.connected = True

    def disconnect(self, *, disable: bool = True) -> None:
        if disable and self.connected:
            self.disable()
        self.connected = False

    close = disconnect

    def enable(self) -> None:
        self._require_connected()
        self.enabled = True
        self.state = self._state(self.state.position_rad)

    def disable(self) -> None:
        self._require_connected()
        self.enabled = False
        self.state = self._state(self.state.position_rad)

    def clear_faults(self) -> None:
        self._require_connected()

    def set_zero(self, *, confirm: bool = False) -> None:
        if not confirm:
            raise GripperConfigurationError(
                "Setting the gripper zero requires confirm=True"
            )
        self._require_connected()
        self.state = self._state(0.0)

    def move_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        self._require_connected()
        calibration = self.config.calibration
        if (
            not math.isfinite(position_rad)
            or not calibration.safe_min_rad
            <= position_rad
            <= calibration.safe_max_rad
        ):
            raise GripperConfigurationError("夹爪位置超出标定范围")
        if (
            not math.isfinite(velocity_rad_s)
            or velocity_rad_s <= 0.0
            or velocity_rad_s > self.config.limits.velocity_max_rad_s
        ):
            raise GripperConfigurationError("夹爪速度超出允许范围")
        self.state = self._state(float(position_rad))
        return self.state if poll else None

    def move_normalized(
        self,
        opening_fraction: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        return self.move_position(
            self.config.calibration.position_from_fraction(opening_fraction),
            velocity_rad_s,
            poll=poll,
        )

    def open_fingers(
        self,
        velocity_rad_s: float = 0.2,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        return self.move_normalized(1.0, velocity_rad_s, poll=poll)

    def close_fingers(
        self,
        velocity_rad_s: float = 0.2,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        return self.move_normalized(0.0, velocity_rad_s, poll=poll)

    def refresh_state(self) -> GripperState:
        self._require_connected()
        self.state = self._state(self.state.position_rad)
        return self.state


__all__ = ["SimulatedGripper"]
