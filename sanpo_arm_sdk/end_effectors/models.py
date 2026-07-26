"""Common gripper models plus Gloria-M register definitions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import time

from .errors import GripperConfigurationError


class GripperControlMode(IntEnum):
    """Control modes exposed by the Gloria-M firmware."""

    MIT = 1
    POSITION_VELOCITY = 2
    SPEED = 3


class GloriaRegister(IntEnum):
    """Gloria-M public register IDs."""

    UV_VALUE = 0
    KT_VALUE = 1
    OT_VALUE = 2
    OC_VALUE = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    DAMP = 11
    INERTIA = 12
    HW_VER = 13
    SW_VER = 14
    SN = 15
    NPP = 16
    RS = 17
    LS = 18
    FLUX = 19
    GR = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_VALUE = 29
    GREF = 30
    DETA = 31
    V_BW = 32
    IQ_C1 = 33
    VL_C1 = 34
    CAN_BR = 35
    SUB_VER = 36
    U_OFF = 50
    V_OFF = 51
    K1 = 52
    K2 = 53
    M_OFF = 54
    DIR = 55
    P_M = 80
    XOUT = 81


U32_REGISTERS = {
    GloriaRegister.MST_ID,
    GloriaRegister.ESC_ID,
    GloriaRegister.TIMEOUT,
    GloriaRegister.CTRL_MODE,
    GloriaRegister.HW_VER,
    GloriaRegister.SW_VER,
    GloriaRegister.SN,
    GloriaRegister.NPP,
    GloriaRegister.CAN_BR,
    GloriaRegister.SUB_VER,
}


@dataclass(frozen=True)
class GripperLimits:
    """Device mapping limits used by Gloria-M MIT payloads."""

    position_max_rad: float = 3.14
    velocity_max_rad_s: float = 10.0
    torque_max_nm: float = 12.0

    def __post_init__(self) -> None:
        values = (
            self.position_max_rad,
            self.velocity_max_rad_s,
            self.torque_max_nm,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise GripperConfigurationError("夹爪 PMAX/VMAX/TMAX 必须为有限正数")


@dataclass(frozen=True)
class GripperCalibration:
    """Map a normalized opening command to Gloria-M motor radians.

    ``opening_fraction=0`` means fully closed and ``1`` means fully open.
    The default endpoints come from the original SDK safety range and must be
    checked on the actual mechanism before gripping a workpiece.
    """

    closed_position_rad: float = 0.0
    open_position_rad: float = 2.7

    def __post_init__(self) -> None:
        values = (self.closed_position_rad, self.open_position_rad)
        if not all(math.isfinite(value) for value in values):
            raise GripperConfigurationError("夹爪开合标定值必须为有限数")
        if self.closed_position_rad == self.open_position_rad:
            raise GripperConfigurationError("夹爪全开和全闭位置不能相同")

    @property
    def safe_min_rad(self) -> float:
        return min(self.closed_position_rad, self.open_position_rad)

    @property
    def safe_max_rad(self) -> float:
        return max(self.closed_position_rad, self.open_position_rad)

    def position_from_fraction(self, opening_fraction: float) -> float:
        fraction = float(opening_fraction)
        if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
            raise GripperConfigurationError("opening_fraction 必须在 0..1 范围内")
        return self.closed_position_rad + fraction * (
            self.open_position_rad - self.closed_position_rad
        )

    def fraction_from_position(self, position_rad: float) -> float:
        span = self.open_position_rad - self.closed_position_rad
        return (float(position_rad) - self.closed_position_rad) / span


@dataclass(frozen=True)
class GloriaGripperConfig:
    """Runtime identity and safety configuration for one Gloria-M."""

    motor_id: int = 1
    master_can_id: int = 0
    timeout_s: float = 0.5
    startup_control_mode: GripperControlMode | None = (
        GripperControlMode.POSITION_VELOCITY
    )
    limits: GripperLimits = GripperLimits()
    calibration: GripperCalibration = GripperCalibration()

    def __post_init__(self) -> None:
        # The status payload stores the motor address in its low four bits.
        if not 0 <= int(self.motor_id) <= 0x0F:
            raise GripperConfigurationError("Gloria-M motor_id 必须在 0..15 范围内")
        if not 0 <= int(self.master_can_id) <= 0x7FF:
            raise GripperConfigurationError("master_can_id 必须在 0..0x7FF 范围内")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise GripperConfigurationError("timeout_s 必须为有限正数")
        if self.startup_control_mode is not None:
            GripperControlMode(self.startup_control_mode)


STATUS_NAMES = {
    0x0: "失能",
    0x1: "使能",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS过温",
    0xC: "线圈过温",
    0xD: "通信丢失",
    0xE: "过载",
}


@dataclass(frozen=True)
class GripperState:
    """One decoded Gloria-M feedback sample."""

    motor_id: int = 0
    status_code: int = 0
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    torque_nm: float = 0.0
    mos_temperature_c: int = 0
    rotor_temperature_c: int = 0
    opening_fraction: float | None = None
    updated_at: float = 0.0

    @property
    def status(self) -> str:
        return STATUS_NAMES.get(
            self.status_code,
            f"未知(0x{self.status_code:X})",
        )

    @property
    def age_s(self) -> float:
        return (
            float("inf")
            if not self.updated_at
            else time.monotonic() - self.updated_at
        )


__all__ = [
    "GloriaGripperConfig",
    "GloriaRegister",
    "GripperCalibration",
    "GripperControlMode",
    "GripperLimits",
    "GripperState",
    "STATUS_NAMES",
    "U32_REGISTERS",
]
