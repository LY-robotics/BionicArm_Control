"""Pure Gloria-M CAN payload encoding and decoding.

This module contains no serial-port or SANPO-board code.  The same Gloria
driver can therefore be reused with USB CDC, SocketCAN, or another CAN backend.
"""

from __future__ import annotations

import math
import struct
import time

from .errors import GripperConfigurationError, GripperProtocolError
from .models import (
    GloriaRegister,
    GripperCalibration,
    GripperLimits,
    GripperState,
)


ENABLE_PAYLOAD = b"\xff\xff\xff\xff\xff\xff\xff\xfc"
DISABLE_PAYLOAD = b"\xff\xff\xff\xff\xff\xff\xff\xfd"
SET_ZERO_PAYLOAD = b"\xff\xff\xff\xff\xff\xff\xff\xfe"
CLEAR_ERROR_PAYLOAD = b"\xff\xff\xff\xff\xff\xff\xff\xfb"


def _require_finite(name: str, value: float) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise GripperConfigurationError(f"{name} 必须为有限数")
    return number


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    """Map a bounded float to an unsigned protocol field without clipping."""

    number = _require_finite("value", value)
    if bits <= 0 or high <= low:
        raise GripperConfigurationError("无效的协议映射范围")
    if not low <= number <= high:
        raise GripperConfigurationError(
            f"数值 {number} 超出协议范围 [{low}, {high}]"
        )
    return int(round((number - low) * (2**bits - 1) / (high - low)))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    if bits <= 0 or high <= low or not 0 <= int(value) < 2**bits:
        raise GripperProtocolError("无效的无符号协议字段")
    return float(value) * (high - low) / (2**bits - 1) + low


def pack_mit(
    position_rad: float,
    velocity_rad_s: float,
    kp: float,
    kd: float,
    torque_nm: float,
    limits: GripperLimits,
) -> bytes:
    """Encode a strict P16/V12/Kp12/Kd12/T12 MIT command."""

    position = float_to_uint(
        position_rad,
        -limits.position_max_rad,
        limits.position_max_rad,
        16,
    )
    velocity = float_to_uint(
        velocity_rad_s,
        -limits.velocity_max_rad_s,
        limits.velocity_max_rad_s,
        12,
    )
    kp_raw = float_to_uint(kp, 0.0, 500.0, 12)
    kd_raw = float_to_uint(kd, 0.0, 5.0, 12)
    torque = float_to_uint(
        torque_nm,
        -limits.torque_max_nm,
        limits.torque_max_nm,
        12,
    )
    return bytes(
        [
            position >> 8,
            position & 0xFF,
            velocity >> 4,
            ((velocity & 0x0F) << 4) | (kp_raw >> 8),
            kp_raw & 0xFF,
            kd_raw >> 4,
            ((kd_raw & 0x0F) << 4) | (torque >> 8),
            torque & 0xFF,
        ]
    )


def pack_position_velocity(position_rad: float, velocity_rad_s: float) -> bytes:
    position = _require_finite("position_rad", position_rad)
    velocity = _require_finite("velocity_rad_s", velocity_rad_s)
    return struct.pack("<ff", position, velocity)


def parse_feedback(
    data: bytes,
    limits: GripperLimits,
    *,
    calibration: GripperCalibration | None = None,
    expected_motor_id: int | None = None,
) -> GripperState:
    """Decode one eight-byte Gloria-M motion feedback payload."""

    if len(data) != 8:
        raise GripperProtocolError("Gloria-M 状态反馈必须为 8 字节")
    motor_id = data[0] & 0x0F
    if expected_motor_id is not None and motor_id != expected_motor_id:
        raise GripperProtocolError(
            f"反馈 motor_id={motor_id}，期望 {expected_motor_id}"
        )
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    position_rad = uint_to_float(
        position_raw,
        -limits.position_max_rad,
        limits.position_max_rad,
        16,
    )
    opening_fraction = (
        None
        if calibration is None
        else calibration.fraction_from_position(position_rad)
    )
    return GripperState(
        motor_id=motor_id,
        status_code=data[0] >> 4,
        position_rad=position_rad,
        velocity_rad_s=uint_to_float(
            velocity_raw,
            -limits.velocity_max_rad_s,
            limits.velocity_max_rad_s,
            12,
        ),
        torque_nm=uint_to_float(
            torque_raw,
            -limits.torque_max_nm,
            limits.torque_max_nm,
            12,
        ),
        mos_temperature_c=data[6],
        rotor_temperature_c=data[7],
        opening_fraction=opening_fraction,
        updated_at=time.monotonic(),
    )


def register_request(
    motor_id: int,
    operation: int,
    register: int | GloriaRegister,
    value: bytes = b"",
) -> bytes:
    """Build one Gloria-M register request payload."""

    if not 0 <= int(motor_id) <= 0x7FF:
        raise GripperConfigurationError("motor_id 超出 11 位")
    register_id = int(register)
    if not 0 <= register_id <= 0xFF:
        raise GripperConfigurationError("register ID 超出 8 位")
    if operation in (0x33, 0xAA) and not value:
        return bytes(
            [
                motor_id & 0xFF,
                (motor_id >> 8) & 0xFF,
                operation,
                register_id,
            ]
        )
    if operation == 0x55 and len(value) == 4:
        return bytes(
            [
                motor_id & 0xFF,
                (motor_id >> 8) & 0xFF,
                operation,
                register_id,
            ]
        ) + value
    raise GripperConfigurationError("寄存器操作或数据长度无效")


__all__ = [
    "CLEAR_ERROR_PAYLOAD",
    "DISABLE_PAYLOAD",
    "ENABLE_PAYLOAD",
    "SET_ZERO_PAYLOAD",
    "float_to_uint",
    "pack_mit",
    "pack_position_velocity",
    "parse_feedback",
    "register_request",
    "uint_to_float",
]
