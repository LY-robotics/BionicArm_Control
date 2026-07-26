"""Independent Gloria-M gripper controller using an injected CAN endpoint."""

from __future__ import annotations

import math
import struct
import threading
from dataclasses import replace
from typing import Optional, Protocol

from ..protocol.can_motor_arm_lib import CanFrame
from .errors import (
    GripperCommunicationError,
    GripperConfigurationError,
    GripperConnectionError,
)
from .gloria_protocol import (
    CLEAR_ERROR_PAYLOAD,
    DISABLE_PAYLOAD,
    ENABLE_PAYLOAD,
    SET_ZERO_PAYLOAD,
    pack_mit,
    pack_position_velocity,
    parse_feedback,
    register_request,
)
from .models import (
    GloriaGripperConfig,
    GloriaRegister,
    GripperControlMode,
    GripperLimits,
    GripperState,
    U32_REGISTERS,
)


class CanEndpoint(Protocol):
    """CAN operations required by the Gloria-M controller."""

    is_open: bool

    def connect(self) -> None: ...
    def close(self) -> None: ...
    def send_std(self, can_id: int, data: bytes, flush_rx: bool = True) -> None: ...
    def request(
        self,
        can_id: int,
        data: bytes,
        expect_reply_id: Optional[int] = None,
        expect_cmd: Optional[int] = None,
        timeout: float = 0.25,
        *,
        match=None,
    ) -> Optional[CanFrame]: ...


class GloriaGripper:
    """Control one Gloria-M without owning any arm-motion behavior.

    The supplied transport should be a channel-bound endpoint when the gripper
    shares one F4 USB serial port with an arm on a different physical CAN port.
    """

    available = True

    def __init__(
        self,
        transport: CanEndpoint,
        *,
        name: str = "gloria_gripper",
        config: GloriaGripperConfig | None = None,
        owns_transport: bool = False,
    ) -> None:
        self.name = name
        self.transport = transport
        self.config = config or GloriaGripperConfig()
        self.owns_transport = owns_transport
        self.connected = False
        self.enabled = False
        self.current_mode: GripperControlMode | None = None
        self.feedback_can_id: int | None = None
        self.feedback_channel: int | None = None
        self.state = GripperState()
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            try:
                self.transport.connect()
            except Exception as exc:
                raise GripperConnectionError(
                    f"{self.name} 无法连接 CAN 端点: {exc}"
                ) from exc
            self.connected = True

    def disconnect(self, *, disable: bool = True) -> None:
        with self._lock:
            try:
                if disable and self.connected:
                    self.disable()
            finally:
                if self.owns_transport:
                    self.transport.close()
                self.connected = False

    close = disconnect

    def __enter__(self) -> "GloriaGripper":
        self.connect()
        return self

    def __exit__(self, *_args: object) -> None:
        self.disconnect(disable=True)

    def _require_connected(self) -> None:
        if not self.connected or not self.transport.is_open:
            raise GripperConnectionError(f"{self.name} 尚未连接")

    def _send(self, can_id: int, data: bytes) -> None:
        self._require_connected()
        try:
            self.transport.send_std(can_id, data)
        except Exception as exc:
            raise GripperCommunicationError(
                f"{self.name} CAN 发送失败: {exc}"
            ) from exc

    def _command(self, data: bytes) -> None:
        self._send(self.config.motor_id, data)

    def enable(self) -> None:
        with self._lock:
            startup_mode = self.config.startup_control_mode
            if startup_mode is not None and self.current_mode != startup_mode:
                # The dashboard sends PV commands, so select the matching
                # volatile mode before enabling. This does not write Flash.
                self.set_mode(startup_mode, save=False)
            self._command(ENABLE_PAYLOAD)
            self.enabled = True

    def disable(self) -> None:
        with self._lock:
            self._command(DISABLE_PAYLOAD)
            self.enabled = False

    def clear_faults(self) -> None:
        with self._lock:
            self._command(CLEAR_ERROR_PAYLOAD)

    def set_zero(self, *, confirm: bool = False) -> None:
        if not confirm:
            raise GripperConfigurationError(
                "设置夹爪零点会写入设备，请显式传入 confirm=True"
            )
        with self._lock:
            self._command(SET_ZERO_PAYLOAD)

    def _check_position(self, position_rad: float) -> float:
        value = float(position_rad)
        calibration = self.config.calibration
        if not math.isfinite(value):
            raise GripperConfigurationError("夹爪位置必须为有限数")
        if not calibration.safe_min_rad <= value <= calibration.safe_max_rad:
            raise GripperConfigurationError(
                f"位置 {value} rad 超出标定安全范围 "
                f"[{calibration.safe_min_rad}, {calibration.safe_max_rad}]"
            )
        return value

    def _check_velocity(self, velocity_rad_s: float) -> float:
        value = float(velocity_rad_s)
        if not math.isfinite(value) or value <= 0.0:
            raise GripperConfigurationError("夹爪速度必须为有限正数")
        if value > self.config.limits.velocity_max_rad_s:
            raise GripperConfigurationError(
                f"夹爪速度超过 {self.config.limits.velocity_max_rad_s} rad/s"
            )
        return value

    def move_position(
        self,
        position_rad: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        """Move using Gloria-M PV mode with explicit rad/rad-s units."""

        position = self._check_position(position_rad)
        velocity = self._check_velocity(velocity_rad_s)
        with self._lock:
            self._send(
                0x100 + self.config.motor_id,
                pack_position_velocity(position, velocity),
            )
            return self.refresh_state() if poll else None

    def move_normalized(
        self,
        opening_fraction: float,
        velocity_rad_s: float,
        *,
        poll: bool = True,
    ) -> GripperState | None:
        position = self.config.calibration.position_from_fraction(
            opening_fraction
        )
        return self.move_position(position, velocity_rad_s, poll=poll)

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

    def send_mit(
        self,
        *,
        position_rad: float,
        velocity_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.5,
        torque_nm: float = 0.0,
        poll: bool = True,
    ) -> GripperState | None:
        position = self._check_position(position_rad)
        with self._lock:
            payload = pack_mit(
                position,
                velocity_rad_s,
                kp,
                kd,
                torque_nm,
                self.config.limits,
            )
            self._command(payload)
            return self.refresh_state() if poll else None

    def _is_state_frame(self, frame: CanFrame) -> bool:
        return (
            len(frame.data) == 8
            and (frame.data[0] & 0x0F) == self.config.motor_id
        )

    def _remember_feedback_route(self, frame: CanFrame) -> None:
        """Record the route reported by the device without rejecting it."""

        self.feedback_can_id = frame.can_id
        self.feedback_channel = frame.channel

    def refresh_state(self) -> GripperState:
        """Actively request one status frame from this gripper."""

        with self._lock:
            self._require_connected()
            motor_id = self.config.motor_id
            request_data = bytes(
                [
                    motor_id & 0xFF,
                    (motor_id >> 8) & 0xFF,
                    0xCC,
                    0,
                    0,
                    0,
                    0,
                    0,
                ]
            )
            try:
                frame = self.transport.request(
                    0x7FF,
                    request_data,
                    # The original Gloria SDK accepts an unknown Master ID and
                    # identifies the device from the payload. Dedicated gripper
                    # CAN ports make that discovery unambiguous.
                    expect_reply_id=None,
                    timeout=self.config.timeout_s,
                    match=self._is_state_frame,
                )
            except Exception as exc:
                raise GripperCommunicationError(
                    f"{self.name} 状态请求失败: {exc}"
                ) from exc
            if frame is None:
                raise GripperCommunicationError(
                    f"{self.name} 等待状态反馈超时"
                )
            self._remember_feedback_route(frame)
            self.state = parse_feedback(
                frame.data,
                self.config.limits,
                calibration=self.config.calibration,
                expected_motor_id=self.config.motor_id,
            )
            return self.state

    def _register_match(
        self,
        register: GloriaRegister,
        operation: int,
    ):
        motor_id = self.config.motor_id

        def match(frame: CanFrame) -> bool:
            data = frame.data
            return (
                len(data) == 8
                and data[0] == (motor_id & 0xFF)
                and data[1] == ((motor_id >> 8) & 0xFF)
                and data[2] == operation
                and data[3] == int(register)
            )

        return match

    def _request_register(
        self,
        register: GloriaRegister,
        operation: int,
        value: bytes = b"",
    ) -> bytes:
        self._require_connected()
        try:
            frame = self.transport.request(
                0x7FF,
                register_request(
                    self.config.motor_id,
                    operation,
                    register,
                    value,
                ),
                expect_reply_id=None,
                timeout=self.config.timeout_s,
                match=self._register_match(register, operation),
            )
        except Exception as exc:
            raise GripperCommunicationError(
                f"{self.name} 寄存器 {register.name} 操作失败: {exc}"
            ) from exc
        if frame is None:
            raise GripperCommunicationError(
                f"{self.name} 寄存器 {register.name} 操作超时"
            )
        self._remember_feedback_route(frame)
        return frame.data

    def read_register(self, register: int | GloriaRegister) -> int | float:
        with self._lock:
            selected = GloriaRegister(int(register))
            data = self._request_register(selected, 0x33)
            fmt = "<I" if selected in U32_REGISTERS else "<f"
            return struct.unpack(fmt, data[4:8])[0]

    def write_register(
        self,
        register: int | GloriaRegister,
        value: int | float,
        *,
        save: bool = False,
    ) -> int | float:
        with self._lock:
            selected = GloriaRegister(int(register))
            if selected in U32_REGISTERS:
                raw = struct.pack("<I", int(value))
                fmt = "<I"
            else:
                number = float(value)
                if not math.isfinite(number):
                    raise GripperConfigurationError(
                        "浮点寄存器值必须为有限数"
                    )
                raw = struct.pack("<f", number)
                fmt = "<f"
            data = self._request_register(selected, 0x55, raw)
            result = struct.unpack(fmt, data[4:8])[0]
            if save:
                self.save_register(selected)
            return result

    def save_register(self, register: int | GloriaRegister) -> None:
        """Persist one register after an explicit caller request."""

        with self._lock:
            selected = GloriaRegister(int(register))
            self._require_connected()
            motor_id = self.config.motor_id

            def match(frame: CanFrame) -> bool:
                data = frame.data
                return (
                    len(data) >= 4
                    and data[0] == (motor_id & 0xFF)
                    and data[1] == ((motor_id >> 8) & 0xFF)
                    and data[2] == 0xAA
                    and data[3] == 1
                )

            frame = self.transport.request(
                0x7FF,
                register_request(motor_id, 0xAA, selected),
                expect_reply_id=None,
                timeout=self.config.timeout_s,
                match=match,
            )
            if frame is None:
                raise GripperCommunicationError(
                    f"{self.name} 保存寄存器 {selected.name} 超时"
                )
            self._remember_feedback_route(frame)

    def set_mode(
        self,
        mode: GripperControlMode,
        *,
        save: bool = False,
    ) -> GripperControlMode:
        selected = GripperControlMode(mode)
        result = self.write_register(
            GloriaRegister.CTRL_MODE,
            int(selected),
            save=save,
        )
        if int(result) != int(selected):
            raise GripperCommunicationError("夹爪控制模式回读不一致")
        self.current_mode = selected
        return selected

    def apply_limits(
        self,
        limits: GripperLimits,
        *,
        save: bool = False,
    ) -> None:
        """Write Gloria-M mapping limits; Flash save is opt-in."""

        values = (
            (GloriaRegister.PMAX, limits.position_max_rad),
            (GloriaRegister.VMAX, limits.velocity_max_rad_s),
            (GloriaRegister.TMAX, limits.torque_max_nm),
        )
        for register, value in values:
            self.write_register(register, value, save=save)
        self.config = replace(self.config, limits=limits)


__all__ = ["CanEndpoint", "GloriaGripper"]
