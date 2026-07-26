"""SANPO USB CDC/CAN backend built on can_motor_arm_lib.py."""

from typing import Dict, Mapping, Optional, Sequence

from ..protocol.can_motor_arm_lib import (
    CanArm,
    JointConfig,
    JointFeedback,
    SerialUsbCanTransport,
)


class SanpoCanBackend:
    """Own one STM32/F4 serial port and one five-joint arm."""

    def __init__(
        self,
        port: str,
        joints: Sequence[JointConfig],
        *,
        name: str,
        baudrate: int = 1_000_000,
        serial_timeout_s: float = 0.02,
        response_timeout_s: float = 0.08,
        usb_mode: str = "advanced",
        channel: int = 0,
        use_host_id_offset: bool = True,
        debug: bool = False,
        transport: Optional[SerialUsbCanTransport] = None,
    ) -> None:
        self.name = name
        self.port = port
        self.connected = False
        self._owns_transport = transport is None
        self.transport = transport or SerialUsbCanTransport(
            port=port,
            baudrate=baudrate,
            timeout=serial_timeout_s,
            debug=debug,
            usb_mode=usb_mode,
            channel=channel,
        )
        self.bus = (
            self.transport.channel_endpoint(channel)
            if transport is not None
            else self.transport
        )
        self.arm = CanArm(
            self.bus,
            joints,
            name=name,
            use_host_id_offset=use_host_id_offset,
            response_timeout=response_timeout_s,
        )
        self.joints = self.arm.joints

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError(f"{self.name} is not connected")

    def connect(self) -> None:
        self.transport.connect()
        self.connected = True

    def close(self) -> None:
        if self._owns_transport:
            self.transport.close()
        self.connected = False

    def read_joint_angle(self, key: str) -> Optional[float]:
        self._require_connected()
        return self.arm.read_joint_angle(key)

    def read_joint_feedback(self, key: str) -> JointFeedback:
        self._require_connected()
        return self.arm.read_joint_motion_feedback(key)

    def read_joint_status(self, key: str) -> Optional[dict]:
        self._require_connected()
        return self.arm.motors[key].read_status()

    def read_motor_version(self, key: str) -> Optional[dict]:
        self._require_connected()
        return self.arm.motors[key].read_version()

    def read_motor_params(self, key: str) -> Optional[dict]:
        self._require_connected()
        return self.arm.motors[key].read_motor_params()

    def read_motor_compact(self, key: str) -> Optional[dict]:
        self._require_connected()
        return self.arm.motors[key].read_compact()

    def set_speed_limit(self, key: str, joint_rpm: float) -> bool:
        self._require_connected()
        return self.arm.set_joint_speed_limit(key, joint_rpm)

    def set_accel_limit(self, key: str, joint_rpm_s: float) -> bool:
        self._require_connected()
        return self.arm.set_joint_accel_limit(key, joint_rpm_s)

    def set_max_current(self, key: str, amp: float) -> bool:
        self._require_connected()
        return self.arm.motors[key].set_max_q_current(amp)

    def set_current_slope(self, key: str, amp_per_sec: float) -> bool:
        self._require_connected()
        return self.arm.motors[key].set_q_current_slope(amp_per_sec)

    def read_or_set_gain(
        self,
        key: str,
        gain: str,
        value: Optional[float],
    ) -> Optional[float]:
        self._require_connected()
        methods = {
            "position_kp": self.arm.motors[key].read_or_set_position_kp,
            "position_ki": self.arm.motors[key].read_or_set_position_ki,
            "speed_kp": self.arm.motors[key].read_or_set_speed_kp,
            "speed_ki": self.arm.motors[key].read_or_set_speed_ki,
        }
        if gain not in methods:
            raise ValueError(f"Unknown gain {gain}")
        return methods[gain](value)

    def set_q_current(self, key: str, amp: float) -> Optional[float]:
        self._require_connected()
        return self.arm.motors[key].set_q_current(amp)

    def set_speed_mode(self, key: str, joint_rpm: float) -> Optional[float]:
        self._require_connected()
        return self.arm.set_joint_speed_mode(key, joint_rpm)

    def go_home_shortest(self, key: str) -> bool:
        self._require_connected()
        return self.arm.motors[key].go_home_shortest() is not None

    def command_positions(self, targets: Mapping[str, float]) -> None:
        self._require_connected()
        # Keep all five targets for one trajectory sample adjacent.  The
        # transport lock is reentrant, so each nested motor send remains safe.
        with self.bus.batch():
            self.arm.command_many_absolute(dict(targets), delay_s=0.0)

    def clear_faults(self) -> Dict[str, bool]:
        self._require_connected()
        return {
            key: self.arm.motors[key].clear_fault() is not None
            for key in self.arm.keys()
        }

    def set_zero(self, key: str) -> bool:
        self._require_connected()
        return self.arm.set_zero(key) is not None

    def set_brake(self, key: str, closed: bool) -> Optional[bool]:
        self._require_connected()
        return self.arm.motors[key].set_brake(closed)

    def read_brake(self, key: str) -> Optional[bool]:
        self._require_connected()
        return self.arm.motors[key].read_brake()

    def disable_all(self) -> bool:
        self._require_connected()
        success = True
        for key in self.arm.keys():
            success = self.arm.motors[key].disable() is not None and success
        return success
