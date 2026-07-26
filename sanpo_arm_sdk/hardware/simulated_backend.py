"""In-memory backend for validating motion and menu flows without hardware."""

from typing import Dict, Mapping, Optional, Sequence

from ..protocol.can_motor_arm_lib import JointConfig, JointFeedback


class SimulatedArmBackend:
    def __init__(
        self,
        joints: Sequence[JointConfig],
        *,
        name: str = "simulated_arm",
        initial_positions: Optional[Mapping[str, float]] = None,
    ) -> None:
        self.name = name
        self.connected = False
        self.joints = {cfg.key: cfg for cfg in joints}
        self.positions = {key: 0.0 for key in self.joints}
        if initial_positions:
            self.positions.update(
                {key: float(value) for key, value in initial_positions.items()}
            )
        self.speed_limits = {
            key: cfg.default_speed_rpm for key, cfg in self.joints.items()
        }
        self.accel_limits = {
            key: cfg.default_accel_rpm_s for key, cfg in self.joints.items()
        }
        self.brakes = {key: False for key in self.joints}
        self.max_currents = {key: None for key in self.joints}
        self.current_slopes = {key: None for key in self.joints}
        self.gains = {
            key: {
                "position_kp": 0.0,
                "position_ki": 0.0,
                "speed_kp": 0.0,
                "speed_ki": 0.0,
            }
            for key in self.joints
        }
        self.enabled = True

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError(f"{self.name} is not connected")

    def connect(self) -> None:
        self.connected = True
        self.enabled = True

    def close(self) -> None:
        self.connected = False

    def read_joint_angle(self, key: str) -> Optional[float]:
        self._require_connected()
        return self.positions[key]

    def read_joint_feedback(self, key: str) -> JointFeedback:
        self._require_connected()
        return JointFeedback(
            key=key,
            motor_id=self.joints[key].motor_id,
            angle_deg=self.positions[key],
            speed_rpm=0.0,
            q_current_a=0.0,
            status=self.read_joint_status(key),
        )

    def read_joint_status(self, key: str) -> Optional[dict]:
        self._require_connected()
        return {
            "bus_voltage_v": 48.0,
            "bus_current_a": 0.0,
            "temperature_c": 25,
            "mode": 4 if self.enabled else 0,
            "mode_text": "position" if self.enabled else "disabled",
            "fault_code": 0,
            "faults": [],
        }

    def read_motor_version(self, key: str) -> Optional[dict]:
        self._require_connected()
        return {
            "boot_version_raw": 0,
            "app_version_raw": 0,
            "hardware_version_raw": 0,
            "can_protocol_version_raw": 0,
        }

    def read_motor_params(self, key: str) -> Optional[dict]:
        self._require_connected()
        return {
            "pole_pairs": 0,
            "torque_constant_n_per_a": 0.0,
            "reduction_ratio_raw": self.joints[key].ratio,
        }

    def read_motor_compact(self, key: str) -> Optional[dict]:
        self._require_connected()
        return {
            "temperature_c": 25.0,
            "q_current_a": 0.0,
            "speed_rpm": 0.0,
            "single_count": 0,
            "single_deg": self.positions[key] % 360.0,
        }

    def set_speed_limit(self, key: str, joint_rpm: float) -> bool:
        self._require_connected()
        self.speed_limits[key] = float(joint_rpm)
        return True

    def set_accel_limit(self, key: str, joint_rpm_s: float) -> bool:
        self._require_connected()
        self.accel_limits[key] = float(joint_rpm_s)
        return True

    def set_max_current(self, key: str, amp: float) -> bool:
        self._require_connected()
        self.max_currents[key] = abs(float(amp))
        return True

    def set_current_slope(self, key: str, amp_per_sec: float) -> bool:
        self._require_connected()
        self.current_slopes[key] = abs(float(amp_per_sec))
        return True

    def read_or_set_gain(
        self,
        key: str,
        gain: str,
        value: Optional[float],
    ) -> Optional[float]:
        self._require_connected()
        if value is not None:
            self.gains[key][gain] = float(value)
        return self.gains[key][gain]

    def set_q_current(self, key: str, amp: float) -> Optional[float]:
        self._require_connected()
        return float(amp)

    def set_speed_mode(self, key: str, joint_rpm: float) -> Optional[float]:
        self._require_connected()
        return float(joint_rpm)

    def go_home_shortest(self, key: str) -> bool:
        self._require_connected()
        self.positions[key] = 0.0
        return True

    def command_positions(self, targets: Mapping[str, float]) -> None:
        self._require_connected()
        for key, value in targets.items():
            cfg = self.joints[key]
            target = float(value)
            if not cfg.min_deg <= target <= cfg.max_deg:
                raise ValueError(f"{key} target is outside limits")
        self.positions.update({key: float(value) for key, value in targets.items()})
        self.enabled = True

    def clear_faults(self) -> Dict[str, bool]:
        self._require_connected()
        return {key: True for key in self.joints}

    def set_zero(self, key: str) -> bool:
        self._require_connected()
        self.positions[key] = 0.0
        return True

    def set_brake(self, key: str, closed: bool) -> Optional[bool]:
        self._require_connected()
        self.brakes[key] = bool(closed)
        return self.brakes[key]

    def read_brake(self, key: str) -> Optional[bool]:
        self._require_connected()
        return self.brakes[key]

    def disable_all(self) -> bool:
        self._require_connected()
        self.enabled = False
        return True
