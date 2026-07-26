"""Five-axis motion controller independent from CAN frame details."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

import numpy as np

from ..config import JOINT_KEYS
from ..errors import (
    ERR_CAN_RX_TIMEOUT,
    ERR_CAN_TX_FAILED,
    ERR_IK_NO_SOLUTION,
    ERR_INVALID_ARGUMENT,
    ERR_INVALID_JOINT_KEY,
    ERR_MOTION_BUSY,
    ERR_MOTION_CANCELLED,
    ERR_NOT_CONNECTED,
    ERR_OUT_OF_LIMIT,
    ERR_PROTOCOL_RESPONSE,
    ERR_SERIAL_OPEN_FAILED,
    ERR_UNKNOWN,
    ERR_WAIT_REACH_TIMEOUT,
    OK,
)
from ..hardware.base import ArmHardware
from ..kinematics.cartesian_line import (
    CartesianLinePlan,
    plan_cartesian_line_trajectory,
)
from ..kinematics.ideal_arm_model import IdealArmModel, build_ideal_arm_model
from ..kinematics.guiji_quintic import (
    CartesianTrajectory,
    JointTrajectory,
    plan_cartesian_point_to_point_trajectory,
    plan_quintic_joint_trajectory_by_limits,
)
from ..kinematics.kinematic_5dof import (
    ArmPose,
    IKRecommendConfig,
    IKRecommendResult,
    IKOptions,
    IKResult,
    forward_kinematics,
    inverse_kinematics,
    recommend_feasible_pitch_j5,
)
from ..settings import LINE_DEFAULTS

JOINT_LIMIT_ATOL_DEG = 1e-3


@dataclass(frozen=True)
class PreparedMotion:
    """A fully planned motion that has not sent any position command yet."""

    kind: str
    trajectory: JointTrajectory
    velocity_limit_deg_s: np.ndarray
    acceleration_limit_deg_s2: np.ndarray
    cartesian_plan: CartesianTrajectory | None = None
    line_plan: CartesianLinePlan | None = None
    ik_result: IKResult | None = None
    recommendation: IKRecommendResult | None = None


class ArmController:
    """Coordinate kinematics, trajectory planning and an arm hardware backend."""

    def __init__(self, hardware: ArmHardware) -> None:
        missing = [key for key in JOINT_KEYS if key not in hardware.joints]
        if missing:
            raise ValueError(f"Hardware is missing joints: {missing}")

        self.hardware = hardware
        self.name = hardware.name
        self.poll_interval_s = 0.05
        self.last_trajectory = None
        self.last_cartesian_plan = None
        self.last_line_plan: Optional[CartesianLinePlan] = None
        self.last_ik_result: Optional[IKResult] = None
        self.last_recommendation: Optional[IKRecommendResult] = None
        self.last_motion_error = OK
        self._motion_thread: Optional[threading.Thread] = None
        self._motion_cancel = threading.Event()
        self._executing = threading.Event()
        self._motion_lock = threading.Lock()

        # Compatibility view for existing code and menu display.
        self.joints = {
            key: {
                "name": hardware.joints[key].name,
                "id": hardware.joints[key].motor_id,
                "ratio": hardware.joints[key].ratio,
                "direction": hardware.joints[key].direction,
                "min": hardware.joints[key].min_deg,
                "max": hardware.joints[key].max_deg,
                "current": 0.0,
                "speed": hardware.joints[key].default_speed_rpm,
                "accel": hardware.joints[key].default_accel_rpm_s,
            }
            for key in JOINT_KEYS
        }

    @property
    def connected(self) -> bool:
        return bool(self.hardware.connected)

    @property
    def is_moving(self) -> bool:
        thread_running = (
            self._motion_thread is not None and self._motion_thread.is_alive()
        )
        return thread_running or self._executing.is_set()

    def _return_motion_error(self, err: int) -> int:
        self.last_motion_error = int(err)
        return self.last_motion_error

    def connect(self) -> int:
        try:
            self.hardware.connect()
            return OK
        except Exception:
            return ERR_SERIAL_OPEN_FAILED

    def close(self) -> int:
        self._motion_cancel.set()
        if self.is_moving and self._motion_thread is not threading.current_thread():
            self._motion_thread.join(timeout=1.0)
        try:
            self.hardware.close()
            return OK
        except Exception:
            return ERR_UNKNOWN

    def _connection_error(self) -> int:
        return OK if self.connected else ERR_NOT_CONNECTED

    def _validate_joint(self, joint: str) -> int:
        return OK if joint in self.joints else ERR_INVALID_JOINT_KEY

    def refresh_status(self, joint: str) -> tuple[int, Optional[dict]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        err = self._connection_error()
        if err != OK:
            return err, None

        try:
            feedback = self.hardware.read_joint_feedback(joint)
        except Exception:
            return ERR_CAN_RX_TIMEOUT, None

        data = {
            "angle_deg": feedback.angle_deg,
            "speed_rpm": feedback.speed_rpm,
            "current_a": feedback.q_current_a,
        }
        if feedback.angle_deg is not None:
            self.joints[joint]["current"] = float(feedback.angle_deg)
        if all(value is None for value in data.values()):
            return ERR_CAN_RX_TIMEOUT, data
        return OK, data

    def refresh_all(self) -> tuple[int, dict[str, Optional[dict]]]:
        result: dict[str, Optional[dict]] = {}
        last_err = OK
        for key in JOINT_KEYS:
            err, data = self.refresh_status(key)
            result[key] = data
            if err != OK:
                last_err = err
        return last_err, result

    def read_motor_status(self, joint: str) -> tuple[int, Optional[dict]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            status = self.hardware.read_joint_status(joint)
        except Exception:
            return ERR_CAN_RX_TIMEOUT, None
        if status is None:
            return ERR_PROTOCOL_RESPONSE, None
        return OK, status

    def read_motor_info(
        self,
        joint: str,
        kind: str,
    ) -> tuple[int, Optional[dict]]:
        """Read version, parameter or compact feedback through the motor API."""
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        readers = {
            "version": self.hardware.read_motor_version,
            "params": self.hardware.read_motor_params,
            "compact": self.hardware.read_motor_compact,
        }
        if kind not in readers:
            return ERR_INVALID_ARGUMENT, None
        try:
            result = readers[kind](joint)
            if result is None:
                return ERR_PROTOCOL_RESPONSE, None
            return OK, result
        except Exception:
            return ERR_CAN_RX_TIMEOUT, None

    def set_max_current(self, joint: str, amp: float) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            value = float(amp)
            if not np.isfinite(value) or value <= 0.0:
                return ERR_INVALID_ARGUMENT
            return (
                OK
                if self.hardware.set_max_current(joint, value)
                else ERR_PROTOCOL_RESPONSE
            )
        except Exception:
            return ERR_CAN_TX_FAILED

    def set_current_slope(self, joint: str, amp_per_sec: float) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            value = float(amp_per_sec)
            if not np.isfinite(value) or value <= 0.0:
                return ERR_INVALID_ARGUMENT
            return (
                OK
                if self.hardware.set_current_slope(joint, value)
                else ERR_PROTOCOL_RESPONSE
            )
        except Exception:
            return ERR_CAN_TX_FAILED

    def read_or_set_gain(
        self,
        joint: str,
        gain: str,
        value: Optional[float] = None,
    ) -> tuple[int, Optional[float]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        if gain not in {"position_kp", "position_ki", "speed_kp", "speed_ki"}:
            return ERR_INVALID_ARGUMENT, None
        try:
            parsed = None if value is None else float(value)
            if parsed is not None and not np.isfinite(parsed):
                return ERR_INVALID_ARGUMENT, None
            result = self.hardware.read_or_set_gain(joint, gain, parsed)
            if result is None:
                return ERR_PROTOCOL_RESPONSE, None
            return OK, float(result)
        except Exception:
            return ERR_CAN_TX_FAILED, None

    def set_q_current(
        self,
        joint: str,
        amp: float,
    ) -> tuple[int, Optional[float]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            value = float(amp)
            if not np.isfinite(value):
                return ERR_INVALID_ARGUMENT, None
            result = self.hardware.set_q_current(joint, value)
            if result is None:
                return ERR_PROTOCOL_RESPONSE, None
            return OK, float(result)
        except Exception:
            return ERR_CAN_TX_FAILED, None

    def set_speed_mode(
        self,
        joint: str,
        joint_rpm: float,
    ) -> tuple[int, Optional[float]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            value = float(joint_rpm)
            if not np.isfinite(value):
                return ERR_INVALID_ARGUMENT, None
            result = self.hardware.set_speed_mode(joint, value)
            if result is None:
                return ERR_PROTOCOL_RESPONSE, None
            return OK, float(result)
        except Exception:
            return ERR_CAN_TX_FAILED, None

    def go_home_shortest(self, joint: str) -> int:
        """Run motor command C4; this is not the same as planned arm home."""
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            if not self.hardware.go_home_shortest(joint):
                return ERR_PROTOCOL_RESPONSE
            self.joints[joint]["current"] = 0.0
            return OK
        except Exception:
            return ERR_CAN_TX_FAILED

    def sync_state(self) -> int:
        """Read all joint angles; movement is rejected if this cannot complete."""
        try:
            self._current_joint_array()
            return OK
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT
        except Exception:
            return ERR_UNKNOWN

    def set_speed(self, joint: str, joint_speed_rpm: float) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            value = float(joint_speed_rpm)
            if not np.isfinite(value) or value <= 0.0:
                return ERR_INVALID_ARGUMENT
            if not self.hardware.set_speed_limit(joint, value):
                return ERR_PROTOCOL_RESPONSE
            self.joints[joint]["speed"] = value
            return OK
        except Exception:
            return ERR_CAN_TX_FAILED

    def set_accel(self, joint: str, joint_accel_rpm_s: float) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            value = float(joint_accel_rpm_s)
            if not np.isfinite(value) or value <= 0.0:
                return ERR_INVALID_ARGUMENT
            if not self.hardware.set_accel_limit(joint, value):
                return ERR_PROTOCOL_RESPONSE
            self.joints[joint]["accel"] = value
            return OK
        except Exception:
            return ERR_CAN_TX_FAILED

    def configure_defaults(self) -> int:
        for key in JOINT_KEYS:
            err = self.set_speed(key, self.joints[key]["speed"])
            if err != OK:
                return err
            err = self.set_accel(key, self.joints[key]["accel"])
            if err != OK:
                return err
        return OK

    def clear_faults(self) -> int:
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            result = self.hardware.clear_faults()
            return OK if all(result.values()) else ERR_PROTOCOL_RESPONSE
        except Exception:
            return ERR_CAN_TX_FAILED

    def set_zero(self, joint: str) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            if not self.hardware.set_zero(joint):
                return ERR_PROTOCOL_RESPONSE
            self.joints[joint]["current"] = 0.0
            return OK
        except Exception:
            return ERR_CAN_TX_FAILED

    def set_zero_all(self) -> int:
        for key in JOINT_KEYS:
            err = self.set_zero(key)
            if err != OK:
                return err
        return OK

    def set_brake(self, joint: str, closed: bool) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        try:
            result = self.hardware.set_brake(joint, bool(closed))
            return OK if result is not None else ERR_PROTOCOL_RESPONSE
        except Exception:
            return ERR_CAN_TX_FAILED

    def read_brake(self, joint: str) -> tuple[int, Optional[bool]]:
        err = self._validate_joint(joint)
        if err != OK:
            return err, None
        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            result = self.hardware.read_brake(joint)
            if result is None:
                return ERR_PROTOCOL_RESPONSE, None
            return OK, bool(result)
        except Exception:
            return ERR_CAN_RX_TIMEOUT, None

    def _joint_array(self, value: object, name: str = "joint_angles") -> np.ndarray:
        if hasattr(value, "as_array"):
            array = np.asarray(value.as_array(), dtype=float).reshape(-1)
        else:
            array = np.asarray(value, dtype=float).reshape(-1)
        if array.size != len(JOINT_KEYS):
            raise ValueError(f"{name} must contain {len(JOINT_KEYS)} values")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain finite values")
        return array

    def _motion_limit_array(self, value: object, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float).reshape(-1)
        if array.size == 1:
            array = np.full(len(JOINT_KEYS), float(array[0]), dtype=float)
        elif array.size != len(JOINT_KEYS):
            raise ValueError(f"{name} must be a scalar or contain 5 values")
        if not np.all(np.isfinite(array)) or np.any(array <= 0.0):
            raise ValueError(f"{name} must contain finite positive values")
        return array

    def _within_limit(self, key: str, target_deg: float) -> bool:
        cfg = self.joints[key]
        return (
            cfg["min"] - JOINT_LIMIT_ATOL_DEG
            <= target_deg
            <= cfg["max"] + JOINT_LIMIT_ATOL_DEG
        )

    def _validate_joint_targets(self, targets_deg: Sequence[float]) -> int:
        for index, key in enumerate(JOINT_KEYS):
            if not self._within_limit(key, float(targets_deg[index])):
                return ERR_OUT_OF_LIMIT
        return OK

    def _clip_joint_targets(self, targets_deg: Sequence[float]) -> np.ndarray:
        targets = self._joint_array(targets_deg, "targets_deg").copy()
        for index, key in enumerate(JOINT_KEYS):
            cfg = self.joints[key]
            if (
                targets[index] < cfg["min"]
                and cfg["min"] - targets[index] <= JOINT_LIMIT_ATOL_DEG
            ):
                targets[index] = cfg["min"]
            if (
                targets[index] > cfg["max"]
                and targets[index] - cfg["max"] <= JOINT_LIMIT_ATOL_DEG
            ):
                targets[index] = cfg["max"]
        return targets

    def _current_joint_array(self) -> np.ndarray:
        if not self.connected:
            raise RuntimeError("Arm is not connected")
        current = []
        for key in JOINT_KEYS:
            value = self.hardware.read_joint_angle(key)
            if value is None:
                raise TimeoutError(f"No angle feedback from {key}")
            self.joints[key]["current"] = float(value)
            current.append(float(value))
        return np.asarray(current, dtype=float)

    def _send_joint_target_array(self, targets_deg: Sequence[float]) -> int:
        targets = self._joint_array(targets_deg, "targets_deg")
        err = self._validate_joint_targets(targets)
        if err != OK:
            return err
        targets = self._clip_joint_targets(targets)
        try:
            self.hardware.command_positions(
                {key: float(targets[index]) for index, key in enumerate(JOINT_KEYS)}
            )
        except ValueError:
            return ERR_OUT_OF_LIMIT
        except Exception:
            return ERR_CAN_TX_FAILED
        for index, key in enumerate(JOINT_KEYS):
            self.joints[key]["current"] = float(targets[index])
        return OK

    def _wait_joint_target_array(
        self,
        targets_deg: Sequence[float],
        *,
        timeout_s: float,
        tol_deg: float,
    ) -> int:
        targets = self._joint_array(targets_deg, "targets_deg")
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self._motion_cancel.is_set():
                return ERR_MOTION_CANCELLED
            all_reached = True
            for index, key in enumerate(JOINT_KEYS):
                try:
                    angle = self.hardware.read_joint_angle(key)
                except Exception:
                    angle = None
                if angle is None:
                    all_reached = False
                    continue
                self.joints[key]["current"] = float(angle)
                if abs(float(angle) - float(targets[index])) > float(tol_deg):
                    all_reached = False
            if all_reached:
                return OK
            self._motion_cancel.wait(self.poll_interval_s)
        return ERR_WAIT_REACH_TIMEOUT

    def _apply_trajectory_limits(
        self,
        velocity_deg_s: Sequence[float],
        acceleration_deg_s2: Sequence[float],
    ) -> int:
        velocity = self._motion_limit_array(velocity_deg_s, "velocity_deg_s")
        acceleration = self._motion_limit_array(
            acceleration_deg_s2, "acceleration_deg_s2"
        )
        for index, key in enumerate(JOINT_KEYS):
            err = self.set_speed(key, velocity[index] / 6.0)
            if err != OK:
                return err
            err = self.set_accel(key, acceleration[index] / 6.0)
            if err != OK:
                return err
        return OK

    def _execute_joint_trajectory(
        self,
        trajectory: object,
        *,
        wait_final: bool,
        timeout_s: Optional[float],
        tol_deg: float,
        start_event: Optional[threading.Event] = None,
    ) -> int:
        self._executing.set()
        try:
            if start_event is not None:
                while not start_event.wait(0.05):
                    if self._motion_cancel.is_set():
                        return self._return_motion_error(ERR_MOTION_CANCELLED)

            command_q_deg = np.asarray(trajectory.command_q_deg, dtype=float)
            start_time = time.monotonic()
            for index, target in enumerate(command_q_deg):
                if self._motion_cancel.is_set():
                    return self._return_motion_error(ERR_MOTION_CANCELLED)
                err = self._send_joint_target_array(target)
                if err != OK:
                    return self._return_motion_error(err)
                if index + 1 < trajectory.point_count:
                    next_time = start_time + float(trajectory.time_s[index + 1])
                    if self._motion_cancel.wait(
                        max(0.0, next_time - time.monotonic())
                    ):
                        return self._return_motion_error(ERR_MOTION_CANCELLED)

            if not wait_final:
                return self._return_motion_error(OK)
            final_timeout_s = 5.0 if timeout_s is None else float(timeout_s)
            return self._return_motion_error(
                self._wait_joint_target_array(
                    command_q_deg[-1],
                    timeout_s=final_timeout_s,
                    tol_deg=tol_deg,
                )
            )
        finally:
            self._executing.clear()

    def _trajectory_worker(
        self,
        trajectory: object,
        tol_deg: float,
        start_event: Optional[threading.Event] = None,
    ) -> None:
        self._execute_joint_trajectory(
            trajectory,
            wait_final=False,
            timeout_s=None,
            tol_deg=tol_deg,
            start_event=start_event,
        )

    def _start_trajectory_thread(
        self,
        trajectory: object,
        tol_deg: float,
        start_event: Optional[threading.Event] = None,
    ) -> int:
        with self._motion_lock:
            if self.is_moving:
                return self._return_motion_error(ERR_MOTION_BUSY)
            self._motion_cancel.clear()
            self._motion_thread = threading.Thread(
                target=self._trajectory_worker,
                args=(trajectory, tol_deg, start_event),
                daemon=True,
                name=f"{self.name}-trajectory",
            )
            self._motion_thread.start()
        return self._return_motion_error(OK)

    def wait_motion(self, timeout_s: Optional[float] = None) -> int:
        thread = self._motion_thread
        if thread is None:
            return self.last_motion_error
        thread.join(timeout=timeout_s)
        return ERR_MOTION_BUSY if thread.is_alive() else self.last_motion_error

    def stop_motion(self, *, disable: bool = True) -> int:
        self._motion_cancel.set()
        if self.is_moving and self._motion_thread is not threading.current_thread():
            self._motion_thread.join(timeout=1.0)
        if not disable:
            return self._return_motion_error(ERR_MOTION_CANCELLED)
        if not self.connected:
            return self._return_motion_error(ERR_NOT_CONNECTED)
        try:
            success = self.hardware.disable_all()
            return self._return_motion_error(OK if success else ERR_PROTOCOL_RESPONSE)
        except Exception:
            return self._return_motion_error(ERR_CAN_TX_FAILED)

    def disable_all(self) -> int:
        return self.stop_motion(disable=True)

    def move_absolute(
        self,
        joint: str,
        target_deg: float,
        timeout_s: float = 5.0,
        tol_deg: float = 1.0,
    ) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        if not self.connected:
            return ERR_NOT_CONNECTED
        target = float(target_deg)
        if not np.isfinite(target):
            return ERR_INVALID_ARGUMENT
        if not self._within_limit(joint, target):
            return ERR_OUT_OF_LIMIT
        try:
            self.hardware.command_positions({joint: target})
        except Exception:
            return ERR_CAN_TX_FAILED

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            try:
                angle = self.hardware.read_joint_angle(joint)
            except Exception:
                angle = None
            if angle is not None and abs(float(angle) - target) <= float(tol_deg):
                self.joints[joint]["current"] = float(angle)
                return OK
            time.sleep(self.poll_interval_s)
        return ERR_WAIT_REACH_TIMEOUT

    def move_relative(
        self,
        joint: str,
        delta_deg: float,
        timeout_s: float = 5.0,
        tol_deg: float = 1.0,
    ) -> int:
        err = self._validate_joint(joint)
        if err != OK:
            return err
        try:
            current = self.hardware.read_joint_angle(joint)
        except Exception:
            return ERR_CAN_RX_TIMEOUT
        if current is None:
            return ERR_CAN_RX_TIMEOUT
        return self.move_absolute(
            joint,
            float(current) + float(delta_deg),
            timeout_s=timeout_s,
            tol_deg=tol_deg,
        )

    def set_pose(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        timeout_s: float = 8.0,
        tol_deg: float = 1.0,
    ) -> int:
        targets = [j1, j2, j3, j4, j5]
        err = self._send_joint_target_array(targets)
        if err != OK:
            return err
        self._motion_cancel.clear()
        return self._wait_joint_target_array(
            targets,
            timeout_s=timeout_s,
            tol_deg=tol_deg,
        )

    def _is_vector_argument(self, value: object, allowed_sizes: set[int]) -> bool:
        if hasattr(value, "as_array") or hasattr(value, "position_mm"):
            return True
        if isinstance(value, dict):
            return True
        if isinstance(value, (str, bytes)):
            return False
        try:
            array = np.asarray(value, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return False
        return int(array.size) in allowed_sizes

    def _parse_movej_args(
        self,
        args: tuple,
        speed: object,
        accel: object,
        blocking: bool,
    ) -> tuple[np.ndarray, object, object, bool]:
        if not args:
            raise ValueError("MoveJ requires a joint target")
        if self._is_vector_argument(args[0], {5}):
            target = args[0]
            rest = args[1:]
        elif len(args) >= 5:
            target = args[:5]
            rest = args[5:]
        else:
            raise ValueError("MoveJ target must contain five joint angles")
        if len(rest) > 3:
            raise ValueError("MoveJ accepts target, speed, accel, blocking")
        if len(rest) >= 1:
            speed = rest[0]
        if len(rest) >= 2:
            accel = rest[1]
        if len(rest) >= 3:
            blocking = bool(rest[2])
        return self._joint_array(target), speed, accel, bool(blocking)

    def _parse_movecart_args(
        self,
        args: tuple,
        speed: object,
        accel: object,
        blocking: bool,
    ) -> tuple[object, object, object, bool]:
        if not args:
            raise ValueError("MoveCart requires a Cartesian target pose")
        if self._is_vector_argument(args[0], {3, 4, 5}):
            target_pose = args[0]
            rest = args[1:]
        elif len(args) in (3, 4, 5):
            target_pose = args
            rest = ()
        elif len(args) >= 6:
            target_pose = args[:5]
            rest = args[5:]
        else:
            raise ValueError("Invalid Cartesian target")
        if len(rest) > 3:
            raise ValueError("MoveCart accepts target, speed, accel, blocking")
        if len(rest) >= 1:
            speed = rest[0]
        if len(rest) >= 2:
            accel = rest[1]
        if len(rest) >= 3:
            blocking = bool(rest[2])
        return target_pose, speed, accel, bool(blocking)

    def _validate_trajectory_targets(self, trajectory: JointTrajectory) -> int:
        for target in np.asarray(trajectory.command_q_deg, dtype=float):
            err = self._validate_joint_targets(target)
            if err != OK:
                return err
        return OK

    def prepare_move_j(
        self,
        target: object,
        *,
        speed: object = 10.0,
        accel: object = 20.0,
        sample_period_s: float = 0.05,
    ) -> tuple[int, Optional[PreparedMotion]]:
        """Plan a joint move without changing motor state."""

        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            target_array = self._joint_array(target)
            velocity = self._motion_limit_array(speed, "speed")
            acceleration = self._motion_limit_array(accel, "accel")
            err = self._validate_joint_targets(target_array)
            if err != OK:
                return err, None
            q_start = self._current_joint_array()
            trajectory = plan_quintic_joint_trajectory_by_limits(
                q_start=q_start,
                q_goal=target_array,
                velocity_limit_deg_s=velocity,
                acceleration_limit_deg_s2=acceleration,
                sample_period_s=sample_period_s,
            )
            return (
                OK,
                PreparedMotion(
                    kind="joint",
                    trajectory=trajectory,
                    velocity_limit_deg_s=velocity,
                    acceleration_limit_deg_s2=acceleration,
                ),
            )
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except (TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    def prepare_move_cart(
        self,
        target_pose: object,
        *,
        speed: object = 10.0,
        accel: object = 20.0,
        sample_period_s: float = 0.05,
        ik_options: Optional[IKOptions] = None,
        allow_recommendation: bool = False,
        recommendation_config: Optional[IKRecommendConfig] = None,
    ) -> tuple[int, Optional[PreparedMotion]]:
        """Plan Cartesian point-to-point motion, optionally accepting an IK fallback."""

        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            velocity = self._motion_limit_array(speed, "speed")
            acceleration = self._motion_limit_array(accel, "accel")
            position, pitch, j5 = self._parse_cartesian_target(target_pose)
            q_start = self._current_joint_array()
            recommendation: IKRecommendResult | None = None
            planning_target: object = target_pose

            if allow_recommendation and pitch is not None and j5 is not None:
                recommendation = recommend_feasible_pitch_j5(
                    position,
                    pitch,
                    j5,
                    q_start,
                    q_reference=q_start,
                    config=recommendation_config,
                )
                if not recommendation.success:
                    self.last_recommendation = recommendation
                    return ERR_IK_NO_SOLUTION, None
                planning_target = [
                    *position,
                    recommendation.recommended_pitch_deg,
                    recommendation.recommended_j5_deg,
                ]

            cartesian_plan = plan_cartesian_point_to_point_trajectory(
                q_start=q_start,
                target_pose=planning_target,
                velocity_limit_deg_s=velocity,
                acceleration_limit_deg_s2=acceleration,
                ik_options=ik_options,
                sample_period_s=sample_period_s,
            )
            trajectory = cartesian_plan.joint_trajectory
            err = self._validate_trajectory_targets(trajectory)
            if err != OK:
                return err, None
            return (
                OK,
                PreparedMotion(
                    kind="cartesian_ptp",
                    trajectory=trajectory,
                    velocity_limit_deg_s=velocity,
                    acceleration_limit_deg_s2=acceleration,
                    cartesian_plan=cartesian_plan,
                    ik_result=cartesian_plan.ik_result,
                    recommendation=recommendation,
                ),
            )
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except ValueError as exc:
            if str(exc).startswith("Cartesian inverse kinematics failed"):
                return ERR_IK_NO_SOLUTION, None
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    def prepare_move_line(
        self,
        target_pose: object,
        *,
        speed: object = 10.0,
        accel: object = 20.0,
        total_time_s: Optional[float] = None,
        sample_period_s: float = LINE_DEFAULTS.sample_period_s,
        minimum_duration_s: float = LINE_DEFAULTS.minimum_duration_s,
        position_tolerance_mm: float = LINE_DEFAULTS.position_tolerance_mm,
        max_branch_jump_deg: float = LINE_DEFAULTS.max_branch_jump_deg,
    ) -> tuple[int, Optional[PreparedMotion]]:
        """Plan a TCP straight line without sending commands to the motors."""

        if not self.connected:
            return ERR_NOT_CONNECTED, None
        try:
            velocity = self._motion_limit_array(speed, "speed")
            acceleration = self._motion_limit_array(accel, "accel")
            position, pitch, j5 = self._parse_cartesian_target(target_pose)
            q_start = self._current_joint_array()
            line_plan = plan_cartesian_line_trajectory(
                q_start,
                position,
                reference_pitch_deg=pitch,
                target_j5_deg=j5,
                velocity_limit_deg_s=velocity,
                acceleration_limit_deg_s2=acceleration,
                total_time_s=total_time_s,
                sample_period_s=sample_period_s,
                minimum_duration_s=minimum_duration_s,
                position_tolerance_mm=position_tolerance_mm,
                max_branch_jump_deg=max_branch_jump_deg,
                velocity_margin=LINE_DEFAULTS.velocity_margin,
                acceleration_margin=LINE_DEFAULTS.acceleration_margin,
            )
            err = self._validate_trajectory_targets(line_plan.joint_trajectory)
            if err != OK:
                return err, None
            return (
                OK,
                PreparedMotion(
                    kind="cartesian_line",
                    trajectory=line_plan.joint_trajectory,
                    velocity_limit_deg_s=velocity,
                    acceleration_limit_deg_s2=acceleration,
                    line_plan=line_plan,
                    ik_result=line_plan.final_ik_result,
                ),
            )
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except RuntimeError:
            return ERR_IK_NO_SOLUTION, None
        except (TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    def configure_prepared_motion(self, motion: PreparedMotion) -> int:
        """Validate a prepared path and apply motor speed/acceleration limits."""

        if not self.connected:
            return self._return_motion_error(ERR_NOT_CONNECTED)
        if self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        err = self._validate_trajectory_targets(motion.trajectory)
        if err == OK:
            err = self._apply_trajectory_limits(
                motion.velocity_limit_deg_s,
                motion.acceleration_limit_deg_s2,
            )
        if err != OK:
            return self._return_motion_error(err)

        self.last_trajectory = motion.trajectory
        self.last_cartesian_plan = motion.cartesian_plan
        self.last_line_plan = motion.line_plan
        self.last_ik_result = motion.ik_result
        self.last_recommendation = motion.recommendation
        return self._return_motion_error(OK)

    def execute_prepared_motion(
        self,
        motion: PreparedMotion,
        *,
        blocking: bool = True,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
        start_event: Optional[threading.Event] = None,
        configured: bool = False,
    ) -> int:
        """Execute an offline plan, optionally waiting on a shared start signal."""

        if not configured:
            err = self.configure_prepared_motion(motion)
            if err != OK:
                return err
        elif self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        self._motion_cancel.clear()
        if blocking:
            return self._execute_joint_trajectory(
                motion.trajectory,
                wait_final=True,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
                start_event=start_event,
            )
        return self._start_trajectory_thread(
            motion.trajectory,
            tol_deg,
            start_event=start_event,
        )

    def MoveJ(
        self,
        *args,
        speed: object = 10.0,
        accel: object = 20.0,
        blocking: bool = True,
        sample_period_s: float = 0.05,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
    ) -> int:
        if not self.connected:
            return self._return_motion_error(ERR_NOT_CONNECTED)
        if self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        try:
            target, velocity, acceleration, blocking = self._parse_movej_args(
                args, speed, accel, blocking
            )
            velocity = self._motion_limit_array(velocity, "speed")
            acceleration = self._motion_limit_array(acceleration, "accel")
        except (TypeError, ValueError):
            return self._return_motion_error(ERR_INVALID_ARGUMENT)
        err = self._validate_joint_targets(target)
        if err != OK:
            return self._return_motion_error(err)

        try:
            q_start = self._current_joint_array()
            trajectory = plan_quintic_joint_trajectory_by_limits(
                q_start=q_start,
                q_goal=target,
                velocity_limit_deg_s=velocity,
                acceleration_limit_deg_s2=acceleration,
                sample_period_s=sample_period_s,
            )
        except TimeoutError:
            return self._return_motion_error(ERR_CAN_RX_TIMEOUT)
        except (TypeError, ValueError):
            return self._return_motion_error(ERR_INVALID_ARGUMENT)
        except Exception:
            return self._return_motion_error(ERR_UNKNOWN)

        err = self._apply_trajectory_limits(velocity, acceleration)
        if err != OK:
            return self._return_motion_error(err)
        self.last_trajectory = trajectory
        self.last_cartesian_plan = None
        self.last_line_plan = None
        self.last_ik_result = None
        self.last_recommendation = None
        self._motion_cancel.clear()
        if blocking:
            return self._execute_joint_trajectory(
                trajectory,
                wait_final=True,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
            )
        return self._start_trajectory_thread(trajectory, tol_deg)

    def MoveCart(
        self,
        *args,
        speed: object = 10.0,
        accel: object = 20.0,
        blocking: bool = True,
        sample_period_s: float = 0.05,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
        ik_options: Optional[IKOptions] = None,
    ) -> int:
        if not self.connected:
            return self._return_motion_error(ERR_NOT_CONNECTED)
        if self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        try:
            target_pose, velocity, acceleration, blocking = (
                self._parse_movecart_args(args, speed, accel, blocking)
            )
            velocity = self._motion_limit_array(velocity, "speed")
            acceleration = self._motion_limit_array(acceleration, "accel")
            q_start = self._current_joint_array()
            cartesian_plan = plan_cartesian_point_to_point_trajectory(
                q_start=q_start,
                target_pose=target_pose,
                velocity_limit_deg_s=velocity,
                acceleration_limit_deg_s2=acceleration,
                ik_options=ik_options,
                sample_period_s=sample_period_s,
            )
        except TimeoutError:
            return self._return_motion_error(ERR_CAN_RX_TIMEOUT)
        except ValueError as exc:
            if str(exc).startswith("Cartesian inverse kinematics failed"):
                return self._return_motion_error(ERR_IK_NO_SOLUTION)
            return self._return_motion_error(ERR_INVALID_ARGUMENT)
        except Exception:
            return self._return_motion_error(ERR_UNKNOWN)

        trajectory = cartesian_plan.joint_trajectory
        err = self._validate_joint_targets(cartesian_plan.ik_result.q_deg)
        if err != OK:
            return self._return_motion_error(err)
        err = self._apply_trajectory_limits(velocity, acceleration)
        if err != OK:
            return self._return_motion_error(err)

        self.last_cartesian_plan = cartesian_plan
        self.last_line_plan = None
        self.last_ik_result = cartesian_plan.ik_result
        self.last_recommendation = None
        self.last_trajectory = trajectory
        self._motion_cancel.clear()
        if blocking:
            return self._execute_joint_trajectory(
                trajectory,
                wait_final=True,
                timeout_s=timeout_s,
                tol_deg=tol_deg,
            )
        return self._start_trajectory_thread(trajectory, tol_deg)

    def MoveCartRecommended(
        self,
        target_pose: object,
        *,
        speed: object = 10.0,
        accel: object = 20.0,
        blocking: bool = True,
        sample_period_s: float = 0.05,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
        recommendation_config: Optional[IKRecommendConfig] = None,
    ) -> int:
        """Move to the nearest feasible pitch/J5 pose when the request is infeasible."""

        if self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        err, motion = self.prepare_move_cart(
            target_pose,
            speed=speed,
            accel=accel,
            sample_period_s=sample_period_s,
            allow_recommendation=True,
            recommendation_config=recommendation_config,
        )
        if err != OK or motion is None:
            return self._return_motion_error(err)
        return self.execute_prepared_motion(
            motion,
            blocking=blocking,
            timeout_s=timeout_s,
            tol_deg=tol_deg,
        )

    def MoveLine(
        self,
        target_pose: object,
        *,
        speed: object = 10.0,
        accel: object = 20.0,
        blocking: bool = True,
        total_time_s: Optional[float] = None,
        sample_period_s: float = LINE_DEFAULTS.sample_period_s,
        minimum_duration_s: float = LINE_DEFAULTS.minimum_duration_s,
        position_tolerance_mm: float = LINE_DEFAULTS.position_tolerance_mm,
        max_branch_jump_deg: float = LINE_DEFAULTS.max_branch_jump_deg,
        timeout_s: Optional[float] = None,
        tol_deg: float = 1.0,
    ) -> int:
        """Move the TCP along a sampled straight line with continuous IK."""

        if self.is_moving:
            return self._return_motion_error(ERR_MOTION_BUSY)
        err, motion = self.prepare_move_line(
            target_pose,
            speed=speed,
            accel=accel,
            total_time_s=total_time_s,
            sample_period_s=sample_period_s,
            minimum_duration_s=minimum_duration_s,
            position_tolerance_mm=position_tolerance_mm,
            max_branch_jump_deg=max_branch_jump_deg,
        )
        if err != OK or motion is None:
            return self._return_motion_error(err)
        return self.execute_prepared_motion(
            motion,
            blocking=blocking,
            timeout_s=timeout_s,
            tol_deg=tol_deg,
        )

    def home(
        self,
        *,
        speed: float = 10.0,
        accel: float = 20.0,
        blocking: bool = True,
    ) -> int:
        return self.MoveJ(
            [0.0] * len(JOINT_KEYS),
            speed=speed,
            accel=accel,
            blocking=blocking,
        )

    def forward_pose(
        self, joints_deg: Optional[Sequence[float]] = None
    ) -> tuple[int, Optional[ArmPose]]:
        try:
            q = self._current_joint_array() if joints_deg is None else joints_deg
            return OK, forward_kinematics(q)
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except (TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    def ideal_model(
        self,
        joints_deg: Optional[Sequence[float]] = None,
        *,
        check_limits: bool = True,
    ) -> tuple[int, Optional[IdealArmModel]]:
        """Return the ideal link/frame model for supplied or live joint angles."""

        try:
            q = self._current_joint_array() if joints_deg is None else joints_deg
            return OK, build_ideal_arm_model(q, check_limits=check_limits)
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except (TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    @staticmethod
    def _parse_cartesian_target(
        target_pose: object,
    ) -> tuple[np.ndarray, Optional[float], Optional[float]]:
        if isinstance(target_pose, Mapping):
            position = target_pose.get("position_mm")
            if position is None:
                position = [
                    target_pose["x"],
                    target_pose["y"],
                    target_pose["z"],
                ]
            pitch = target_pose.get("pitch_deg", target_pose.get("pitch"))
            j5 = target_pose.get("j5_deg", target_pose.get("j5"))
        else:
            array = np.asarray(target_pose, dtype=float).reshape(-1)
            if array.size not in (3, 4, 5):
                raise ValueError("Target must have x, y, z and optional pitch/J5")
            position = array[:3]
            pitch = None if array.size < 4 else float(array[3])
            j5 = None if array.size < 5 else float(array[4])
        position_array = np.asarray(position, dtype=float).reshape(-1)
        if position_array.size != 3 or not np.all(np.isfinite(position_array)):
            raise ValueError("Position must contain three finite values")
        return (
            position_array,
            None if pitch is None else float(pitch),
            None if j5 is None else float(j5),
        )

    def preview_ik(
        self,
        target_pose: object,
        *,
        ik_options: Optional[IKOptions] = None,
    ) -> tuple[int, Optional[IKResult]]:
        try:
            position, pitch, j5 = self._parse_cartesian_target(target_pose)
            q_start = self._current_joint_array()
            result = inverse_kinematics(
                position,
                q_seed=q_start,
                target_pitch_deg=pitch,
                target_j5_deg=j5,
                q_reference=q_start,
                options=ik_options,
            )
            return (OK if result.success else ERR_IK_NO_SOLUTION), result
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except (KeyError, TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    def preview_ik_recommendation(
        self,
        target_pose: object,
        *,
        config: Optional[IKRecommendConfig] = None,
    ) -> tuple[int, Optional[IKRecommendResult]]:
        """Return the requested IK or the nearest feasible pitch/J5 alternative."""

        try:
            position, pitch, j5 = self._parse_cartesian_target(target_pose)
            if pitch is None or j5 is None:
                return ERR_INVALID_ARGUMENT, None
            q_start = self._current_joint_array()
            result = recommend_feasible_pitch_j5(
                position,
                pitch,
                j5,
                q_start,
                q_reference=q_start,
                config=config,
            )
            self.last_recommendation = result
            self.last_ik_result = result.ik_result
            return (OK if result.success else ERR_IK_NO_SOLUTION), result
        except TimeoutError:
            return ERR_CAN_RX_TIMEOUT, None
        except (KeyError, TypeError, ValueError):
            return ERR_INVALID_ARGUMENT, None
        except Exception:
            return ERR_UNKNOWN, None

    move_j = MoveJ
    move_cart = MoveCart
    move_cart_recommended = MoveCartRecommended
    move_line = MoveLine


__all__ = ["ArmController", "PreparedMotion"]
