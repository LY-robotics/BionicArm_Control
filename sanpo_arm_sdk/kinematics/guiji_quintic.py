"""Quintic joint-space trajectory planning for the 5-DOF arm."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

try:
    from .kinematic_5dof import IKOptions, IKResult, inverse_kinematics
except ImportError:  # pragma: no cover - supports direct script-style imports.
    from kinematic_5dof import IKOptions, IKResult, inverse_kinematics


ArrayLike = Iterable[float] | np.ndarray
_QUINTIC_MAX_VELOCITY_COEFF = 1.875
_QUINTIC_MAX_ACCELERATION_COEFF = 10.0 * np.sqrt(3.0) / 3.0


@dataclass(frozen=True)
class JointTrajectory:
    """Joint trajectory container used by the CAN execution program."""

    time_s: np.ndarray
    q_deg: np.ndarray
    qd_deg_s: np.ndarray
    qdd_deg_s2: np.ndarray
    q_cmd_deg: np.ndarray | None = None
    compensation_enabled: bool = False
    compensation_info: dict | None = None

    @property
    def point_count(self) -> int:
        return int(self.q_deg.shape[0])

    @property
    def max_joint_velocity_deg_s(self) -> np.ndarray:
        return np.max(np.abs(self.qd_deg_s), axis=0)

    @property
    def max_joint_acceleration_deg_s2(self) -> np.ndarray:
        return np.max(np.abs(self.qdd_deg_s2), axis=0)

    @property
    def command_q_deg(self) -> np.ndarray:
        if self.q_cmd_deg is None:
            return self.q_deg
        return self.q_cmd_deg

    @property
    def max_compensation_abs_deg(self) -> np.ndarray:
        if self.q_cmd_deg is None:
            return np.zeros(5, dtype=float)
        return np.max(np.abs(self.q_cmd_deg - self.q_deg), axis=0)


@dataclass(frozen=True)
class CartesianTrajectory:
    """Cartesian point-to-point plan represented as a joint trajectory."""

    joint_trajectory: JointTrajectory
    ik_result: IKResult
    target_position_mm: np.ndarray
    target_yaw_deg: float | None
    target_j5_deg: float | None


def _joint_to_array(q: object) -> np.ndarray:
    """Convert JointAngle or a 5-element array-like object to ndarray."""
    if hasattr(q, "as_array"):
        array = np.asarray(q.as_array(), dtype=float).reshape(-1)
    else:
        array = np.asarray(q, dtype=float).reshape(-1)

    if array.size != 5:
        raise ValueError("关节角必须包含 J1~J5 共 5 个数值")
    if not np.all(np.isfinite(array)):
        raise ValueError("关节角中存在 NaN 或无穷大")
    return array


def _as_5_array(value: ArrayLike, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size != 5:
        raise ValueError(f"{name} 必须包含 5 个数值，对应 J1~J5")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 中存在 NaN 或无穷大")
    if np.any(array < 0.0):
        raise ValueError(f"{name} 必须是非负补偿量")
    return array


def _as_motion_limit_array(value: ArrayLike | float, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 1:
        array = np.full(5, float(array[0]), dtype=float)
    elif array.size != 5:
        raise ValueError(f"{name} must be a scalar or contain 5 values")

    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or infinity")
    if np.any(array <= 0.0):
        raise ValueError(f"{name} must be positive")
    return array


def _smoothstep(x: np.ndarray | float) -> np.ndarray | float:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def make_friction_compensated_q_cmd(
    time_s: np.ndarray,
    q_deg: np.ndarray,
    qd_deg_s: np.ndarray,
    comp_pos_deg: ArrayLike,
    comp_neg_deg: ArrayLike,
    min_move_deg: float = 0.30,
    ramp_ratio: float = 0.08,
    zero_endpoint: bool = True,
) -> np.ndarray:
    """Generate a position feed-forward command trajectory for friction/slack compensation."""
    time_s = np.asarray(time_s, dtype=float).reshape(-1)
    q_deg = np.asarray(q_deg, dtype=float)
    qd_deg_s = np.asarray(qd_deg_s, dtype=float)
    comp_pos_deg = _as_5_array(comp_pos_deg, "comp_pos_deg").reshape(1, 5)
    comp_neg_deg = _as_5_array(comp_neg_deg, "comp_neg_deg").reshape(1, 5)

    if q_deg.ndim != 2 or q_deg.shape[1] != 5:
        raise ValueError("q_deg 必须是 N x 5 数组")
    if qd_deg_s.shape != q_deg.shape:
        raise ValueError("qd_deg_s 形状必须与 q_deg 一致")
    if time_s.shape != (q_deg.shape[0],):
        raise ValueError("time_s 长度必须与 q_deg 点数一致")
    if q_deg.shape[0] < 2:
        raise ValueError("轨迹至少需要 2 个点")
    if not np.all(np.isfinite(time_s)):
        raise ValueError("time_s 中存在 NaN 或无穷大")
    if not np.all(np.isfinite(q_deg)):
        raise ValueError("q_deg 中存在 NaN 或无穷大")
    if not np.all(np.isfinite(qd_deg_s)):
        raise ValueError("qd_deg_s 中存在 NaN 或无穷大")
    if np.any(np.diff(time_s) <= 0.0):
        raise ValueError("time_s 必须严格递增")
    if not np.isfinite(min_move_deg) or min_move_deg < 0.0:
        raise ValueError("min_move_deg 必须是非负数")
    if not np.isfinite(ramp_ratio) or ramp_ratio <= 0.0 or ramp_ratio >= 0.5:
        raise ValueError("ramp_ratio 必须在 0 到 0.5 之间")

    total_time = float(time_s[-1])
    if total_time <= 0.0:
        return q_deg.copy()

    tau = (time_s / total_time).reshape(-1, 1)
    window = np.minimum(_smoothstep(tau / ramp_ratio), _smoothstep((1.0 - tau) / ramp_ratio))
    dq_total = q_deg[-1] - q_deg[0]
    direction = np.sign(qd_deg_s)

    near_zero_speed = np.abs(qd_deg_s) < 1e-9
    total_direction = np.sign(dq_total).reshape(1, 5)
    total_direction_full = np.broadcast_to(total_direction, qd_deg_s.shape)
    direction[near_zero_speed] = total_direction_full[near_zero_speed]

    active = (np.abs(dq_total) >= float(min_move_deg)).reshape(1, 5)
    comp_abs = np.where(direction >= 0.0, comp_pos_deg, comp_neg_deg)
    q_cmd_deg = q_deg + direction * comp_abs * window * active

    if zero_endpoint:
        q_cmd_deg[0] = q_deg[0]
        q_cmd_deg[-1] = q_deg[-1]
    return q_cmd_deg


def estimate_quintic_duration(
    q_start: object,
    q_goal: object,
    velocity_limit_deg_s: ArrayLike | float,
    acceleration_limit_deg_s2: ArrayLike | float,
    min_time_s: float = 0.2,
) -> float:
    """Estimate the shortest quintic duration under joint velocity/accel limits."""
    q0 = _joint_to_array(q_start)
    qf = _joint_to_array(q_goal)
    velocity_limit = _as_motion_limit_array(
        velocity_limit_deg_s, "velocity_limit_deg_s"
    )
    acceleration_limit = _as_motion_limit_array(
        acceleration_limit_deg_s2, "acceleration_limit_deg_s2"
    )

    if not np.isfinite(min_time_s) or min_time_s <= 0.0:
        raise ValueError("min_time_s must be positive")

    dq = np.abs(qf - q0)
    velocity_time = _QUINTIC_MAX_VELOCITY_COEFF * dq / velocity_limit
    acceleration_time = np.sqrt(
        _QUINTIC_MAX_ACCELERATION_COEFF * dq / acceleration_limit
    )
    return float(
        max(float(min_time_s), np.max(velocity_time), np.max(acceleration_time))
    )


def plan_quintic_joint_trajectory(
    q_start: object,
    q_goal: object,
    total_time_s: float = 8.0,
    sample_period_s: float = 0.05,
    friction_comp_enable: bool = False,
    comp_pos_deg: ArrayLike | None = None,
    comp_neg_deg: ArrayLike | None = None,
    min_move_deg: float = 0.30,
    ramp_ratio: float = 0.08,
) -> JointTrajectory:
    """Plan a zero-velocity, zero-acceleration quintic joint trajectory.

    Units:
        q_start, q_goal: deg
        total_time_s, sample_period_s: s
        qd_deg_s: deg/s
        qdd_deg_s2: deg/s^2
    """
    q0 = _joint_to_array(q_start)
    qf = _joint_to_array(q_goal)

    if not np.isfinite(total_time_s) or total_time_s <= 0.0:
        raise ValueError("total_time_s 必须大于 0")
    if not np.isfinite(sample_period_s) or sample_period_s <= 0.0:
        raise ValueError("sample_period_s 必须大于 0")

    total_time_s = float(total_time_s)
    sample_period_s = float(sample_period_s)

    interval_count = max(1, int(np.ceil(total_time_s / sample_period_s)))
    time_s = np.linspace(0.0, total_time_s, interval_count + 1)
    tau = (time_s / total_time_s).reshape(-1, 1)

    h = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    hd = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / total_time_s
    hdd = (
        60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
    ) / (total_time_s**2)

    dq = qf - q0
    q_deg = q0 + h * dq
    qd_deg_s = hd * dq
    qdd_deg_s2 = hdd * dq

    # Make endpoint boundary conditions exact despite floating-point roundoff.
    q_deg[0] = q0
    q_deg[-1] = qf
    qd_deg_s[0] = 0.0
    qd_deg_s[-1] = 0.0
    qdd_deg_s2[0] = 0.0
    qdd_deg_s2[-1] = 0.0

    q_cmd_deg = None
    compensation_info = {
        "enabled": bool(friction_comp_enable),
        "type": "position_feedforward",
        "min_move_deg": float(min_move_deg),
        "ramp_ratio": float(ramp_ratio),
        "comp_pos_deg": None,
        "comp_neg_deg": None,
    }
    if friction_comp_enable:
        if comp_pos_deg is None:
            comp_pos_deg = [0.05, 0.05, 0.05, 0.08, 0.03]
        if comp_neg_deg is None:
            comp_neg_deg = [0.05, 0.05, 0.05, 0.08, 0.03]
        comp_pos_array = _as_5_array(comp_pos_deg, "comp_pos_deg")
        comp_neg_array = _as_5_array(comp_neg_deg, "comp_neg_deg")
        q_cmd_deg = make_friction_compensated_q_cmd(
            time_s=time_s,
            q_deg=q_deg,
            qd_deg_s=qd_deg_s,
            comp_pos_deg=comp_pos_array,
            comp_neg_deg=comp_neg_array,
            min_move_deg=min_move_deg,
            ramp_ratio=ramp_ratio,
            zero_endpoint=True,
        )
        compensation_info["comp_pos_deg"] = comp_pos_array.copy()
        compensation_info["comp_neg_deg"] = comp_neg_array.copy()

    return JointTrajectory(
        time_s=time_s,
        q_deg=q_deg,
        qd_deg_s=qd_deg_s,
        qdd_deg_s2=qdd_deg_s2,
        q_cmd_deg=q_cmd_deg,
        compensation_enabled=bool(friction_comp_enable),
        compensation_info=compensation_info,
    )


def plan_quintic_joint_trajectory_by_limits(
    q_start: object,
    q_goal: object,
    velocity_limit_deg_s: ArrayLike | float,
    acceleration_limit_deg_s2: ArrayLike | float,
    sample_period_s: float = 0.05,
    min_time_s: float = 0.2,
    friction_comp_enable: bool = False,
    comp_pos_deg: ArrayLike | None = None,
    comp_neg_deg: ArrayLike | None = None,
    min_move_deg: float = 0.30,
    ramp_ratio: float = 0.08,
) -> JointTrajectory:
    """Plan a quintic joint trajectory from velocity/acceleration limits."""
    total_time_s = estimate_quintic_duration(
        q_start,
        q_goal,
        velocity_limit_deg_s,
        acceleration_limit_deg_s2,
        min_time_s=min_time_s,
    )
    return plan_quintic_joint_trajectory(
        q_start=q_start,
        q_goal=q_goal,
        total_time_s=total_time_s,
        sample_period_s=sample_period_s,
        friction_comp_enable=friction_comp_enable,
        comp_pos_deg=comp_pos_deg,
        comp_neg_deg=comp_neg_deg,
        min_move_deg=min_move_deg,
        ramp_ratio=ramp_ratio,
    )


def _optional_float(value: object, name: str) -> float | None:
    if value is None:
        return None
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _mapping_first(mapping: dict, *keys: str) -> object | None:
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def _cartesian_pose_to_ik_target(
    target_pose: object,
) -> tuple[np.ndarray, float | None, float | None]:
    """Parse ``[x, y, z, yaw, j5]`` or an equivalent mapping."""
    if isinstance(target_pose, dict):
        position_value = _mapping_first(
            target_pose, "position_mm", "target_mm", "target_tcp_mm", "tcp_mm"
        )
        if position_value is None:
            try:
                position_value = [
                    target_pose["x"],
                    target_pose["y"],
                    target_pose["z"],
                ]
            except KeyError as exc:
                raise ValueError(
                    "target_pose dict must contain position_mm or x/y/z"
                ) from exc

        position = np.asarray(position_value, dtype=float).reshape(-1)
        yaw = _optional_float(
            _mapping_first(target_pose, "yaw_deg", "yaw", "target_yaw_deg"),
            "target_yaw_deg",
        )
        j5 = _optional_float(
            _mapping_first(target_pose, "j5_deg", "j5", "target_j5_deg"),
            "target_j5_deg",
        )
    elif hasattr(target_pose, "position_mm"):
        position = np.asarray(target_pose.position_mm, dtype=float).reshape(-1)
        yaw = _optional_float(getattr(target_pose, "yaw_deg", None), "yaw_deg")
        j5 = _optional_float(getattr(target_pose, "j5_deg", None), "j5_deg")
    else:
        array = np.asarray(target_pose, dtype=float).reshape(-1)
        if array.size not in (3, 4, 5):
            raise ValueError(
                "target_pose must be [x, y, z], [x, y, z, yaw], "
                "or [x, y, z, yaw, j5]"
            )
        position = array[:3]
        yaw = None if array.size < 4 else _optional_float(array[3], "yaw_deg")
        j5 = None if array.size < 5 else _optional_float(array[4], "j5_deg")

    if position.size != 3:
        raise ValueError("target Cartesian position must contain x, y, z")
    if not np.all(np.isfinite(position)):
        raise ValueError("target Cartesian position contains NaN or infinity")
    return position, yaw, j5


def plan_cartesian_point_to_point_trajectory(
    q_start: object,
    target_pose: object,
    velocity_limit_deg_s: ArrayLike | float,
    acceleration_limit_deg_s2: ArrayLike | float,
    *,
    q_reference: object | None = None,
    ik_options: IKOptions | None = None,
    sample_period_s: float = 0.05,
    min_time_s: float = 0.2,
    friction_comp_enable: bool = False,
    comp_pos_deg: ArrayLike | None = None,
    comp_neg_deg: ArrayLike | None = None,
    min_move_deg: float = 0.30,
    ramp_ratio: float = 0.08,
) -> CartesianTrajectory:
    """Plan a point-to-point Cartesian move through IK then joint interpolation."""
    q0 = _joint_to_array(q_start)
    target_position, target_yaw, target_j5 = _cartesian_pose_to_ik_target(
        target_pose
    )
    reference = q0 if q_reference is None else _joint_to_array(q_reference)

    ik_result = inverse_kinematics(
        target_position,
        q_seed=q0,
        target_yaw_deg=target_yaw,
        target_j5_deg=target_j5,
        q_reference=reference,
        options=ik_options,
    )
    if not ik_result.success:
        raise ValueError(f"Cartesian inverse kinematics failed: {ik_result.message}")

    joint_trajectory = plan_quintic_joint_trajectory_by_limits(
        q_start=q0,
        q_goal=ik_result.q_deg,
        velocity_limit_deg_s=velocity_limit_deg_s,
        acceleration_limit_deg_s2=acceleration_limit_deg_s2,
        sample_period_s=sample_period_s,
        min_time_s=min_time_s,
        friction_comp_enable=friction_comp_enable,
        comp_pos_deg=comp_pos_deg,
        comp_neg_deg=comp_neg_deg,
        min_move_deg=min_move_deg,
        ramp_ratio=ramp_ratio,
    )

    return CartesianTrajectory(
        joint_trajectory=joint_trajectory,
        ik_result=ik_result,
        target_position_mm=target_position.copy(),
        target_yaw_deg=target_yaw,
        target_j5_deg=ik_result.target_j5_deg,
    )


__all__ = [
    "ArrayLike",
    "CartesianTrajectory",
    "JointTrajectory",
    "estimate_quintic_duration",
    "make_friction_compensated_q_cmd",
    "plan_cartesian_point_to_point_trajectory",
    "plan_quintic_joint_trajectory",
    "plan_quintic_joint_trajectory_by_limits",
]
