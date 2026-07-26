"""Sampled TCP straight-line interpolation with continuous IK branch selection."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .guiji_quintic import JointTrajectory, estimate_quintic_duration
from .kinematic_5dof import (
    IKOptions,
    IKResult,
    JOINT_MAX_DEG,
    JOINT_MIN_DEG,
    forward_kinematics,
    inverse_kinematics,
    validate_joints,
)


ArrayLike = Iterable[float] | np.ndarray


@dataclass(frozen=True)
class CartesianLinePlan:
    """A Cartesian line represented by continuously selected joint samples."""

    joint_trajectory: JointTrajectory
    start_position_mm: np.ndarray
    target_position_mm: np.ndarray
    desired_position_mm: np.ndarray
    actual_position_mm: np.ndarray
    nominal_pitch_deg: np.ndarray
    actual_pitch_deg: np.ndarray
    desired_j5_deg: np.ndarray
    position_error_mm: np.ndarray
    final_ik_result: IKResult
    requested_duration_s: float | None
    adjusted_duration_s: float

    @property
    def max_position_error_mm(self) -> float:
        return float(np.max(self.position_error_mm))

    @property
    def max_line_deviation_mm(self) -> float:
        start = self.start_position_mm
        line = self.target_position_mm - start
        length = float(np.linalg.norm(line))
        if length <= 1e-12:
            return float(np.max(np.linalg.norm(self.actual_position_mm - start, axis=1)))
        unit = line / length
        relative = self.actual_position_mm - start
        along = relative @ unit
        closest = start + along.reshape(-1, 1) * unit
        return float(np.max(np.linalg.norm(self.actual_position_mm - closest, axis=1)))


def _as_limit_array(value: ArrayLike | float, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 1:
        array = np.full(5, float(array[0]), dtype=float)
    if array.size != 5 or not np.all(np.isfinite(array)) or np.any(array <= 0.0):
        raise ValueError(f"{name} must be one positive value or five positive values")
    return array


def _quintic_blend(normalized_time: np.ndarray) -> np.ndarray:
    tau = np.clip(np.asarray(normalized_time, dtype=float), 0.0, 1.0)
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _finite_difference(
    time_s: np.ndarray,
    q_deg: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    edge_order = 2 if len(time_s) >= 3 else 1
    velocity = np.gradient(q_deg, time_s, axis=0, edge_order=edge_order)
    acceleration = np.gradient(velocity, time_s, axis=0, edge_order=edge_order)
    velocity[[0, -1]] = 0.0
    acceleration[[0, -1]] = 0.0
    return velocity, acceleration


def _candidate_seeds(
    previous_q: np.ndarray,
    predicted_q: np.ndarray,
    target_j5_deg: float,
) -> list[np.ndarray]:
    """Generate a compact local seed set before allowing global IK fallbacks."""

    offsets = (
        (0.0, 0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0, 0.0),
        (-5.0, 0.0, 0.0, 0.0),
        (0.0, 5.0, 0.0, -5.0),
        (0.0, -5.0, 0.0, 5.0),
        (0.0, 0.0, 5.0, -5.0),
        (0.0, 0.0, -5.0, 5.0),
    )
    seeds: list[np.ndarray] = []
    seen: set[tuple[float, ...]] = set()
    for base in (predicted_q, previous_q):
        for offset in offsets:
            seed = np.asarray(base, dtype=float).copy()
            seed[:4] += np.asarray(offset, dtype=float)
            seed = np.clip(seed, JOINT_MIN_DEG, JOINT_MAX_DEG)
            seed[4] = target_j5_deg
            key = tuple(np.round(seed, 3))
            if key not in seen:
                seen.add(key)
                seeds.append(seed)
    return seeds


def _solve_continuous_sample(
    target_position_mm: np.ndarray,
    previous_q: np.ndarray,
    predicted_q: np.ndarray,
    target_j5_deg: float,
    *,
    position_tolerance_mm: float,
    max_branch_jump_deg: float,
    ik_options: IKOptions,
) -> IKResult:
    primary = inverse_kinematics(
        target_position_mm,
        q_seed=previous_q,
        target_pitch_deg=None,
        target_j5_deg=target_j5_deg,
        q_reference=previous_q,
        options=ik_options,
        use_fallback_seeds=False,
    )
    primary_jump = float(np.max(np.abs(primary.q_deg - previous_q)))
    if (
        (primary.success or primary.error_norm_mm <= position_tolerance_mm)
        and primary_jump <= max_branch_jump_deg
    ):
        return primary

    candidates: list[IKResult] = []
    for seed in _candidate_seeds(previous_q, predicted_q, target_j5_deg):
        result = inverse_kinematics(
            target_position_mm,
            q_seed=seed,
            target_pitch_deg=None,
            target_j5_deg=target_j5_deg,
            q_reference=previous_q,
            options=ik_options,
            use_fallback_seeds=False,
        )
        if result.success or result.error_norm_mm <= position_tolerance_mm:
            candidates.append(result)

    if not candidates:
        fallback = inverse_kinematics(
            target_position_mm,
            q_seed=previous_q,
            target_pitch_deg=None,
            target_j5_deg=target_j5_deg,
            q_reference=previous_q,
            options=ik_options,
            use_fallback_seeds=True,
        )
        candidates.append(fallback)

    joint_range = np.maximum(JOINT_MAX_DEG - JOINT_MIN_DEG, 1.0)

    def score(result: IKResult) -> tuple[float, float, float]:
        jump = float(np.max(np.abs(result.q_deg - previous_q)))
        jump_penalty = 0.0 if jump <= max_branch_jump_deg else 100.0
        continuity = float(np.linalg.norm((result.q_deg - previous_q) / joint_range))
        prediction = float(np.linalg.norm((result.q_deg - predicted_q) / joint_range))
        return (
            jump_penalty + continuity + 0.25 * prediction,
            result.error_norm_mm,
            jump,
        )

    return min(candidates, key=score)


def _required_duration(
    q_path: np.ndarray,
    velocity_limit_deg_s: np.ndarray,
    acceleration_limit_deg_s2: np.ndarray,
    minimum_duration_s: float,
    velocity_margin: float,
    acceleration_margin: float,
) -> float:
    """Retime a normalized path so sampled velocity/acceleration stay in limits."""

    normalized_time = np.linspace(0.0, 1.0, len(q_path))
    qd_norm, qdd_norm = _finite_difference(normalized_time, q_path)
    peak_velocity_per_unit_time = np.max(np.abs(qd_norm), axis=0)
    peak_acceleration_per_unit_time2 = np.max(np.abs(qdd_norm), axis=0)
    velocity_time = np.max(peak_velocity_per_unit_time / velocity_limit_deg_s)
    acceleration_time = np.sqrt(
        np.max(peak_acceleration_per_unit_time2 / acceleration_limit_deg_s2)
    )
    return float(
        max(
            minimum_duration_s,
            velocity_margin * velocity_time,
            np.sqrt(acceleration_margin) * acceleration_time,
        )
    )


def plan_cartesian_line_trajectory(
    q_start: ArrayLike,
    target_position_mm: ArrayLike,
    *,
    reference_pitch_deg: float | None,
    target_j5_deg: float | None,
    velocity_limit_deg_s: ArrayLike | float,
    acceleration_limit_deg_s2: ArrayLike | float,
    total_time_s: float | None = None,
    sample_period_s: float = 0.05,
    minimum_duration_s: float = 1.0,
    position_tolerance_mm: float = 0.5,
    max_branch_jump_deg: float = 25.0,
    velocity_margin: float = 1.10,
    acceleration_margin: float = 1.10,
) -> CartesianLinePlan:
    """Plan an offline TCP line while allowing pitch to follow a feasible curve.

    TCP position and J5 are hard constraints at each sample.  Pitch is a
    reference curve used for reporting because a five-axis arm cannot in
    general satisfy position, pitch and J5 everywhere along an arbitrary line.
    """

    q0 = validate_joints(q_start)
    target = np.asarray(target_position_mm, dtype=float).reshape(-1)
    if target.size != 3 or not np.all(np.isfinite(target)):
        raise ValueError("target_position_mm must contain finite x, y and z")
    if sample_period_s <= 0.0 or minimum_duration_s <= 0.0:
        raise ValueError("sample periods and durations must be positive")
    if position_tolerance_mm <= 0.0 or max_branch_jump_deg <= 0.0:
        raise ValueError("IK tolerance and branch jump limit must be positive")

    requested_duration = None if total_time_s is None else float(total_time_s)
    if requested_duration is not None and requested_duration <= 0.0:
        raise ValueError("total_time_s must be positive")

    velocity_limit = _as_limit_array(velocity_limit_deg_s, "velocity_limit_deg_s")
    acceleration_limit = _as_limit_array(
        acceleration_limit_deg_s2, "acceleration_limit_deg_s2"
    )
    start_pose = forward_kinematics(q0)
    start_position = start_pose.position_mm
    pitch_target = (
        float(start_pose.pitch_deg)
        if reference_pitch_deg is None
        else float(reference_pitch_deg)
    )
    j5_target = float(q0[4]) if target_j5_deg is None else float(target_j5_deg)
    if not JOINT_MIN_DEG[4] <= j5_target <= JOINT_MAX_DEG[4]:
        raise ValueError("target_j5_deg is outside the kinematic joint limit")

    target_probe = inverse_kinematics(
        target,
        q_seed=q0,
        target_pitch_deg=None,
        target_j5_deg=j5_target,
        q_reference=q0,
    )
    point_to_point_duration = minimum_duration_s
    if target_probe.success:
        point_to_point_duration = 1.2 * estimate_quintic_duration(
            q0,
            target_probe.q_deg,
            velocity_limit,
            acceleration_limit,
            min_time_s=minimum_duration_s,
        )
    planning_duration = max(
        minimum_duration_s,
        requested_duration or 0.0,
        point_to_point_duration,
    )
    distance_mm = float(np.linalg.norm(target - start_position))
    point_count = max(
        3,
        int(np.ceil(planning_duration / sample_period_s)) + 1,
        int(np.ceil(distance_mm / 5.0)) + 1,
    )
    normalized_time = np.linspace(0.0, 1.0, point_count)
    blend = _quintic_blend(normalized_time)
    desired_position = (
        start_position
        + blend.reshape(-1, 1) * (target - start_position).reshape(1, 3)
    )
    nominal_pitch = start_pose.pitch_deg + blend * (
        pitch_target - start_pose.pitch_deg
    )
    desired_j5 = q0[4] + blend * (j5_target - q0[4])

    q_path = np.zeros((point_count, 5), dtype=float)
    q_path[0] = q0
    actual_position = np.zeros((point_count, 3), dtype=float)
    actual_position[0] = start_position
    actual_pitch = np.zeros(point_count, dtype=float)
    actual_pitch[0] = start_pose.pitch_deg
    position_error = np.zeros(point_count, dtype=float)
    final_result: IKResult | None = None
    options = IKOptions(
        position_tolerance_mm=min(position_tolerance_mm, 0.1),
        pitch_tolerance_deg=0.5,
        max_iterations=220,
        stagnation_iterations=30,
        max_step_deg=4.0,
    )

    for index in range(1, point_count):
        previous_q = q_path[index - 1]
        predicted_q = (
            previous_q
            if index < 2
            else previous_q + (previous_q - q_path[index - 2])
        )
        result = _solve_continuous_sample(
            desired_position[index],
            previous_q,
            predicted_q,
            float(desired_j5[index]),
            position_tolerance_mm=position_tolerance_mm,
            max_branch_jump_deg=max_branch_jump_deg,
            ik_options=options,
        )
        jump = float(np.max(np.abs(result.q_deg - previous_q)))
        if result.error_norm_mm > position_tolerance_mm:
            raise RuntimeError(
                f"line IK failed at point {index + 1}/{point_count}: "
                f"position error {result.error_norm_mm:.3f} mm"
            )
        if jump > max_branch_jump_deg:
            raise RuntimeError(
                f"line IK branch jump at point {index + 1}/{point_count}: "
                f"{jump:.3f} deg > {max_branch_jump_deg:.3f} deg"
            )
        q_path[index] = result.q_deg
        actual_position[index] = result.position_mm
        actual_pitch[index] = result.pitch_deg
        position_error[index] = result.error_norm_mm
        final_result = result

    assert final_result is not None
    auto_duration = _required_duration(
        q_path,
        velocity_limit,
        acceleration_limit,
        minimum_duration_s,
        velocity_margin,
        acceleration_margin,
    )
    adjusted_duration = max(auto_duration, requested_duration or 0.0)
    time_s = normalized_time * adjusted_duration
    qd_deg_s, qdd_deg_s2 = _finite_difference(time_s, q_path)
    trajectory = JointTrajectory(
        time_s=time_s,
        q_deg=q_path,
        qd_deg_s=qd_deg_s,
        qdd_deg_s2=qdd_deg_s2,
    )
    return CartesianLinePlan(
        joint_trajectory=trajectory,
        start_position_mm=start_position.copy(),
        target_position_mm=target.copy(),
        desired_position_mm=desired_position,
        actual_position_mm=actual_position,
        nominal_pitch_deg=nominal_pitch,
        actual_pitch_deg=actual_pitch,
        desired_j5_deg=desired_j5,
        position_error_mm=position_error,
        final_ik_result=final_result,
        requested_duration_s=requested_duration,
        adjusted_duration_s=adjusted_duration,
    )


__all__ = ["CartesianLinePlan", "plan_cartesian_line_trajectory"]
