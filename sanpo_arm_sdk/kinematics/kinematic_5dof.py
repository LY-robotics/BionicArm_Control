"""Forward and inverse kinematics for the 5-DOF tendon-driven arm.

This module provides the single stable kinematics entry point for SDK code.

Base coordinate convention:
    +x: vertically downward
    +y: forward
    +z: right

Actual joint convention:
    J1: left positive, right negative
    J2: forward positive, backward negative
    J3: clockwise positive, counterclockwise negative
    J4: elbow flexion positive
    J5: counterclockwise positive, clockwise negative

Lengths and positions use millimetres. Joint inputs use degrees.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


ArrayLike = Iterable[float] | np.ndarray

# Geometry, in mm.
UPPER_ARM_MM = 350.0
FOREARM_MM = 250.0
BASE_OFFSET_MM = np.array([0.0, 0.0, -18.0], dtype=float)
TCP_OFFSET_WRIST_MM = np.array([0.0, -50.9117, 84.9117], dtype=float)
TCP_ROTATION_X_DEG = 45.0

# Compatibility name retained for earlier code.
GRIPPER_OFFSET_WRIST_MM = TCP_OFFSET_WRIST_MM

# Limits converted from the supplied mechanical-limit table.
JOINT_MIN_DEG = np.array([-147.0, -146.4, -166.0, -10.0, -311.0], dtype=float)
JOINT_MAX_DEG = np.array([40.7, 144.0, 160.0, 140.0, 45.0], dtype=float)

# Modified-DH constants.
MDH_ALPHA_DEG = np.full(5, -90.0, dtype=float)
MDH_A_MM = np.zeros(5, dtype=float)
MDH_D_MM = np.array(
    [0.0, 0.0, UPPER_ARM_MM, 0.0, FOREARM_MM], dtype=float
)


def as_vector(values: ArrayLike, size: int, name: str) -> np.ndarray:
    """Return ``values`` as a finite 1-D float vector of ``size``."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} must contain {size} values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


@dataclass(frozen=True)
class JointAngle:
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.j1, self.j2, self.j3, self.j4, self.j5], dtype=float
        )

    @classmethod
    def from_array(cls, q_deg: ArrayLike) -> "JointAngle":
        q = as_vector(q_deg, 5, "q_deg")
        return cls(*(float(value) for value in q))


@dataclass(frozen=True)
class ArmPose:
    """Pose of the terminal TCP, with the wrist pose retained for inspection."""

    position_mm: np.ndarray
    rotation: np.ndarray
    transform: np.ndarray
    mdh_theta_deg: np.ndarray
    wrist_position_mm: np.ndarray
    wrist_rotation: np.ndarray
    wrist_transform: np.ndarray

    @property
    def x(self) -> float:
        return float(self.position_mm[0])

    @property
    def y(self) -> float:
        return float(self.position_mm[1])

    @property
    def z(self) -> float:
        return float(self.position_mm[2])

    @property
    def gripper_axis(self) -> np.ndarray:
        """Compatibility name for the terminal tool-axis unit vector."""
        return self.rotation[:, 2].copy()

    @property
    def tool_axis(self) -> np.ndarray:
        """Unit vector of the final 45-degree terminal segment."""
        return self.rotation[:, 2].copy()

    @property
    def pitch_deg(self) -> float:
        """Terminal elevation: 0 deg horizontal, +90 deg vertically down."""
        return gripper_pitch_deg(self.rotation)


@dataclass(frozen=True)
class IKOptions:
    position_tolerance_mm: float = 1e-3
    pitch_tolerance_deg: float = 1e-3
    pitch_weight_mm_per_rad: float = 100.0
    max_iterations: int = 500
    damping: float = 1e-2
    max_step_deg: float = 5.0
    nullspace_gain: float = 0.15
    jacobian_step_rad: float = 1e-5
    stagnation_iterations: int = 30

    def __post_init__(self) -> None:
        if self.position_tolerance_mm <= 0.0:
            raise ValueError("position_tolerance_mm must be positive")
        if self.pitch_tolerance_deg <= 0.0:
            raise ValueError("pitch_tolerance_deg must be positive")
        if self.pitch_weight_mm_per_rad <= 0.0:
            raise ValueError("pitch_weight_mm_per_rad must be positive")
        if self.max_iterations <= 0 or self.stagnation_iterations <= 0:
            raise ValueError("Iteration counts must be positive")
        if self.damping <= 0.0 or self.max_step_deg <= 0.0:
            raise ValueError("damping and max_step_deg must be positive")
        if self.nullspace_gain < 0.0 or self.jacobian_step_rad <= 0.0:
            raise ValueError("Invalid null-space or Jacobian option")


@dataclass(frozen=True)
class IKResult:
    success: bool
    q_deg: np.ndarray
    position_mm: np.ndarray
    error_mm: np.ndarray
    error_norm_mm: float
    pitch_deg: float
    pitch_error_deg: float | None
    target_pitch_deg: float | None
    target_j5_deg: float
    iterations: int
    message: str

    @property
    def joint_angle(self) -> JointAngle:
        return JointAngle.from_array(self.q_deg)


@dataclass(frozen=True)
class PoseRecommendationOptions:
    """Options for the hierarchical pitch-first pose recommendation."""

    max_iterations: int = 800
    secondary_gain: float = 0.30
    max_step_deg: float = 3.0
    position_tolerance_mm: float = 1e-3
    jacobian_step_rad: float = 1e-5

    def __post_init__(self) -> None:
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.secondary_gain <= 0.0 or self.max_step_deg <= 0.0:
            raise ValueError("secondary_gain and max_step_deg must be positive")
        if self.position_tolerance_mm <= 0.0:
            raise ValueError("position_tolerance_mm must be positive")
        if self.jacobian_step_rad <= 0.0:
            raise ValueError("jacobian_step_rad must be positive")


@dataclass(frozen=True)
class PoseRecommendationResult:
    """Strict IK result plus the nearest feasible pose when strict IK fails."""

    requested_result: IKResult
    recommended_result: IKResult | None
    requested_pitch_deg: float
    requested_j5_deg: float
    recommended_pitch_deg: float | None
    recommended_j5_deg: float | None
    angular_distance_deg: float | None
    used_recommendation: bool
    failure_reason: str | None

    @property
    def success(self) -> bool:
        return (
            self.recommended_result is not None
            and self.recommended_result.success
        )

    @property
    def result(self) -> IKResult:
        """Return the strict result, or the recommended result after fallback."""
        if self.recommended_result is not None:
            return self.recommended_result
        return self.requested_result


@dataclass(frozen=True)
class IKRecommendConfig:
    pitch_min_deg: float = -90.0
    pitch_max_deg: float = 90.0
    pitch_step_deg: float = 1.0
    j5_step_deg: float = 1.0


@dataclass(frozen=True)
class IKRecommendResult:
    success: bool
    ik_result: IKResult
    requested_pitch_deg: float
    requested_j5_deg: float
    recommended_pitch_deg: float
    recommended_j5_deg: float
    changed_pitch: bool
    changed_j5: bool
    message: str


def joint_array(q: JointAngle | ArrayLike) -> np.ndarray:
    if isinstance(q, JointAngle):
        return q.as_array()
    return as_vector(q, 5, "q_deg")


def joints_within_limits(q: JointAngle | ArrayLike, atol: float = 1e-9) -> bool:
    q_deg = joint_array(q)
    return bool(
        np.all(q_deg >= JOINT_MIN_DEG - atol)
        and np.all(q_deg <= JOINT_MAX_DEG + atol)
    )


def validate_joints(q: JointAngle | ArrayLike) -> np.ndarray:
    q_deg = joint_array(q)
    if joints_within_limits(q_deg):
        return q_deg

    messages: list[str] = []
    for index, value in enumerate(q_deg):
        lower = JOINT_MIN_DEG[index]
        upper = JOINT_MAX_DEG[index]
        if value < lower or value > upper:
            messages.append(
                f"J{index + 1}={value:.6f} deg is outside "
                f"[{lower:.6f}, {upper:.6f}] deg"
            )
    raise ValueError("; ".join(messages))


def actual_to_mdh_theta(q: JointAngle | ArrayLike) -> np.ndarray:
    """Convert actual arm angles to modified-DH theta angles, in degrees."""
    j1, j2, j3, j4, j5 = joint_array(q)
    return np.array(
        [j1, -j2 - 90.0, j3, j4 + 180.0, -j5 - 90.0],
        dtype=float,
    )


def modified_dh_matrix(
    alpha_rad: float,
    a_mm: float,
    d_mm: float,
    theta_rad: float,
) -> np.ndarray:
    ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
    ct, st = np.cos(theta_rad), np.sin(theta_rad)
    return np.array(
        [
            [ct, -st, 0.0, a_mm],
            [st * ca, ct * ca, -sa, -d_mm * sa],
            [st * sa, ct * sa, ca, d_mm * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotation_x_matrix(angle_rad: float) -> np.ndarray:
    """Return a 3x3 right-handed rotation matrix about local x."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def gripper_pitch_deg(rotation: np.ndarray) -> float:
    """Return gripper-axis pitch in the base coordinate system."""
    rotation_array = np.asarray(rotation, dtype=float)
    if rotation_array.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")
    axis = rotation_array[:, 2]
    return float(
        np.rad2deg(np.arctan2(axis[0], np.hypot(axis[1], axis[2])))
    )


def forward_kinematics_legacy(
    q: JointAngle | ArrayLike,
    *,
    check_limits: bool = True,
) -> ArmPose:
    """Calculate the original MDH terminal-TCP pose from actual joint angles."""
    q_deg = joint_array(q)
    if check_limits:
        validate_joints(q_deg)

    theta_deg = actual_to_mdh_theta(q_deg)
    wrist_transform = np.eye(4, dtype=float)
    wrist_transform[:3, 3] = BASE_OFFSET_MM

    for alpha_deg, a_mm, d_mm, theta_i_deg in zip(
        MDH_ALPHA_DEG,
        MDH_A_MM,
        MDH_D_MM,
        theta_deg,
    ):
        wrist_transform = wrist_transform @ modified_dh_matrix(
            np.deg2rad(alpha_deg),
            a_mm,
            d_mm,
            np.deg2rad(theta_i_deg),
        )

    wrist_to_tcp = np.eye(4, dtype=float)
    wrist_to_tcp[:3, :3] = rotation_x_matrix(np.deg2rad(TCP_ROTATION_X_DEG))
    wrist_to_tcp[:3, 3] = TCP_OFFSET_WRIST_MM
    tcp_transform = wrist_transform @ wrist_to_tcp

    return ArmPose(
        position_mm=tcp_transform[:3, 3].copy(),
        rotation=tcp_transform[:3, :3].copy(),
        transform=tcp_transform.copy(),
        mdh_theta_deg=theta_deg,
        wrist_position_mm=wrist_transform[:3, 3].copy(),
        wrist_rotation=wrist_transform[:3, :3].copy(),
        wrist_transform=wrist_transform.copy(),
    )


def rotation_y_for_j1(j1_deg: float) -> np.ndarray:
    """J1 correction rotation identified from measured coupled J1/J2 motion."""
    angle_rad = np.deg2rad(-float(j1_deg))
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array(
        [
            [c, 0.0, -s],
            [0.0, 1.0, 0.0],
            [s, 0.0, c],
        ],
        dtype=float,
    )


def rotation_z_for_j2(j2_deg: float) -> np.ndarray:
    """Planar J2 rotation in the J1=0 reference plane."""
    angle_rad = np.deg2rad(float(j2_deg))
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def axis_order_correction_matrix(j1_deg: float, j2_deg: float) -> np.ndarray:
    """Return the measured J1/J2 axis-order correction matrix."""
    rz_j2 = rotation_z_for_j2(j2_deg)
    ry_j1 = rotation_y_for_j1(j1_deg)
    rz_neg_j2 = rotation_z_for_j2(-j2_deg)
    return rz_j2 @ ry_j1 @ rz_neg_j2


def _apply_rotation_to_pose(pose: ArmPose, rotation: np.ndarray) -> ArmPose:
    tcp_transform = pose.transform.copy()
    wrist_transform = pose.wrist_transform.copy()

    tcp_transform[:3, :3] = rotation @ pose.rotation
    tcp_transform[:3, 3] = rotation @ pose.position_mm

    wrist_transform[:3, :3] = rotation @ pose.wrist_rotation
    wrist_transform[:3, 3] = rotation @ pose.wrist_position_mm

    return ArmPose(
        position_mm=tcp_transform[:3, 3].copy(),
        rotation=tcp_transform[:3, :3].copy(),
        transform=tcp_transform.copy(),
        mdh_theta_deg=pose.mdh_theta_deg.copy(),
        wrist_position_mm=wrist_transform[:3, 3].copy(),
        wrist_rotation=wrist_transform[:3, :3].copy(),
        wrist_transform=wrist_transform.copy(),
    )


def forward_kinematics(
    q: JointAngle | ArrayLike,
    *,
    check_limits: bool = True,
) -> ArmPose:
    """Calculate the corrected terminal-TCP pose from actual joint angles."""
    q_deg = joint_array(q)
    if check_limits:
        validate_joints(q_deg)

    j1_deg = float(q_deg[0])
    j2_deg = float(q_deg[1])

    q_plane = q_deg.copy()
    q_plane[0] = 0.0
    plane_pose = forward_kinematics_legacy(q_plane, check_limits=False)

    correction = axis_order_correction_matrix(j1_deg, j2_deg)
    return _apply_rotation_to_pose(plane_pose, correction)


def forward_position(
    q: JointAngle | ArrayLike,
    *,
    check_limits: bool = True,
) -> np.ndarray:
    return forward_kinematics(q, check_limits=check_limits).position_mm


def forward_wrist_position(
    q: JointAngle | ArrayLike,
    *,
    check_limits: bool = True,
) -> np.ndarray:
    return forward_kinematics(q, check_limits=check_limits).wrist_position_mm


def clip_joints(q: JointAngle | ArrayLike) -> np.ndarray:
    return np.clip(joint_array(q), JOINT_MIN_DEG, JOINT_MAX_DEG)


def _validate_pitch(target_pitch_deg: float | None) -> float | None:
    if target_pitch_deg is None:
        return None
    pitch = float(target_pitch_deg)
    if not np.isfinite(pitch):
        raise ValueError("target_pitch_deg must be finite")
    if pitch < -90.0 or pitch > 90.0:
        raise ValueError("target_pitch_deg must be within [-90, 90] deg")
    return pitch


def _validate_j5(target_j5_deg: float | None, seed_j5_deg: float) -> float:
    j5 = seed_j5_deg if target_j5_deg is None else float(target_j5_deg)
    if not np.isfinite(j5):
        raise ValueError("target_j5_deg must be finite")
    if j5 < JOINT_MIN_DEG[4] or j5 > JOINT_MAX_DEG[4]:
        raise ValueError(
            f"target_j5_deg={j5:.6f} is outside "
            f"[{JOINT_MIN_DEG[4]:.6f}, {JOINT_MAX_DEG[4]:.6f}] deg"
        )
    return j5


def task_jacobian(
    q: JointAngle | ArrayLike,
    *,
    include_pitch: bool,
    pitch_weight_mm_per_rad: float,
    step_rad: float = 1e-5,
) -> np.ndarray:
    """Numerical Jacobian of [TCP x, y, z, weighted pitch] for J1-J4."""
    q_deg = validate_joints(q)
    if step_rad <= 0.0:
        raise ValueError("step_rad must be positive")

    rows = 4 if include_pitch else 3
    jacobian = np.zeros((rows, 4), dtype=float)
    step_deg = np.rad2deg(step_rad)

    for index in range(4):
        q_minus = q_deg.copy()
        q_plus = q_deg.copy()
        q_minus[index] = max(q_deg[index] - step_deg, JOINT_MIN_DEG[index])
        q_plus[index] = min(q_deg[index] + step_deg, JOINT_MAX_DEG[index])
        delta_rad = np.deg2rad(q_plus[index] - q_minus[index])
        if delta_rad == 0.0:
            continue

        pose_minus = forward_kinematics(q_minus, check_limits=False)
        pose_plus = forward_kinematics(q_plus, check_limits=False)
        jacobian[:3, index] = (
            pose_plus.position_mm - pose_minus.position_mm
        ) / delta_rad

        if include_pitch:
            pitch_delta_rad = np.deg2rad(
                pose_plus.pitch_deg - pose_minus.pitch_deg
            )
            jacobian[3, index] = (
                pitch_weight_mm_per_rad * pitch_delta_rad / delta_rad
            )

    return jacobian


def position_jacobian(
    q: JointAngle | ArrayLike,
    step_rad: float = 1e-5,
) -> np.ndarray:
    """Numerical TCP-position Jacobian in mm/rad for all five joints."""
    q_deg = validate_joints(q)
    if step_rad <= 0.0:
        raise ValueError("step_rad must be positive")

    jacobian = np.zeros((3, 5), dtype=float)
    step_deg = np.rad2deg(step_rad)

    for index in range(5):
        q_minus = q_deg.copy()
        q_plus = q_deg.copy()
        q_minus[index] = max(q_deg[index] - step_deg, JOINT_MIN_DEG[index])
        q_plus[index] = min(q_deg[index] + step_deg, JOINT_MAX_DEG[index])
        delta_rad = np.deg2rad(q_plus[index] - q_minus[index])
        if delta_rad == 0.0:
            continue

        p_minus = forward_kinematics(q_minus, check_limits=False).position_mm
        p_plus = forward_kinematics(q_plus, check_limits=False).position_mm
        jacobian[:, index] = (p_plus - p_minus) / delta_rad

    return jacobian


def _position_and_pitch_jacobians(
    q: np.ndarray,
    step_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Calculate position and terminal-pitch Jacobians in one FK pass."""
    position_j = np.zeros((3, 5), dtype=float)
    pitch_j = np.zeros(5, dtype=float)
    step_deg = np.rad2deg(step_rad)

    for index in range(5):
        q_minus = q.copy()
        q_plus = q.copy()
        q_minus[index] = max(q[index] - step_deg, JOINT_MIN_DEG[index])
        q_plus[index] = min(q[index] + step_deg, JOINT_MAX_DEG[index])
        delta_rad = np.deg2rad(q_plus[index] - q_minus[index])
        if delta_rad == 0.0:
            continue

        pose_minus = forward_kinematics(q_minus, check_limits=False)
        pose_plus = forward_kinematics(q_plus, check_limits=False)
        position_j[:, index] = (
            pose_plus.position_mm - pose_minus.position_mm
        ) / delta_rad
        pitch_j[index] = np.deg2rad(
            pose_plus.pitch_deg - pose_minus.pitch_deg
        ) / delta_rad

    return position_j, pitch_j


def _make_ik_result(
    success: bool,
    q: np.ndarray,
    target_mm: np.ndarray,
    target_pitch_deg: float | None,
    target_j5_deg: float,
    iterations: int,
    message: str,
) -> IKResult:
    pose = forward_kinematics(q, check_limits=False)
    error_mm = target_mm - pose.position_mm
    pitch_error_deg = (
        None
        if target_pitch_deg is None
        else float(target_pitch_deg - pose.pitch_deg)
    )
    return IKResult(
        success=success,
        q_deg=q.copy(),
        position_mm=pose.position_mm.copy(),
        error_mm=error_mm,
        error_norm_mm=float(np.linalg.norm(error_mm)),
        pitch_deg=pose.pitch_deg,
        pitch_error_deg=pitch_error_deg,
        target_pitch_deg=target_pitch_deg,
        target_j5_deg=target_j5_deg,
        iterations=iterations,
        message=message,
    )


def _solve_one_seed(
    target_mm: np.ndarray,
    target_pitch_deg: float | None,
    target_j5_deg: float,
    q_seed_deg: np.ndarray,
    q_reference_deg: np.ndarray,
    options: IKOptions,
) -> IKResult:
    q = clip_joints(q_seed_deg)
    q_reference = clip_joints(q_reference_deg)
    q[4] = target_j5_deg
    q_reference[4] = target_j5_deg
    include_pitch = target_pitch_deg is not None
    best_task_error = np.inf
    stagnant = 0
    iteration = 0

    for iteration in range(1, options.max_iterations + 1):
        q[4] = target_j5_deg
        pose = forward_kinematics(q, check_limits=False)
        position_error = target_mm - pose.position_mm
        position_error_norm = float(np.linalg.norm(position_error))
        pitch_error_deg = (
            0.0
            if target_pitch_deg is None
            else float(target_pitch_deg - pose.pitch_deg)
        )

        position_ok = position_error_norm <= options.position_tolerance_mm
        pitch_ok = (
            not include_pitch
            or abs(pitch_error_deg) <= options.pitch_tolerance_deg
        )
        if position_ok and pitch_ok:
            task_name = "TCP position" if not include_pitch else "TCP pose"
            return _make_ik_result(
                True,
                q,
                target_mm,
                target_pitch_deg,
                target_j5_deg,
                iteration,
                f"{task_name} inverse kinematics converged",
            )

        error_parts = [position_error]
        if include_pitch:
            error_parts.append(
                np.array(
                    [
                        options.pitch_weight_mm_per_rad
                        * np.deg2rad(pitch_error_deg)
                    ]
                )
            )
        task_error = np.concatenate(error_parts)
        task_error_norm = float(np.linalg.norm(task_error))

        if task_error_norm < best_task_error - 1e-9:
            best_task_error = task_error_norm
            stagnant = 0
        else:
            stagnant += 1
            if stagnant >= options.stagnation_iterations:
                break

        jacobian = task_jacobian(
            q,
            include_pitch=include_pitch,
            pitch_weight_mm_per_rad=options.pitch_weight_mm_per_rad,
            step_rad=options.jacobian_step_rad,
        )
        regularized = (
            jacobian @ jacobian.T
            + options.damping**2 * np.eye(jacobian.shape[0], dtype=float)
        )
        jacobian_pinv = jacobian.T @ np.linalg.solve(
            regularized, np.eye(jacobian.shape[0], dtype=float)
        )
        dq_task_rad = jacobian_pinv @ task_error

        nullspace = np.eye(4, dtype=float) - jacobian_pinv @ jacobian
        reference_delta_rad = np.deg2rad(q_reference[:4] - q[:4])
        dq_rad = dq_task_rad + (
            options.nullspace_gain * nullspace @ reference_delta_rad
        )

        step_norm_deg = float(np.linalg.norm(np.rad2deg(dq_rad)))
        if step_norm_deg > options.max_step_deg:
            dq_rad *= options.max_step_deg / step_norm_deg

        q[:4] += np.rad2deg(dq_rad)
        q = clip_joints(q)
        q[4] = target_j5_deg

    mode = "TCP position" if not include_pitch else "TCP pose"
    return _make_ik_result(
        False,
        q,
        target_mm,
        target_pitch_deg,
        target_j5_deg,
        min(iteration, options.max_iterations),
        f"{mode} inverse kinematics did not converge",
    )


def inverse_kinematics(
    target_mm: ArrayLike,
    q_seed: JointAngle | ArrayLike,
    *,
    target_pitch_deg: float | None = None,
    target_j5_deg: float | None = None,
    q_reference: JointAngle | ArrayLike | None = None,
    options: IKOptions | None = None,
    use_fallback_seeds: bool = True,
) -> IKResult:
    """Solve the offset terminal TCP target.

    ``target_pitch_deg`` is terminal-axis elevation: 0 deg is horizontal and
    +90 deg is vertically down. ``target_j5_deg`` is the physical J5 command.
    Omitting ``target_pitch_deg`` keeps position-only compatibility mode.
    """
    target = as_vector(target_mm, 3, "target_mm")
    seed = validate_joints(q_seed)
    reference = (
        seed.copy()
        if q_reference is None
        else validate_joints(q_reference)
    )
    target_pitch = _validate_pitch(target_pitch_deg)
    target_j5 = _validate_j5(target_j5_deg, seed[4])
    solve_options = options or IKOptions()

    primary = _solve_one_seed(
        target,
        target_pitch,
        target_j5,
        seed,
        reference,
        solve_options,
    )
    if primary.success or not use_fallback_seeds:
        return primary

    fallback_seeds = [
        reference,
        np.array([0.0, 0.0, 0.0, 30.0, target_j5]),
        np.array([-90.0, 30.0, 0.0, 90.0, target_j5]),
        np.array([0.0, 0.0, 90.0, 90.0, target_j5]),
        np.array([30.0, 30.0, 90.0, 90.0, target_j5]),
        np.array([-30.0, -30.0, -90.0, 90.0, target_j5]),
        np.array([40.0, 0.0, 0.0, 90.0, target_j5]),
    ]

    candidates: list[IKResult] = []
    for fallback in fallback_seeds:
        candidate = _solve_one_seed(
            target,
            target_pitch,
            target_j5,
            clip_joints(fallback),
            reference,
            solve_options,
        )
        if candidate.success:
            candidates.append(candidate)

    if not candidates:
        return primary

    joint_ranges = (JOINT_MAX_DEG - JOINT_MIN_DEG)[:4]
    return min(
        candidates,
        key=lambda result: np.linalg.norm(
            (result.q_deg[:4] - reference[:4]) / joint_ranges
        ),
    )


def _nearest_pose_from_seed(
    target_mm: np.ndarray,
    target_pitch_deg: float,
    target_j5_deg: float,
    seed_deg: np.ndarray,
    options: PoseRecommendationOptions,
) -> IKResult:
    """Minimise pitch/J5 deviation while keeping TCP position primary."""
    q = clip_joints(seed_deg)
    target_secondary_rad = np.deg2rad(
        np.array([target_pitch_deg, target_j5_deg], dtype=float)
    )

    for _ in range(1, options.max_iterations + 1):
        pose = forward_kinematics(q, check_limits=False)
        position_error = target_mm - pose.position_mm
        position_j, pitch_j = _position_and_pitch_jacobians(
            q, options.jacobian_step_rad
        )

        regularized = (
            position_j @ position_j.T + 1e-4 * np.eye(3, dtype=float)
        )
        position_pinv = position_j.T @ np.linalg.solve(
            regularized, np.eye(3, dtype=float)
        )
        nullspace = np.eye(5, dtype=float) - position_pinv @ position_j

        current_secondary_rad = np.array(
            [np.deg2rad(pose.pitch_deg), np.deg2rad(q[4])],
            dtype=float,
        )
        secondary_error = target_secondary_rad - current_secondary_rad
        secondary_j = np.vstack(
            [pitch_j, np.array([0.0, 0.0, 0.0, 0.0, 1.0])]
        )

        dq_rad = position_pinv @ position_error
        dq_rad += (
            options.secondary_gain
            * nullspace
            @ secondary_j.T
            @ secondary_error
        )

        step_norm_deg = float(np.linalg.norm(np.rad2deg(dq_rad)))
        if step_norm_deg > options.max_step_deg:
            dq_rad *= options.max_step_deg / step_norm_deg

        q = clip_joints(q + np.rad2deg(dq_rad))

    pose = forward_kinematics(q, check_limits=False)
    position_error = target_mm - pose.position_mm
    error_norm = float(np.linalg.norm(position_error))
    success = error_norm <= options.position_tolerance_mm
    return _make_ik_result(
        success,
        q,
        target_mm,
        pose.pitch_deg,
        float(q[4]),
        options.max_iterations,
        (
            "Closest feasible pitch/J5 search converged"
            if success
            else "Closest feasible pitch/J5 search did not reach the position"
        ),
    )


def _integer_pitch_candidates(target_pitch_deg: float) -> list[float]:
    """Return every integer pitch in [-90, 90], nearest target first."""
    return sorted(
        (float(pitch) for pitch in range(-90, 91)),
        key=lambda pitch: (abs(pitch - target_pitch_deg), pitch),
    )


def _first_feasible_integer_pitch(
    target_mm: np.ndarray,
    target_pitch_deg: float,
    fixed_j5_deg: float,
    q_seed_deg: np.ndarray,
    q_reference_deg: np.ndarray,
    options: IKOptions | None,
    *,
    skip_requested_pitch: bool,
) -> IKResult | None:
    """Find the nearest feasible integer pitch while keeping J5 fixed."""
    seed = q_seed_deg.copy()
    seed[4] = fixed_j5_deg
    reference = q_reference_deg.copy()
    reference[4] = fixed_j5_deg

    for integer_pitch in _integer_pitch_candidates(target_pitch_deg):
        if skip_requested_pitch and np.isclose(
            integer_pitch, target_pitch_deg, atol=1e-12
        ):
            continue

        candidate = inverse_kinematics(
            target_mm,
            seed,
            target_pitch_deg=integer_pitch,
            target_j5_deg=fixed_j5_deg,
            q_reference=reference,
            options=options,
        )
        if candidate.success:
            return candidate

    return None


def inverse_kinematics_with_recommendation(
    target_mm: ArrayLike,
    q_seed: JointAngle | ArrayLike,
    *,
    target_pitch_deg: float,
    target_j5_deg: float,
    q_reference: JointAngle | ArrayLike | None = None,
    options: IKOptions | None = None,
    recommendation_options: PoseRecommendationOptions | None = None,
) -> PoseRecommendationResult:
    """Solve the requested pose, or recommend the closest feasible pose."""
    target = as_vector(target_mm, 3, "target_mm")
    seed = validate_joints(q_seed)
    reference = (
        seed.copy()
        if q_reference is None
        else validate_joints(q_reference)
    )
    target_pitch = _validate_pitch(target_pitch_deg)
    assert target_pitch is not None
    target_j5 = _validate_j5(target_j5_deg, seed[4])

    requested = inverse_kinematics(
        target,
        seed,
        target_pitch_deg=target_pitch,
        target_j5_deg=target_j5,
        q_reference=reference,
        options=options,
    )
    if requested.success:
        return PoseRecommendationResult(
            requested_result=requested,
            recommended_result=requested,
            requested_pitch_deg=target_pitch,
            requested_j5_deg=target_j5,
            recommended_pitch_deg=target_pitch,
            recommended_j5_deg=target_j5,
            angular_distance_deg=0.0,
            used_recommendation=False,
            failure_reason=None,
        )

    fixed_j5_result = _first_feasible_integer_pitch(
        target,
        target_pitch,
        target_j5,
        requested.q_deg,
        reference,
        options,
        skip_requested_pitch=True,
    )
    if fixed_j5_result is not None:
        recommended_pitch = int(round(fixed_j5_result.target_pitch_deg))
        return PoseRecommendationResult(
            requested_result=requested,
            recommended_result=fixed_j5_result,
            requested_pitch_deg=target_pitch,
            requested_j5_deg=target_j5,
            recommended_pitch_deg=recommended_pitch,
            recommended_j5_deg=target_j5,
            angular_distance_deg=abs(recommended_pitch - target_pitch),
            used_recommendation=True,
            failure_reason=(
                "Requested TCP pose did not converge; keeping J5 fixed, "
                "a feasible solution was found by changing only pitch."
            ),
        )

    search_options = recommendation_options or PoseRecommendationOptions()
    fallback_seeds = [
        seed,
        reference,
        requested.q_deg,
        np.array([0.0, 0.0, 0.0, 30.0, target_j5]),
        np.array([-90.0, 30.0, 0.0, 90.0, target_j5]),
        np.array([0.0, 0.0, 90.0, 90.0, target_j5]),
        np.array([-30.0, -30.0, -90.0, 90.0, target_j5]),
    ]

    candidates = [
        _nearest_pose_from_seed(
            target,
            target_pitch,
            target_j5,
            clip_joints(candidate_seed),
            search_options,
        )
        for candidate_seed in fallback_seeds
    ]
    feasible = [candidate for candidate in candidates if candidate.success]

    verified_integer_candidates: list[IKResult] = []
    for continuous_candidate in feasible:
        verified = _first_feasible_integer_pitch(
            target,
            target_pitch,
            float(continuous_candidate.q_deg[4]),
            continuous_candidate.q_deg,
            continuous_candidate.q_deg,
            options,
            skip_requested_pitch=False,
        )
        if verified is not None:
            verified_integer_candidates.append(verified)

    if not verified_integer_candidates:
        return PoseRecommendationResult(
            requested_result=requested,
            recommended_result=None,
            requested_pitch_deg=target_pitch,
            requested_j5_deg=target_j5,
            recommended_pitch_deg=None,
            recommended_j5_deg=None,
            angular_distance_deg=None,
            used_recommendation=False,
            failure_reason=(
                "Requested TCP pose did not converge. No verified integer "
                "pitch solution was found after fixed-J5 and variable-J5 search."
            ),
        )

    def angular_distance(candidate: IKResult) -> float:
        return float(
            np.hypot(
                float(candidate.target_pitch_deg) - target_pitch,
                candidate.target_j5_deg - target_j5,
            )
        )

    verified = min(verified_integer_candidates, key=angular_distance)
    recommended_pitch = int(round(verified.target_pitch_deg))
    recommended_j5 = float(verified.target_j5_deg)

    distance_to_limit = np.minimum(
        verified.q_deg - JOINT_MIN_DEG,
        JOINT_MAX_DEG - verified.q_deg,
    )
    if float(np.min(distance_to_limit)) <= 0.05:
        reason = (
            "No fixed-J5 integer pitch solution was found. A feasible integer "
            "pitch was found after changing J5, but at least one joint is near "
            "its limit."
        )
    else:
        reason = (
            "No fixed-J5 integer pitch solution was found. J5 was adjusted and "
            "the recommended integer pitch/J5 pair passed strict IK validation."
        )

    return PoseRecommendationResult(
        requested_result=requested,
        recommended_result=verified,
        requested_pitch_deg=target_pitch,
        requested_j5_deg=target_j5,
        recommended_pitch_deg=recommended_pitch,
        recommended_j5_deg=recommended_j5,
        angular_distance_deg=angular_distance(verified),
        used_recommendation=True,
        failure_reason=reason,
    )


def _validate_scalar(value: float, name: str) -> float:
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _grid_candidates_nearest(
    center: float,
    lower: float,
    upper: float,
    step: float,
) -> list[float]:
    """Return bounded grid values, ordered from nearest to farthest."""
    center = _validate_scalar(center, "center")
    lower = _validate_scalar(lower, "lower")
    upper = _validate_scalar(upper, "upper")
    step = _validate_scalar(step, "step")

    if lower > upper:
        raise ValueError("lower must not be greater than upper")
    if step <= 0.0:
        raise ValueError("step must be positive")

    start = int(np.ceil((lower - center) / step))
    stop = int(np.floor((upper - center) / step))
    values = [center + k * step for k in range(start, stop + 1)]
    values = [float(np.clip(value, lower, upper)) for value in values]

    unique_values = []
    seen = set()
    for value in values:
        key = round(value, 10)
        if key not in seen:
            unique_values.append(value)
            seen.add(key)

    return sorted(
        unique_values,
        key=lambda value: (abs(value - center), abs(value)),
    )


def _make_recommend_result(
    ik_result: IKResult,
    requested_pitch_deg: float,
    requested_j5_deg: float,
    recommended_pitch_deg: float,
    recommended_j5_deg: float,
    message: str,
) -> IKRecommendResult:
    return IKRecommendResult(
        success=bool(ik_result.success),
        ik_result=ik_result,
        requested_pitch_deg=float(requested_pitch_deg),
        requested_j5_deg=float(requested_j5_deg),
        recommended_pitch_deg=float(recommended_pitch_deg),
        recommended_j5_deg=float(recommended_j5_deg),
        changed_pitch=not np.isclose(
            requested_pitch_deg, recommended_pitch_deg, atol=1e-9
        ),
        changed_j5=not np.isclose(
            requested_j5_deg, recommended_j5_deg, atol=1e-9
        ),
        message=message,
    )


def recommend_feasible_pitch_j5(
    target_tcp_mm: ArrayLike,
    target_pitch_deg: float,
    target_j5_deg: float,
    q_seed: JointAngle | ArrayLike,
    *,
    q_reference: JointAngle | ArrayLike | None = None,
    config: IKRecommendConfig | None = None,
) -> IKRecommendResult:
    """Recommend feasible pitch/J5 for a TCP target.

    The search policy is:
    1. Try the requested TCP + pitch + J5 target.
    2. If it fails, keep J5 fixed and search the nearest bounded pitch.
    3. If that fails, search the nearest bounded J5 and pitch pair.
    """
    target = as_vector(target_tcp_mm, 3, "target_tcp_mm")
    requested_pitch = _validate_scalar(target_pitch_deg, "target_pitch_deg")
    requested_j5 = _validate_scalar(target_j5_deg, "target_j5_deg")
    search_config = config or IKRecommendConfig()
    reference = q_seed if q_reference is None else q_reference

    original = inverse_kinematics(
        target,
        q_seed=q_seed,
        target_pitch_deg=requested_pitch,
        target_j5_deg=requested_j5,
        q_reference=reference,
    )
    if original.success:
        return _make_recommend_result(
            original,
            requested_pitch,
            requested_j5,
            requested_pitch,
            requested_j5,
            "Requested pitch and J5 are feasible",
        )

    pitch_candidates = _grid_candidates_nearest(
        requested_pitch,
        search_config.pitch_min_deg,
        search_config.pitch_max_deg,
        search_config.pitch_step_deg,
    )

    for pitch in pitch_candidates:
        candidate = inverse_kinematics(
            target,
            q_seed=q_seed,
            target_pitch_deg=pitch,
            target_j5_deg=requested_j5,
            q_reference=reference,
        )
        if candidate.success:
            return _make_recommend_result(
                candidate,
                requested_pitch,
                requested_j5,
                pitch,
                requested_j5,
                (
                    "Requested target was infeasible; J5 was kept fixed and "
                    "the nearest feasible pitch was recommended"
                ),
            )

    j5_candidates = _grid_candidates_nearest(
        requested_j5,
        float(JOINT_MIN_DEG[4]),
        float(JOINT_MAX_DEG[4]),
        search_config.j5_step_deg,
    )

    for j5 in j5_candidates:
        for pitch in pitch_candidates:
            candidate = inverse_kinematics(
                target,
                q_seed=q_seed,
                target_pitch_deg=pitch,
                target_j5_deg=j5,
                q_reference=reference,
            )
            if candidate.success:
                return _make_recommend_result(
                    candidate,
                    requested_pitch,
                    requested_j5,
                    pitch,
                    j5,
                    (
                        "Fixed-J5 pitch search was infeasible; the nearest "
                        "bounded J5 and pitch pair was recommended"
                    ),
                )

    return _make_recommend_result(
        original,
        requested_pitch,
        requested_j5,
        requested_pitch,
        requested_j5,
        "No feasible recommendation was found within the pitch and J5 limits",
    )


# Compatibility names retained for existing calling code.
KIS = forward_kinematics
KIS_inverse = inverse_kinematics


__all__ = [
    "ArrayLike",
    "ArmPose",
    "BASE_OFFSET_MM",
    "FOREARM_MM",
    "GRIPPER_OFFSET_WRIST_MM",
    "IKOptions",
    "IKRecommendConfig",
    "IKRecommendResult",
    "IKResult",
    "JOINT_MAX_DEG",
    "JOINT_MIN_DEG",
    "JointAngle",
    "KIS",
    "KIS_inverse",
    "MDH_A_MM",
    "MDH_ALPHA_DEG",
    "MDH_D_MM",
    "PoseRecommendationOptions",
    "PoseRecommendationResult",
    "TCP_OFFSET_WRIST_MM",
    "TCP_ROTATION_X_DEG",
    "UPPER_ARM_MM",
    "actual_to_mdh_theta",
    "as_vector",
    "axis_order_correction_matrix",
    "clip_joints",
    "forward_kinematics",
    "forward_kinematics_legacy",
    "forward_position",
    "forward_wrist_position",
    "gripper_pitch_deg",
    "inverse_kinematics",
    "inverse_kinematics_with_recommendation",
    "joint_array",
    "joints_within_limits",
    "modified_dh_matrix",
    "position_jacobian",
    "recommend_feasible_pitch_j5",
    "rotation_x_matrix",
    "rotation_y_for_j1",
    "rotation_z_for_j2",
    "task_jacobian",
    "validate_joints",
]
