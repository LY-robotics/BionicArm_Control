"""Forward and inverse kinematics for the 5-DOF tendon-driven arm.

Task-space convention
---------------------
The public Cartesian pose is ``[x, y, z, yaw, j5]``:

* ``x/y/z`` describe the gripper grasp center in millimetres.
* ``yaw`` describes the horizontal direction of the gripper approach axis.
* ``j5`` rolls the fingers about that approach axis.

The shoulder base frame is ``+X`` down, ``+Y`` forward and ``+Z`` right.
Consequently yaw is zero when the gripper points forward, positive when it
turns right and negative when it turns left.  J5 does not change TCP position
or yaw because the 145 mm gripper offset lies on the J5 rotation axis.

Joint inputs and orientation values use degrees.
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
GRIPPER_TCP_DISTANCE_MM = 145.0
TCP_OFFSET_WRIST_MM = np.array(
    [0.0, 0.0, GRIPPER_TCP_DISTANCE_MM],
    dtype=float,
)
TCP_ROTATION_WRIST = np.eye(3, dtype=float)

# Compatibility name retained for earlier geometry callers.
GRIPPER_OFFSET_WRIST_MM = TCP_OFFSET_WRIST_MM

# Limits converted from the supplied mechanical-limit table.
JOINT_MIN_DEG = np.array([-147.0, -146.4, -166.0, -10.0, -311.0], dtype=float)
JOINT_MAX_DEG = np.array([40.7, 144.0, 160.0, 140.0, 45.0], dtype=float)

# Modified-DH constants.
MDH_ALPHA_DEG = np.full(5, -90.0, dtype=float)
MDH_A_MM = np.zeros(5, dtype=float)
MDH_D_MM = np.array(
    [0.0, 0.0, UPPER_ARM_MM, 0.0, FOREARM_MM],
    dtype=float,
)

# Below this horizontal projection, yaw becomes physically ill-conditioned.
YAW_SINGULARITY_PROJECTION = 1e-8


def as_vector(values: ArrayLike, size: int, name: str) -> np.ndarray:
    """Return ``values`` as a finite one-dimensional vector."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} must contain {size} values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def wrap_angle_deg(angle_deg: float) -> float:
    """Wrap an angle to ``[-180, 180)`` degrees."""
    return float((float(angle_deg) + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class JointAngle:
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.j1, self.j2, self.j3, self.j4, self.j5],
            dtype=float,
        )

    @classmethod
    def from_array(cls, q_deg: ArrayLike) -> "JointAngle":
        q = as_vector(q_deg, 5, "q_deg")
        return cls(*(float(value) for value in q))


@dataclass(frozen=True)
class ArmPose:
    """Pose of the gripper grasp center, retaining the wrist pose."""

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
    def approach_axis(self) -> np.ndarray:
        """Gripper approach axis: TCP-local +Z from wrist to grasp center."""
        return self.rotation[:, 2].copy()

    @property
    def finger_axis(self) -> np.ndarray:
        """Reference left-right finger axis: TCP-local +X."""
        return self.rotation[:, 0].copy()

    @property
    def gripper_axis(self) -> np.ndarray:
        return self.approach_axis

    @property
    def tool_axis(self) -> np.ndarray:
        return self.approach_axis

    @property
    def yaw_horizontal_projection(self) -> float:
        axis = self.approach_axis
        return float(np.hypot(axis[1], axis[2]))

    @property
    def yaw_is_defined(self) -> bool:
        return self.yaw_horizontal_projection > YAW_SINGULARITY_PROJECTION

    @property
    def yaw_deg(self) -> float:
        """Horizontal yaw: zero forward, positive right, negative left."""
        return gripper_yaw_deg(self.rotation)


@dataclass(frozen=True)
class IKOptions:
    position_tolerance_mm: float = 1e-3
    yaw_tolerance_deg: float = 1e-3
    yaw_weight_mm_per_rad: float = 100.0
    max_iterations: int = 500
    damping: float = 1e-2
    max_step_deg: float = 5.0
    nullspace_gain: float = 0.15
    jacobian_step_rad: float = 1e-5
    stagnation_iterations: int = 30
    yaw_projection_min: float = 1e-5

    def __post_init__(self) -> None:
        if self.position_tolerance_mm <= 0.0:
            raise ValueError("position_tolerance_mm must be positive")
        if self.yaw_tolerance_deg <= 0.0:
            raise ValueError("yaw_tolerance_deg must be positive")
        if self.yaw_weight_mm_per_rad <= 0.0:
            raise ValueError("yaw_weight_mm_per_rad must be positive")
        if self.max_iterations <= 0 or self.stagnation_iterations <= 0:
            raise ValueError("Iteration counts must be positive")
        if self.damping <= 0.0 or self.max_step_deg <= 0.0:
            raise ValueError("damping and max_step_deg must be positive")
        if self.nullspace_gain < 0.0 or self.jacobian_step_rad <= 0.0:
            raise ValueError("Invalid null-space or Jacobian option")
        if self.yaw_projection_min <= YAW_SINGULARITY_PROJECTION:
            raise ValueError(
                "yaw_projection_min must exceed YAW_SINGULARITY_PROJECTION"
            )


@dataclass(frozen=True)
class IKResult:
    success: bool
    q_deg: np.ndarray
    position_mm: np.ndarray
    error_mm: np.ndarray
    error_norm_mm: float
    yaw_deg: float
    yaw_error_deg: float | None
    target_yaw_deg: float | None
    target_j5_deg: float
    yaw_defined: bool
    yaw_horizontal_projection: float
    iterations: int
    message: str

    @property
    def joint_angle(self) -> JointAngle:
        return JointAngle.from_array(self.q_deg)


@dataclass(frozen=True)
class PoseRecommendationOptions:
    """Nearest-yaw search settings; J5 remains fixed."""

    yaw_min_deg: float = -180.0
    yaw_max_deg: float = 180.0
    yaw_step_deg: float = 1.0

    def __post_init__(self) -> None:
        _validate_yaw_search(
            self.yaw_min_deg,
            self.yaw_max_deg,
            self.yaw_step_deg,
        )


@dataclass(frozen=True)
class PoseRecommendationResult:
    requested_result: IKResult
    recommended_result: IKResult | None
    requested_yaw_deg: float
    requested_j5_deg: float
    recommended_yaw_deg: float | None
    recommended_j5_deg: float | None
    angular_distance_deg: float | None
    used_recommendation: bool
    failure_reason: str | None

    @property
    def success(self) -> bool:
        return bool(
            self.recommended_result is not None
            and self.recommended_result.success
        )

    @property
    def result(self) -> IKResult:
        return self.recommended_result or self.requested_result


@dataclass(frozen=True)
class IKRecommendConfig:
    """Search grid used when a requested yaw has no strict IK solution."""

    yaw_min_deg: float = -180.0
    yaw_max_deg: float = 180.0
    yaw_step_deg: float = 1.0

    def __post_init__(self) -> None:
        _validate_yaw_search(
            self.yaw_min_deg,
            self.yaw_max_deg,
            self.yaw_step_deg,
        )


@dataclass(frozen=True)
class IKRecommendResult:
    success: bool
    ik_result: IKResult
    requested_yaw_deg: float
    requested_j5_deg: float
    recommended_yaw_deg: float
    recommended_j5_deg: float
    changed_yaw: bool
    changed_j5: bool
    message: str


class YawSingularityError(RuntimeError):
    """Raised when yaw is requested while the approach axis is vertical."""


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
    """Convert physical joint angles to modified-DH theta angles."""
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


def gripper_yaw_deg(rotation: np.ndarray) -> float:
    """Return horizontal gripper yaw in the shoulder base frame.

    The gripper approach axis is TCP-local +Z.  Its projection in the base
    Y-Z plane gives ``yaw = atan2(approach_z, approach_y)``.  Base +Z points
    right, so this directly implements positive-right and negative-left yaw.
    """
    rotation_array = np.asarray(rotation, dtype=float)
    if rotation_array.shape != (3, 3):
        raise ValueError("rotation must be a 3x3 matrix")
    if not np.all(np.isfinite(rotation_array)):
        raise ValueError("rotation must contain finite values")
    axis = rotation_array[:, 2]
    return wrap_angle_deg(np.rad2deg(np.arctan2(axis[2], axis[1])))


def forward_kinematics_legacy(
    q: JointAngle | ArrayLike,
    *,
    check_limits: bool = True,
) -> ArmPose:
    """Calculate the original-MDH gripper-TCP pose."""
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
    wrist_to_tcp[:3, :3] = TCP_ROTATION_WRIST
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
    """J1 correction rotation identified from measured J1/J2 coupling."""
    angle_rad = np.deg2rad(-float(j1_deg))
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array(
        [[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]],
        dtype=float,
    )


def rotation_z_for_j2(j2_deg: float) -> np.ndarray:
    """Planar J2 rotation in the J1=0 reference plane."""
    angle_rad = np.deg2rad(float(j2_deg))
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def axis_order_correction_matrix(j1_deg: float, j2_deg: float) -> np.ndarray:
    """Return the measured J1/J2 axis-order correction matrix."""
    return (
        rotation_z_for_j2(j2_deg)
        @ rotation_y_for_j1(j1_deg)
        @ rotation_z_for_j2(-j2_deg)
    )


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
    """Calculate the corrected gripper-TCP pose from physical joints."""
    q_deg = joint_array(q)
    if check_limits:
        validate_joints(q_deg)

    q_plane = q_deg.copy()
    q_plane[0] = 0.0
    plane_pose = forward_kinematics_legacy(q_plane, check_limits=False)
    correction = axis_order_correction_matrix(
        float(q_deg[0]),
        float(q_deg[1]),
    )
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


def _validate_yaw(target_yaw_deg: float | None) -> float | None:
    if target_yaw_deg is None:
        return None
    yaw = float(target_yaw_deg)
    if not np.isfinite(yaw):
        raise ValueError("target_yaw_deg must be finite")
    return wrap_angle_deg(yaw)


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


def _check_pose_yaw_valid(pose: ArmPose, projection_min: float) -> None:
    if pose.yaw_horizontal_projection <= projection_min:
        raise YawSingularityError(
            "Gripper approach axis is too close to vertical; yaw is undefined"
        )


def task_jacobian(
    q: JointAngle | ArrayLike,
    *,
    include_yaw: bool,
    yaw_weight_mm_per_rad: float,
    step_rad: float = 1e-5,
    yaw_projection_min: float = 1e-5,
) -> np.ndarray:
    """Numerical Jacobian of ``[TCP x, y, z, weighted yaw]`` for J1-J4."""
    q_deg = validate_joints(q)
    if step_rad <= 0.0:
        raise ValueError("step_rad must be positive")

    rows = 4 if include_yaw else 3
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
        if include_yaw:
            _check_pose_yaw_valid(pose_minus, yaw_projection_min)
            _check_pose_yaw_valid(pose_plus, yaw_projection_min)
            yaw_delta_rad = np.deg2rad(
                wrap_angle_deg(pose_plus.yaw_deg - pose_minus.yaw_deg)
            )
            jacobian[3, index] = (
                yaw_weight_mm_per_rad * yaw_delta_rad / delta_rad
            )
    return jacobian


def position_jacobian(
    q: JointAngle | ArrayLike,
    step_rad: float = 1e-5,
) -> np.ndarray:
    """Numerical grasp-center position Jacobian in mm/rad."""
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
        p_minus = forward_position(q_minus, check_limits=False)
        p_plus = forward_position(q_plus, check_limits=False)
        jacobian[:, index] = (p_plus - p_minus) / delta_rad
    return jacobian


def _make_ik_result(
    success: bool,
    q: np.ndarray,
    target_mm: np.ndarray,
    target_yaw_deg: float | None,
    target_j5_deg: float,
    iterations: int,
    message: str,
) -> IKResult:
    pose = forward_kinematics(q, check_limits=False)
    error_mm = target_mm - pose.position_mm
    yaw_error_deg = (
        None
        if target_yaw_deg is None
        else wrap_angle_deg(target_yaw_deg - pose.yaw_deg)
    )
    return IKResult(
        success=success,
        q_deg=q.copy(),
        position_mm=pose.position_mm.copy(),
        error_mm=error_mm,
        error_norm_mm=float(np.linalg.norm(error_mm)),
        yaw_deg=pose.yaw_deg,
        yaw_error_deg=yaw_error_deg,
        target_yaw_deg=target_yaw_deg,
        target_j5_deg=target_j5_deg,
        yaw_defined=pose.yaw_is_defined,
        yaw_horizontal_projection=pose.yaw_horizontal_projection,
        iterations=iterations,
        message=message,
    )


def _solve_one_seed(
    target_mm: np.ndarray,
    target_yaw_deg: float | None,
    target_j5_deg: float,
    q_seed_deg: np.ndarray,
    q_reference_deg: np.ndarray,
    options: IKOptions,
) -> IKResult:
    q = clip_joints(q_seed_deg)
    q_reference = clip_joints(q_reference_deg)
    q[4] = target_j5_deg
    q_reference[4] = target_j5_deg
    include_yaw = target_yaw_deg is not None
    best_task_error = np.inf
    stagnant = 0
    iteration = 0

    for iteration in range(1, options.max_iterations + 1):
        q[4] = target_j5_deg
        pose = forward_kinematics(q, check_limits=False)
        position_error = target_mm - pose.position_mm
        position_error_norm = float(np.linalg.norm(position_error))

        if include_yaw and (
            pose.yaw_horizontal_projection <= options.yaw_projection_min
        ):
            return _make_ik_result(
                False,
                q,
                target_mm,
                target_yaw_deg,
                target_j5_deg,
                iteration,
                "Yaw is undefined because the approach axis is near vertical",
            )

        yaw_error_deg = (
            0.0
            if target_yaw_deg is None
            else wrap_angle_deg(target_yaw_deg - pose.yaw_deg)
        )
        position_ok = position_error_norm <= options.position_tolerance_mm
        yaw_ok = (
            not include_yaw
            or abs(yaw_error_deg) <= options.yaw_tolerance_deg
        )
        if position_ok and yaw_ok:
            task_name = "TCP position" if not include_yaw else "TCP position+yaw"
            return _make_ik_result(
                True,
                q,
                target_mm,
                target_yaw_deg,
                target_j5_deg,
                iteration,
                f"{task_name} inverse kinematics converged",
            )

        error_parts = [position_error]
        if include_yaw:
            error_parts.append(
                np.array(
                    [
                        options.yaw_weight_mm_per_rad
                        * np.deg2rad(yaw_error_deg)
                    ],
                    dtype=float,
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

        try:
            jacobian = task_jacobian(
                q,
                include_yaw=include_yaw,
                yaw_weight_mm_per_rad=options.yaw_weight_mm_per_rad,
                step_rad=options.jacobian_step_rad,
                yaw_projection_min=options.yaw_projection_min,
            )
        except YawSingularityError:
            break

        regularized = (
            jacobian @ jacobian.T
            + options.damping**2 * np.eye(jacobian.shape[0], dtype=float)
        )
        try:
            jacobian_pinv = jacobian.T @ np.linalg.solve(
                regularized,
                np.eye(jacobian.shape[0], dtype=float),
            )
        except np.linalg.LinAlgError:
            jacobian_pinv = np.linalg.pinv(jacobian)

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

    mode = "TCP position" if not include_yaw else "TCP position+yaw"
    return _make_ik_result(
        False,
        q,
        target_mm,
        target_yaw_deg,
        target_j5_deg,
        min(iteration, options.max_iterations),
        f"{mode} inverse kinematics did not converge",
    )


def inverse_kinematics(
    target_mm: ArrayLike,
    q_seed: JointAngle | ArrayLike,
    *,
    target_yaw_deg: float | None = None,
    target_j5_deg: float | None = None,
    q_reference: JointAngle | ArrayLike | None = None,
    options: IKOptions | None = None,
    use_fallback_seeds: bool = True,
) -> IKResult:
    """Solve grasp-center position and optional horizontal yaw.

    J1-J4 solve ``x/y/z/yaw``.  J5 is a fixed roll command and is not used as
    an extra variable by the solver.
    """
    target = as_vector(target_mm, 3, "target_mm")
    seed = validate_joints(q_seed)
    reference = seed.copy() if q_reference is None else validate_joints(q_reference)
    target_yaw = _validate_yaw(target_yaw_deg)
    target_j5 = _validate_j5(target_j5_deg, seed[4])
    solve_options = options or IKOptions()

    primary = _solve_one_seed(
        target,
        target_yaw,
        target_j5,
        seed,
        reference,
        solve_options,
    )
    if primary.success or not use_fallback_seeds:
        return primary

    fallback_seeds = [
        reference,
        np.array([0.0, 0.0, 0.0, 90.0, target_j5]),
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
            target_yaw,
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


def _validate_yaw_search(
    yaw_min_deg: float,
    yaw_max_deg: float,
    yaw_step_deg: float,
) -> None:
    values = np.asarray(
        [yaw_min_deg, yaw_max_deg, yaw_step_deg],
        dtype=float,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("yaw search values must be finite")
    if yaw_min_deg >= yaw_max_deg:
        raise ValueError("yaw_min_deg must be smaller than yaw_max_deg")
    if yaw_step_deg <= 0.0:
        raise ValueError("yaw_step_deg must be positive")


def _yaw_candidates_nearest(
    target_yaw_deg: float,
    yaw_min_deg: float,
    yaw_max_deg: float,
    yaw_step_deg: float,
) -> list[float]:
    """Return the bounded yaw grid ordered by circular angular distance."""
    _validate_yaw_search(yaw_min_deg, yaw_max_deg, yaw_step_deg)
    count = int(np.floor((yaw_max_deg - yaw_min_deg) / yaw_step_deg))
    values = [yaw_min_deg + index * yaw_step_deg for index in range(count + 1)]
    if values[-1] < yaw_max_deg - 1e-12:
        values.append(yaw_max_deg)

    unique: list[float] = []
    seen: set[float] = set()
    for value in values:
        wrapped = wrap_angle_deg(value)
        key = round(wrapped, 10)
        if key not in seen:
            seen.add(key)
            unique.append(wrapped)
    return sorted(
        unique,
        key=lambda yaw: (
            abs(wrap_angle_deg(yaw - target_yaw_deg)),
            abs(yaw),
        ),
    )


def inverse_kinematics_with_recommendation(
    target_mm: ArrayLike,
    q_seed: JointAngle | ArrayLike,
    *,
    target_yaw_deg: float,
    target_j5_deg: float,
    q_reference: JointAngle | ArrayLike | None = None,
    options: IKOptions | None = None,
    recommendation_options: PoseRecommendationOptions | None = None,
) -> PoseRecommendationResult:
    """Solve requested yaw or recommend the nearest feasible yaw."""
    target = as_vector(target_mm, 3, "target_mm")
    seed = validate_joints(q_seed)
    reference = seed.copy() if q_reference is None else validate_joints(q_reference)
    requested_yaw = _validate_yaw(target_yaw_deg)
    assert requested_yaw is not None
    requested_j5 = _validate_j5(target_j5_deg, seed[4])

    requested = inverse_kinematics(
        target,
        seed,
        target_yaw_deg=requested_yaw,
        target_j5_deg=requested_j5,
        q_reference=reference,
        options=options,
    )
    if requested.success:
        return PoseRecommendationResult(
            requested_result=requested,
            recommended_result=requested,
            requested_yaw_deg=requested_yaw,
            requested_j5_deg=requested_j5,
            recommended_yaw_deg=requested_yaw,
            recommended_j5_deg=requested_j5,
            angular_distance_deg=0.0,
            used_recommendation=False,
            failure_reason=None,
        )

    search = recommendation_options or PoseRecommendationOptions()
    for candidate_yaw in _yaw_candidates_nearest(
        requested_yaw,
        search.yaw_min_deg,
        search.yaw_max_deg,
        search.yaw_step_deg,
    ):
        if abs(wrap_angle_deg(candidate_yaw - requested_yaw)) <= 1e-12:
            continue
        candidate = inverse_kinematics(
            target,
            requested.q_deg,
            target_yaw_deg=candidate_yaw,
            target_j5_deg=requested_j5,
            q_reference=reference,
            options=options,
        )
        if candidate.success:
            distance = abs(wrap_angle_deg(candidate_yaw - requested_yaw))
            return PoseRecommendationResult(
                requested_result=requested,
                recommended_result=candidate,
                requested_yaw_deg=requested_yaw,
                requested_j5_deg=requested_j5,
                recommended_yaw_deg=candidate_yaw,
                recommended_j5_deg=requested_j5,
                angular_distance_deg=distance,
                used_recommendation=True,
                failure_reason=(
                    "目标位置和航向角未收敛；保持 J5 不变时，"
                    "已找到最近的可行航向角。"
                ),
            )

    return PoseRecommendationResult(
        requested_result=requested,
        recommended_result=None,
        requested_yaw_deg=requested_yaw,
        requested_j5_deg=requested_j5,
        recommended_yaw_deg=None,
        recommended_j5_deg=None,
        angular_distance_deg=None,
        used_recommendation=False,
        failure_reason=(
            "航向角搜索范围内没有可行解。J5 仅控制夹爪滚转，"
            "调整 J5 不能改善 TCP 位置和航向角的可解性。"
        ),
    )


def _make_recommend_result(
    ik_result: IKResult,
    requested_yaw_deg: float,
    requested_j5_deg: float,
    recommended_yaw_deg: float,
    message: str,
) -> IKRecommendResult:
    return IKRecommendResult(
        success=bool(ik_result.success),
        ik_result=ik_result,
        requested_yaw_deg=float(requested_yaw_deg),
        requested_j5_deg=float(requested_j5_deg),
        recommended_yaw_deg=float(recommended_yaw_deg),
        recommended_j5_deg=float(requested_j5_deg),
        changed_yaw=(
            abs(wrap_angle_deg(recommended_yaw_deg - requested_yaw_deg)) > 1e-9
        ),
        changed_j5=False,
        message=message,
    )


def recommend_feasible_yaw(
    target_tcp_mm: ArrayLike,
    target_yaw_deg: float,
    target_j5_deg: float,
    q_seed: JointAngle | ArrayLike,
    *,
    q_reference: JointAngle | ArrayLike | None = None,
    config: IKRecommendConfig | None = None,
) -> IKRecommendResult:
    """Recommend the nearest feasible yaw while keeping TCP and J5 fixed."""
    target = as_vector(target_tcp_mm, 3, "target_tcp_mm")
    seed = validate_joints(q_seed)
    reference = seed.copy() if q_reference is None else validate_joints(q_reference)
    requested_yaw = _validate_yaw(target_yaw_deg)
    assert requested_yaw is not None
    requested_j5 = _validate_j5(target_j5_deg, seed[4])
    search = config or IKRecommendConfig()

    original = inverse_kinematics(
        target,
        seed,
        target_yaw_deg=requested_yaw,
        target_j5_deg=requested_j5,
        q_reference=reference,
    )
    if original.success:
        return _make_recommend_result(
            original,
            requested_yaw,
            requested_j5,
            requested_yaw,
            "Requested yaw and J5 are feasible",
        )

    for candidate_yaw in _yaw_candidates_nearest(
        requested_yaw,
        search.yaw_min_deg,
        search.yaw_max_deg,
        search.yaw_step_deg,
    ):
        if abs(wrap_angle_deg(candidate_yaw - requested_yaw)) <= 1e-12:
            continue
        candidate = inverse_kinematics(
            target,
            original.q_deg,
            target_yaw_deg=candidate_yaw,
            target_j5_deg=requested_j5,
            q_reference=reference,
        )
        if candidate.success:
            return _make_recommend_result(
                candidate,
                requested_yaw,
                requested_j5,
                candidate_yaw,
                "Requested pose was infeasible; nearest feasible yaw recommended",
            )

    return _make_recommend_result(
        original,
        requested_yaw,
        requested_j5,
        requested_yaw,
        (
            "No feasible yaw was found in the search range. J5 only controls "
            "gripper roll and cannot improve position/yaw feasibility."
        ),
    )


# Compatibility names used by the original motion-control scripts.
KIS = forward_kinematics
KIS_inverse = inverse_kinematics


__all__ = [
    "ArrayLike",
    "ArmPose",
    "BASE_OFFSET_MM",
    "FOREARM_MM",
    "GRIPPER_OFFSET_WRIST_MM",
    "GRIPPER_TCP_DISTANCE_MM",
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
    "TCP_ROTATION_WRIST",
    "UPPER_ARM_MM",
    "YAW_SINGULARITY_PROJECTION",
    "YawSingularityError",
    "actual_to_mdh_theta",
    "as_vector",
    "axis_order_correction_matrix",
    "clip_joints",
    "forward_kinematics",
    "forward_kinematics_legacy",
    "forward_position",
    "forward_wrist_position",
    "gripper_yaw_deg",
    "inverse_kinematics",
    "inverse_kinematics_with_recommendation",
    "joint_array",
    "joints_within_limits",
    "modified_dh_matrix",
    "position_jacobian",
    "recommend_feasible_yaw",
    "rotation_y_for_j1",
    "rotation_z_for_j2",
    "task_jacobian",
    "validate_joints",
    "wrap_angle_deg",
]
