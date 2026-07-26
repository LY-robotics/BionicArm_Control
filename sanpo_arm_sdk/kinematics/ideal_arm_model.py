"""Ideal link model and coordinate frames derived from the active kinematics."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from .kinematic_5dof import (
    BASE_OFFSET_MM,
    MDH_A_MM,
    MDH_ALPHA_DEG,
    MDH_D_MM,
    TCP_OFFSET_WRIST_MM,
    TCP_ROTATION_X_DEG,
    ArmPose,
    actual_to_mdh_theta,
    axis_order_correction_matrix,
    forward_kinematics,
    joint_array,
    modified_dh_matrix,
    rotation_x_matrix,
    validate_joints,
)


ArrayLike = Iterable[float] | np.ndarray


@dataclass(frozen=True)
class CoordinateFrame:
    """One right-handed coordinate frame expressed in the shoulder base frame."""

    name: str
    transform: np.ndarray

    @property
    def origin_mm(self) -> np.ndarray:
        return self.transform[:3, 3].copy()

    @property
    def rotation(self) -> np.ndarray:
        return self.transform[:3, :3].copy()

    @property
    def x_axis(self) -> np.ndarray:
        return self.transform[:3, 0].copy()

    @property
    def y_axis(self) -> np.ndarray:
        return self.transform[:3, 1].copy()

    @property
    def z_axis(self) -> np.ndarray:
        return self.transform[:3, 2].copy()


@dataclass(frozen=True)
class IdealArmModel:
    """The ideal arm geometry for one logical joint vector."""

    q_deg: np.ndarray
    pose: ArmPose
    frames: tuple[CoordinateFrame, ...]
    link_points_mm: np.ndarray

    def frame(self, name: str) -> CoordinateFrame:
        key = str(name).strip().lower()
        for frame in self.frames:
            if frame.name.lower() == key:
                return frame
        raise KeyError(f"Unknown frame {name!r}")

    @property
    def base_frame(self) -> CoordinateFrame:
        return self.frame("Base")

    @property
    def joint_frames(self) -> tuple[CoordinateFrame, ...]:
        return tuple(self.frame(f"J{index}") for index in range(1, 6))

    @property
    def tcp_frame(self) -> CoordinateFrame:
        return self.frame("TCP")

    @property
    def tcp_position_mm(self) -> np.ndarray:
        return self.tcp_frame.origin_mm


@dataclass(frozen=True)
class PositionComparison:
    """Target/theory/feedback TCP positions and their Euclidean differences."""

    target_mm: np.ndarray
    theoretical_mm: np.ndarray
    feedback_mm: np.ndarray | None = None

    @property
    def target_to_theoretical_mm(self) -> float:
        return float(np.linalg.norm(self.theoretical_mm - self.target_mm))

    @property
    def target_to_feedback_mm(self) -> float | None:
        if self.feedback_mm is None:
            return None
        return float(np.linalg.norm(self.feedback_mm - self.target_mm))

    @property
    def theoretical_to_feedback_mm(self) -> float | None:
        if self.feedback_mm is None:
            return None
        return float(np.linalg.norm(self.feedback_mm - self.theoretical_mm))


def _correct_transform(transform: np.ndarray, correction: np.ndarray) -> np.ndarray:
    corrected = np.asarray(transform, dtype=float).copy()
    corrected[:3, :3] = correction @ transform[:3, :3]
    corrected[:3, 3] = correction @ transform[:3, 3]
    return corrected


def build_ideal_arm_model(
    q_deg: ArrayLike,
    *,
    check_limits: bool = True,
) -> IdealArmModel:
    """Build Base, J1..J5 and TCP frames from the active corrected FK chain.

    The Base frame origin is the shoulder reference used by the motion model.
    ``BASE_OFFSET_MM`` is retained between Base and the intersecting shoulder
    joint frames because it is part of the existing, hardware-tested FK.
    """

    q = joint_array(q_deg)
    if check_limits:
        validate_joints(q)

    q_plane = q.copy()
    q_plane[0] = 0.0
    theta_deg = actual_to_mdh_theta(q_plane)
    correction = axis_order_correction_matrix(float(q[0]), float(q[1]))

    base_transform = np.eye(4, dtype=float)
    plane_transform = np.eye(4, dtype=float)
    plane_transform[:3, 3] = BASE_OFFSET_MM

    joint_transforms: list[np.ndarray] = []
    for alpha_deg, a_mm, d_mm, theta_i_deg in zip(
        MDH_ALPHA_DEG,
        MDH_A_MM,
        MDH_D_MM,
        theta_deg,
    ):
        plane_transform = plane_transform @ modified_dh_matrix(
            np.deg2rad(alpha_deg),
            float(a_mm),
            float(d_mm),
            np.deg2rad(theta_i_deg),
        )
        joint_transforms.append(_correct_transform(plane_transform, correction))

    wrist_to_tcp = np.eye(4, dtype=float)
    wrist_to_tcp[:3, :3] = rotation_x_matrix(np.deg2rad(TCP_ROTATION_X_DEG))
    wrist_to_tcp[:3, 3] = TCP_OFFSET_WRIST_MM
    tcp_transform = joint_transforms[-1] @ wrist_to_tcp

    frames = (
        CoordinateFrame("Base", base_transform),
        *(
            CoordinateFrame(f"J{index}", transform)
            for index, transform in enumerate(joint_transforms, start=1)
        ),
        CoordinateFrame("TCP", tcp_transform),
    )

    # J1/J2 intersect at the shoulder and J3/J4 intersect at the elbow.
    # Repeated frame origins are omitted from the visible link polyline.
    link_points = np.vstack(
        [
            base_transform[:3, 3],
            joint_transforms[0][:3, 3],
            joint_transforms[2][:3, 3],
            joint_transforms[4][:3, 3],
            tcp_transform[:3, 3],
        ]
    )
    pose = forward_kinematics(q, check_limits=check_limits)
    return IdealArmModel(
        q_deg=q.copy(),
        pose=pose,
        frames=tuple(frames),
        link_points_mm=link_points,
    )


def compare_tcp_positions(
    target_mm: ArrayLike,
    theoretical_model: IdealArmModel,
    feedback_model: IdealArmModel | None = None,
) -> PositionComparison:
    target = np.asarray(target_mm, dtype=float).reshape(-1)
    if target.size != 3 or not np.all(np.isfinite(target)):
        raise ValueError("target_mm must contain finite x, y and z")
    return PositionComparison(
        target_mm=target.copy(),
        theoretical_mm=theoretical_model.tcp_position_mm,
        feedback_mm=(
            None if feedback_model is None else feedback_model.tcp_position_mm
        ),
    )


def export_ideal_model_csv(
    path: str | Path,
    *,
    target_mm: ArrayLike,
    theoretical_model: IdealArmModel,
    feedback_model: IdealArmModel | None = None,
) -> Path:
    """Export target, link frames, joints and comparison errors to UTF-8 CSV."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    comparison = compare_tcp_positions(
        target_mm,
        theoretical_model,
        feedback_model,
    )
    with destination.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "record",
                "name",
                "x_mm",
                "y_mm",
                "z_mm",
                "x_axis_x",
                "x_axis_y",
                "x_axis_z",
                "y_axis_x",
                "y_axis_y",
                "y_axis_z",
                "z_axis_x",
                "z_axis_y",
                "z_axis_z",
                "value",
                "unit",
            ]
        )

        def position_row(record: str, name: str, position: np.ndarray) -> None:
            writer.writerow([record, name, *position, *([""] * 9), "", "mm"])

        position_row("position", "target_tcp", comparison.target_mm)
        position_row("position", "theoretical_tcp", comparison.theoretical_mm)
        if comparison.feedback_mm is not None:
            position_row("position", "feedback_tcp", comparison.feedback_mm)
        for frame in theoretical_model.frames:
            writer.writerow(
                [
                    "frame",
                    frame.name,
                    *frame.origin_mm,
                    *frame.x_axis,
                    *frame.y_axis,
                    *frame.z_axis,
                    "",
                    "",
                ]
            )
        for prefix, model in (
            ("theoretical", theoretical_model),
            ("feedback", feedback_model),
        ):
            if model is None:
                continue
            for index, value in enumerate(model.q_deg, start=1):
                writer.writerow(
                    [
                        "joint",
                        f"{prefix}_J{index}",
                        *([""] * 12),
                        float(value),
                        "deg",
                    ]
                )
        errors = (
            ("target_to_theoretical", comparison.target_to_theoretical_mm),
            ("target_to_feedback", comparison.target_to_feedback_mm),
            (
                "theoretical_to_feedback",
                comparison.theoretical_to_feedback_mm,
            ),
        )
        for name, value in errors:
            if value is not None:
                writer.writerow(
                    [
                        "error",
                        name,
                        *([""] * 12),
                        float(value),
                        "mm",
                    ]
                )
    return destination


__all__ = [
    "CoordinateFrame",
    "IdealArmModel",
    "PositionComparison",
    "build_ideal_arm_model",
    "compare_tcp_positions",
    "export_ideal_model_csv",
]
