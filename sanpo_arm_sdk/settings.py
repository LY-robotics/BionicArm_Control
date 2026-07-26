"""User-facing defaults for motion planning and telemetry.

Keep frequently tuned values here.  Hardware identity, motor direction and
joint limits remain in ``config.py`` because they are calibration data rather
than runtime motion preferences.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionDefaults:
    """Defaults shared by single-arm and dual-arm commands."""

    speed_deg_s: float = 20.0
    acceleration_deg_s2: float = 40.0
    sample_period_s: float = 0.05
    minimum_duration_s: float = 0.5
    final_tolerance_deg: float = 1.0


@dataclass(frozen=True)
class CartesianLineDefaults:
    """Defaults for sampled TCP straight-line interpolation."""

    sample_period_s: float = 0.05
    minimum_duration_s: float = 1.0
    position_tolerance_mm: float = 0.1
    max_branch_jump_deg: float = 25.0
    velocity_margin: float = 1.10
    acceleration_margin: float = 1.10


@dataclass(frozen=True)
class RecommendationDefaults:
    """Search grid used when the requested pitch/J5 pose has no IK solution."""

    pitch_min_deg: float = -90.0
    pitch_max_deg: float = 90.0
    pitch_step_deg: float = 1.0
    j5_step_deg: float = 2.0


@dataclass(frozen=True)
class TelemetryDefaults:
    """Sampling and in-memory history defaults for live feedback."""

    sample_period_s: float = 0.10
    max_samples_per_joint: int = 10_000


MOTION_DEFAULTS = MotionDefaults()
LINE_DEFAULTS = CartesianLineDefaults()
RECOMMENDATION_DEFAULTS = RecommendationDefaults()
TELEMETRY_DEFAULTS = TelemetryDefaults()


__all__ = [
    "CartesianLineDefaults",
    "LINE_DEFAULTS",
    "MOTION_DEFAULTS",
    "MotionDefaults",
    "RECOMMENDATION_DEFAULTS",
    "RecommendationDefaults",
    "TELEMETRY_DEFAULTS",
    "TelemetryDefaults",
]
