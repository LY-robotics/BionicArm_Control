"""Mechanical and motor configuration for the two 5-DOF arms."""

from dataclasses import replace
from typing import Dict, List, Sequence

from .protocol.can_motor_arm_lib import JointConfig

JOINT_KEYS = ("j1", "j2", "j3", "j4", "j5")

_JOINT_NAMES = (
    "J1_Motion_Original_J2_Shoulder_Yaw",
    "J2_Motion_Original_J1_Shoulder_Pitch",
    "J3_Shoulder_Roll",
    "J4_Elbow_Pitch",
    "J5_Wrist_Roll",
)
_RATIOS = (3.0, 3.0, 4.0, 4.2, 1.0)
_LIMITS = (
    (-170.0, 120.0),
    (-110.0, 110.0),
    (-180.0, 180.0),
    (-90.0, 150.0),
    (-120.0, 120.0),
)


def _build_profile(ids: Sequence[int], direction: float) -> tuple[JointConfig, ...]:
    return tuple(
        JointConfig(
            key=key,
            motor_id=int(ids[index]),
            name=_JOINT_NAMES[index],
            ratio=_RATIOS[index],
            direction=direction,
            min_deg=_LIMITS[index][0],
            max_deg=_LIMITS[index][1],
            default_speed_rpm=5.0,
            default_accel_rpm_s=10.0,
        )
        for index, key in enumerate(JOINT_KEYS)
    )


# The motion-control model labels the first two axes in the opposite order
# from the original protocol example:
#   motion J1 -> original J2
#   motion J2 -> original J1
# Keep the swap here at the hardware boundary so kinematics, commands and
# feedback all use one consistent motion-control joint order.
RIGHT_ARM_JOINTS = _build_profile((34, 35, 31, 32, 33), direction=1.0)
LEFT_ARM_JOINTS = _build_profile((1, 55, 15, 18, 23), direction=-1.0)

ARM_PROFILES: Dict[str, tuple[JointConfig, ...]] = {
    "right": RIGHT_ARM_JOINTS,
    "left": LEFT_ARM_JOINTS,
}


def get_arm_joint_configs(profile: str) -> List[JointConfig]:
    """Return an independent mutable copy of one arm profile."""
    key = str(profile).strip().lower()
    if key not in ARM_PROFILES:
        raise ValueError(f"Unknown arm profile {profile!r}; choose left or right")
    return [replace(cfg) for cfg in ARM_PROFILES[key]]
