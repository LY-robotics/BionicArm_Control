"""Regression tests for the public XYZ + yaw + J5 task definition."""

import unittest

import numpy as np

from sanpo_arm_sdk.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    plan_cartesian_line_trajectory,
    recommend_feasible_yaw,
)
from sanpo_arm_sdk.kinematics.kinematic_5dof import gripper_yaw_deg


def rotation_with_horizontal_yaw(yaw_deg: float) -> np.ndarray:
    """Build a right-handed frame whose local +Z has the requested yaw."""
    angle = np.deg2rad(yaw_deg)
    approach = np.array([0.0, np.cos(angle), np.sin(angle)])
    finger = np.array([1.0, 0.0, 0.0])
    local_y = np.cross(approach, finger)
    return np.column_stack((finger, local_y, approach))


class YawKinematicsTests(unittest.TestCase):
    def test_yaw_sign_is_positive_right_and_negative_left(self) -> None:
        # Base +Z points right, so a positive approach-axis Z component is right.
        self.assertAlmostEqual(
            gripper_yaw_deg(rotation_with_horizontal_yaw(30.0)),
            30.0,
            places=10,
        )
        self.assertAlmostEqual(
            gripper_yaw_deg(rotation_with_horizontal_yaw(-25.0)),
            -25.0,
            places=10,
        )

    def test_reference_pose_points_forward_with_zero_yaw(self) -> None:
        pose = forward_kinematics([0.0, 0.0, 0.0, 90.0, 0.0])
        np.testing.assert_allclose(
            pose.approach_axis,
            [0.0, 1.0, 0.0],
            atol=1e-12,
        )
        self.assertAlmostEqual(pose.yaw_deg, 0.0, places=10)

    def test_j5_roll_does_not_change_tcp_or_yaw(self) -> None:
        q_a = [-20.0, 20.0, 30.0, 45.0, 0.0]
        q_b = [-20.0, 20.0, 30.0, 45.0, -100.0]
        pose_a = forward_kinematics(q_a)
        pose_b = forward_kinematics(q_b)
        np.testing.assert_allclose(
            pose_a.position_mm,
            pose_b.position_mm,
            atol=1e-10,
        )
        self.assertAlmostEqual(pose_a.yaw_deg, pose_b.yaw_deg, places=10)
        self.assertFalse(np.allclose(pose_a.finger_axis, pose_b.finger_axis))

    def test_position_yaw_ik_round_trip(self) -> None:
        q_expected = np.array([-20.0, 20.0, 30.0, 45.0, -15.0])
        pose = forward_kinematics(q_expected)
        result = inverse_kinematics(
            pose.position_mm,
            q_seed=[-18.0, 18.0, 28.0, 43.0, -15.0],
            target_yaw_deg=pose.yaw_deg,
            target_j5_deg=q_expected[4],
        )
        self.assertTrue(result.success, result.message)
        self.assertLess(result.error_norm_mm, 1e-3)
        self.assertIsNotNone(result.yaw_error_deg)
        self.assertLess(abs(result.yaw_error_deg), 1e-3)
        self.assertAlmostEqual(result.q_deg[4], q_expected[4], places=10)

    def test_feasible_recommendation_never_changes_j5(self) -> None:
        q = [-10.0, 15.0, 20.0, 30.0, -25.0]
        pose = forward_kinematics(q)
        result = recommend_feasible_yaw(
            pose.position_mm,
            pose.yaw_deg,
            q[4],
            q,
        )
        self.assertTrue(result.success)
        self.assertFalse(result.changed_yaw)
        self.assertFalse(result.changed_j5)
        self.assertAlmostEqual(result.recommended_j5_deg, q[4], places=10)

    def test_line_interpolation_tracks_a_changing_yaw(self) -> None:
        q_start = [-10.0, 15.0, 20.0, 30.0, 5.0]
        target_pose = forward_kinematics([-12.0, 15.0, 20.0, 30.0, 5.0])
        plan = plan_cartesian_line_trajectory(
            q_start,
            target_pose.position_mm,
            target_yaw_deg=target_pose.yaw_deg,
            target_j5_deg=5.0,
            velocity_limit_deg_s=1000.0,
            acceleration_limit_deg_s2=10000.0,
            sample_period_s=0.02,
            position_tolerance_mm=0.1,
        )
        self.assertLess(plan.max_line_deviation_mm, 0.01)
        self.assertLess(plan.max_abs_yaw_error_deg, 0.5)
        self.assertAlmostEqual(
            plan.actual_yaw_deg[-1],
            target_pose.yaw_deg,
            delta=0.5,
        )


if __name__ == "__main__":
    unittest.main()
