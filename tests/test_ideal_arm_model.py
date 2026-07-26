"""Geometry invariants for the ideal link and coordinate-frame model."""

import unittest
import tempfile
from pathlib import Path

import numpy as np

from sanpo_arm_sdk.kinematics import (
    build_ideal_arm_model,
    compare_tcp_positions,
    export_ideal_model_csv,
    forward_kinematics,
)
from sanpo_arm_sdk.kinematics.kinematic_5dof import (
    FOREARM_MM,
    TCP_OFFSET_WRIST_MM,
    UPPER_ARM_MM,
)


class IdealArmModelTests(unittest.TestCase):
    def test_model_tcp_is_identical_to_active_forward_kinematics(self) -> None:
        for q_deg in (
            [0.0, 0.0, 0.0, 30.0, 0.0],
            [-10.0, 15.0, 20.0, 30.0, 5.0],
            [-30.0, -20.0, 40.0, 50.0, -10.0],
        ):
            model = build_ideal_arm_model(q_deg)
            pose = forward_kinematics(q_deg)
            np.testing.assert_allclose(
                model.tcp_position_mm,
                pose.position_mm,
                atol=1e-10,
            )
            np.testing.assert_allclose(
                model.tcp_frame.rotation,
                pose.rotation,
                atol=1e-10,
            )

    def test_link_lengths_follow_the_kinematic_constants(self) -> None:
        model = build_ideal_arm_model([-10.0, 15.0, 20.0, 30.0, 5.0])
        points = model.link_points_mm
        self.assertAlmostEqual(
            float(np.linalg.norm(points[2] - points[1])),
            UPPER_ARM_MM,
            places=9,
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(points[3] - points[2])),
            FOREARM_MM,
            places=9,
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(points[4] - points[3])),
            float(np.linalg.norm(TCP_OFFSET_WRIST_MM)),
            places=9,
        )

    def test_all_coordinate_frames_are_right_handed_and_orthonormal(self) -> None:
        model = build_ideal_arm_model([-10.0, 15.0, 20.0, 30.0, 5.0])
        self.assertEqual(
            [frame.name for frame in model.frames],
            ["Base", "J1", "J2", "J3", "J4", "J5", "TCP"],
        )
        for frame in model.frames:
            rotation = frame.rotation
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
            self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=10)

    def test_position_comparison_reports_all_three_distances(self) -> None:
        theoretical = build_ideal_arm_model([0.0, 0.0, 0.0, 30.0, 0.0])
        feedback = build_ideal_arm_model([-1.0, 1.0, 0.0, 30.0, 0.0])
        target = theoretical.tcp_position_mm + np.array([1.0, 2.0, 2.0])
        comparison = compare_tcp_positions(target, theoretical, feedback)
        self.assertAlmostEqual(comparison.target_to_theoretical_mm, 3.0)
        self.assertIsNotNone(comparison.target_to_feedback_mm)
        self.assertIsNotNone(comparison.theoretical_to_feedback_mm)

    def test_model_csv_contains_frames_joints_and_errors(self) -> None:
        theoretical = build_ideal_arm_model([0.0, 0.0, 0.0, 30.0, 0.0])
        feedback = build_ideal_arm_model([-1.0, 1.0, 0.0, 30.0, 0.0])
        with tempfile.TemporaryDirectory() as directory:
            destination = export_ideal_model_csv(
                Path(directory) / "model.csv",
                target_mm=theoretical.tcp_position_mm,
                theoretical_model=theoretical,
                feedback_model=feedback,
            )
            text = destination.read_text(encoding="utf-8-sig")
        self.assertIn("frame,Base", text)
        self.assertIn("frame,TCP", text)
        self.assertIn("joint,theoretical_J1", text)
        self.assertIn("joint,feedback_J5", text)
        self.assertIn("error,theoretical_to_feedback", text)


if __name__ == "__main__":
    unittest.main()
