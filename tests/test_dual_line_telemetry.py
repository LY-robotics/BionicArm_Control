"""Regression tests for the new interpolation, dual-arm and feedback layers."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from sanpo_arm_sdk import OK, TelemetryRecorder, create_dual_simulated_controller
from sanpo_arm_sdk.kinematics import forward_kinematics


class DualLineTelemetryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.dual = create_dual_simulated_controller()
        self.assertTrue(self.dual.connect().success)
        self.assertTrue(self.dual.sync_state().success)

    def tearDown(self) -> None:
        self.dual.close()

    def test_dual_joint_move_has_common_duration_and_reaches_both_targets(self) -> None:
        left_target = [-10.0, 15.0, 20.0, 30.0, 5.0]
        right_target = [-8.0, 12.0, 15.0, 25.0, 3.0]
        result, prepared = self.dual.prepare_both(
            left_target,
            right_target,
            mode="joint",
            speed=1000.0,
            accel=10000.0,
            sample_period_s=0.01,
            synchronize_finish=True,
        )
        self.assertTrue(result.success)
        self.assertIsNotNone(prepared)
        self.assertAlmostEqual(
            prepared["left"].trajectory.time_s[-1],
            prepared["right"].trajectory.time_s[-1],
        )
        self.assertTrue(self.dual.execute_prepared_both(prepared).success)
        np.testing.assert_allclose(
            [self.dual.left.hardware.positions[key] for key in self.dual.left.joints],
            left_target,
        )
        np.testing.assert_allclose(
            [self.dual.right.hardware.positions[key] for key in self.dual.right.joints],
            right_target,
        )

    def test_tcp_line_interpolation_stays_on_requested_line(self) -> None:
        arm = self.dual.right
        source_joints = [-10.0, 15.0, 20.0, 30.0, 5.0]
        self.assertEqual(
            arm.MoveJ(
                source_joints,
                speed=1000.0,
                accel=10000.0,
                sample_period_s=0.01,
            ),
            OK,
        )
        pose = forward_kinematics(source_joints)
        target = [pose.x + 2.0, pose.y, pose.z, pose.yaw_deg, source_joints[4]]
        error, motion = arm.prepare_move_line(
            target,
            speed=1000.0,
            accel=10000.0,
            sample_period_s=0.02,
            position_tolerance_mm=0.1,
        )
        self.assertEqual(error, OK)
        self.assertIsNotNone(motion)
        self.assertLess(motion.line_plan.max_line_deviation_mm, 0.01)
        self.assertLessEqual(motion.line_plan.position_error_mm[-1], 0.1)
        self.assertLessEqual(motion.line_plan.max_abs_yaw_error_deg, 0.5)
        self.assertEqual(arm.execute_prepared_motion(motion), OK)
        reached = forward_kinematics(
            [arm.hardware.positions[key] for key in arm.joints]
        )
        np.testing.assert_allclose(reached.position_mm, target[:3], atol=0.11)

    def test_feasible_pose_recommendation_keeps_requested_yaw_and_j5(self) -> None:
        arm = self.dual.right
        source_joints = [-10.0, 15.0, 20.0, 30.0, 5.0]
        self.assertEqual(
            arm.MoveJ(source_joints, speed=1000.0, accel=10000.0),
            OK,
        )
        pose = forward_kinematics(source_joints)
        error, result = arm.preview_ik_recommendation(
            [pose.x, pose.y, pose.z, pose.yaw_deg, source_joints[4]]
        )
        self.assertEqual(error, OK)
        self.assertTrue(result.success)
        self.assertFalse(result.changed_yaw)
        self.assertFalse(result.changed_j5)

    def test_feedback_records_peaks_and_exports_utf8_csv(self) -> None:
        recorder = TelemetryRecorder(
            {"left": self.dual.left, "right": self.dual.right},
            sample_period_s=0.01,
        )
        samples = recorder.poll_once()
        self.assertEqual(len(samples), 10)
        self.assertEqual(len(recorder.peak_summaries()), 10)
        with tempfile.TemporaryDirectory() as directory:
            destination = recorder.export_csv(Path(directory) / "feedback.csv")
            self.assertTrue(destination.exists())
            text = destination.read_text(encoding="utf-8-sig")
            self.assertIn("elapsed_s,arm,joint,angle_deg", text)
            self.assertIn("left,j1", text)
            peak_destination = recorder.export_peak_csv(
                Path(directory) / "feedback_peaks.csv"
            )
            peak_text = peak_destination.read_text(encoding="utf-8-sig")
            self.assertIn("max_abs_speed_rpm,max_abs_current_a", peak_text)


if __name__ == "__main__":
    unittest.main()
