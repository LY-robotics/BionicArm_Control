"""End-to-end kinematics, trajectory and backend tests without hardware."""

import unittest

import numpy as np

from sanpo_arm_sdk.errors import ERR_OUT_OF_LIMIT, OK
from sanpo_arm_sdk.factory import create_simulated_controller
from sanpo_arm_sdk.kinematics import forward_kinematics


class MotionChainTests(unittest.TestCase):
    def setUp(self) -> None:
        self.arm = create_simulated_controller(profile="right")
        self.assertEqual(self.arm.connect(), OK)

    def tearDown(self) -> None:
        self.arm.close()

    def test_movej_reaches_planned_joint_target(self) -> None:
        target = [-10.0, 15.0, 20.0, 30.0, 5.0]
        err = self.arm.MoveJ(
            target,
            speed=1000.0,
            accel=10000.0,
            sample_period_s=0.01,
            timeout_s=1.0,
        )
        self.assertEqual(err, OK)
        actual = [self.arm.hardware.positions[key] for key in self.arm.joints]
        np.testing.assert_allclose(actual, target, atol=1e-9)

    def test_coordinate_to_ik_to_can_backend_chain(self) -> None:
        source_joints = [-20.0, 20.0, 30.0, 45.0, 0.0]
        pose = forward_kinematics(source_joints)
        target = [pose.x, pose.y, pose.z, pose.yaw_deg, source_joints[4]]

        err = self.arm.MoveCart(
            target,
            speed=1000.0,
            accel=10000.0,
            sample_period_s=0.01,
            timeout_s=1.0,
        )

        self.assertEqual(err, OK)
        self.assertIsNotNone(self.arm.last_ik_result)
        actual = np.array(
            [self.arm.hardware.positions[key] for key in self.arm.joints]
        )
        np.testing.assert_allclose(
            actual,
            self.arm.last_ik_result.q_deg,
            atol=1e-9,
        )
        reached_pose = forward_kinematics(actual)
        np.testing.assert_allclose(
            reached_pose.position_mm,
            pose.position_mm,
            atol=1e-3,
        )

    def test_joint_limit_is_checked_before_sending(self) -> None:
        err = self.arm.MoveJ([999.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(err, ERR_OUT_OF_LIMIT)

    def test_live_joint_feedback_builds_an_ideal_model(self) -> None:
        target = [-10.0, 15.0, 20.0, 30.0, 5.0]
        self.assertEqual(
            self.arm.MoveJ(target, speed=1000.0, accel=10000.0),
            OK,
        )
        error, model = self.arm.ideal_model()
        self.assertEqual(error, OK)
        self.assertIsNotNone(model)
        np.testing.assert_allclose(model.q_deg, target)


if __name__ == "__main__":
    unittest.main()
