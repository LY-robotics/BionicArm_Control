"""Protocol and topology tests for the independent Gloria-M controller."""

import tempfile
import unittest
from pathlib import Path

from sanpo_arm_sdk import (
    GloriaGripper,
    GloriaGripperConfig,
    DualF4System,
    DualGripperController,
    GripperCalibration,
    GripperTelemetryRecorder,
    SimulatedGripper,
    create_dual_f4_system,
    create_dual_simulated_controller,
    create_dual_simulated_system,
)
from sanpo_arm_sdk.end_effectors.errors import GripperConfigurationError
from sanpo_arm_sdk.end_effectors.gloria_protocol import (
    SET_ZERO_PAYLOAD,
    pack_mit,
    parse_feedback,
)
from sanpo_arm_sdk.end_effectors.models import GripperLimits
from sanpo_arm_sdk.protocol.can_motor_arm_lib import CanFrame


class FakeCanEndpoint:
    def __init__(self, *, reply_can_id: int = 0) -> None:
        self.is_open = False
        self.reply_can_id = reply_can_id
        self.sent: list[tuple[int, bytes, bool]] = []
        self.requests: list[tuple[int, bytes, int | None]] = []

    def connect(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def send_std(
        self,
        can_id: int,
        data: bytes,
        flush_rx: bool = True,
    ) -> None:
        self.sent.append((can_id, bytes(data), flush_rx))

    def request(
        self,
        can_id: int,
        data: bytes,
        expect_reply_id: int | None = None,
        expect_cmd: int | None = None,
        timeout: float = 0.25,
        *,
        match=None,
    ) -> CanFrame | None:
        del expect_cmd, timeout
        self.requests.append((can_id, bytes(data), expect_reply_id))
        if len(data) >= 4 and data[2] in (0x33, 0x55, 0xAA):
            response_data = bytes(data[:8]).ljust(8, b"\x00")
            if data[2] == 0xAA:
                response_data = bytes([data[0], data[1], 0xAA, 1, 0, 0, 0, 0])
        else:
            # Enabled motor 1, all mapped fields near their midpoints.
            response_data = bytes([0x11, 0x80, 0, 0x80, 0, 0x80, 25, 26])
        frame = CanFrame(
            can_id=self.reply_can_id,
            data=response_data,
            channel=2,
        )
        return frame if match is None or match(frame) else None


class GripperIntegrationTests(unittest.TestCase):
    def test_mit_parameters_are_rejected_instead_of_silently_clipped(self) -> None:
        limits = GripperLimits()
        with self.assertRaises(GripperConfigurationError):
            pack_mit(0.0, 99.0, 0.0, 0.5, 0.0, limits)
        with self.assertRaises(GripperConfigurationError):
            pack_mit(0.0, 0.0, 600.0, 0.5, 0.0, limits)

    def test_feedback_checks_expected_motor_id(self) -> None:
        with self.assertRaises(Exception):
            parse_feedback(
                bytes([0x12, 0x80, 0, 0x80, 0, 0x80, 25, 26]),
                GripperLimits(),
                expected_motor_id=1,
            )

    def test_gloria_controller_uses_injected_can_endpoint(self) -> None:
        endpoint = FakeCanEndpoint()
        gripper = GloriaGripper(
            endpoint,
            config=GloriaGripperConfig(
                calibration=GripperCalibration(0.0, 2.0),
            ),
        )
        gripper.connect()
        gripper.enable()
        state = gripper.move_normalized(0.5, 0.2)

        self.assertTrue(endpoint.is_open)
        self.assertEqual(endpoint.sent[0][0], 1)
        self.assertEqual(endpoint.sent[1][0], 0x101)
        self.assertEqual(endpoint.requests[-1][0], 0x7FF)
        self.assertIsNone(endpoint.requests[-1][2])
        self.assertEqual(state.motor_id, 1)

    def test_feedback_master_id_is_discovered_from_matching_payload(self) -> None:
        endpoint = FakeCanEndpoint(reply_can_id=0x321)
        gripper = GloriaGripper(
            endpoint,
            config=GloriaGripperConfig(master_can_id=0),
        )
        gripper.connect()

        gripper.refresh_state()

        self.assertEqual(gripper.feedback_can_id, 0x321)
        self.assertEqual(gripper.feedback_channel, 2)
        self.assertIsNone(endpoint.requests[-1][2])

    def test_enable_selects_position_velocity_mode_without_flash_save(self) -> None:
        endpoint = FakeCanEndpoint()
        gripper = GloriaGripper(endpoint)
        gripper.connect()

        gripper.enable()

        mode_request = endpoint.requests[0]
        self.assertEqual(mode_request[0], 0x7FF)
        self.assertEqual(mode_request[1][2:4], bytes([0x55, 10]))
        self.assertEqual(int.from_bytes(mode_request[1][4:8], "little"), 2)
        self.assertEqual(endpoint.sent[-1][0], 1)

    def test_gripper_zero_requires_confirmation_and_sends_device_command(self) -> None:
        endpoint = FakeCanEndpoint()
        gripper = GloriaGripper(endpoint)
        gripper.connect()

        with self.assertRaises(GripperConfigurationError):
            gripper.set_zero()

        gripper.set_zero(confirm=True)
        self.assertEqual(endpoint.sent[-1][0], 1)
        self.assertEqual(endpoint.sent[-1][1], SET_ZERO_PAYLOAD)

    def test_dual_f4_factory_binds_four_separate_can_channels(self) -> None:
        system = create_dual_f4_system(
            "LEFT_TEST",
            "RIGHT_TEST",
            left_arm_channel=1,
            left_gripper_channel=2,
            right_arm_channel=3,
            right_gripper_channel=4,
        )

        self.assertEqual(system.arms.left.hardware.bus.channel, 1)
        self.assertEqual(system.grippers.left.transport.channel, 2)
        self.assertEqual(system.arms.right.hardware.bus.channel, 3)
        self.assertEqual(system.grippers.right.transport.channel, 4)
        self.assertIs(
            system.arms.left.hardware.transport,
            system.grippers.left.transport.parent,
        )
        self.assertIs(
            system.arms.right.hardware.transport,
            system.grippers.right.transport.parent,
        )

    def test_arm_and_gripper_channels_cannot_collide_on_one_f4(self) -> None:
        with self.assertRaises(ValueError):
            create_dual_f4_system(
                "LEFT_TEST",
                "RIGHT_TEST",
                left_arm_channel=1,
                left_gripper_channel=1,
            )

    def test_simulated_system_keeps_arm_and_gripper_objects_separate(self) -> None:
        system = create_dual_simulated_system()
        connection = system.connect()
        self.assertTrue(connection.success)
        self.assertTrue(system.arms.sync_state().success)
        self.assertTrue(system.grippers.enable_both().success)
        result, states = system.grippers.move_both(0.25, 0.75, 0.2)
        self.assertTrue(result.success)
        self.assertAlmostEqual(states["left"].opening_fraction, 0.25)
        self.assertAlmostEqual(states["right"].opening_fraction, 0.75)
        system.close()

    def test_disabled_left_gripper_is_skipped_while_right_remains_usable(self) -> None:
        system = create_dual_simulated_system(
            left_gripper_enabled=False,
            right_gripper_enabled=True,
        )
        connection = system.connect()

        self.assertTrue(connection.success)
        self.assertTrue(connection.all_requested_devices_connected)
        self.assertFalse(connection.grippers.left_available)
        self.assertTrue(connection.grippers.right_available)

        result, states = system.grippers.move_both(0.25, 0.75, 0.2)
        self.assertTrue(result.success)
        self.assertIsNone(states["left"])
        self.assertAlmostEqual(states["right"].opening_fraction, 0.75)
        system.close()

    def test_missing_configured_gripper_does_not_disconnect_arms(self) -> None:
        class MissingFeedbackGripper(SimulatedGripper):
            def refresh_state(self):
                raise RuntimeError("no CAN feedback")

        system = DualF4System(
            create_dual_simulated_controller(),
            DualGripperController(
                MissingFeedbackGripper(name="missing_left"),
                SimulatedGripper(name="working_right"),
            ),
        )
        connection = system.connect()

        self.assertTrue(connection.success)
        self.assertFalse(connection.all_requested_devices_connected)
        self.assertFalse(connection.grippers.left_success)
        self.assertTrue(connection.grippers.right_success)
        self.assertTrue(system.arms.sync_state().success)
        system.close()

    def test_gripper_telemetry_exports_both_sides(self) -> None:
        system = create_dual_simulated_system()
        self.assertTrue(system.connect().success)
        recorder = GripperTelemetryRecorder(system.grippers)
        samples = recorder.poll_once()
        self.assertEqual({sample.side for sample in samples}, {"left", "right"})
        self.assertEqual(len(recorder.peak_summaries()), 2)
        with tempfile.TemporaryDirectory() as directory:
            path = recorder.export_csv(Path(directory) / "grippers.csv")
            text = path.read_text(encoding="utf-8-sig")
            self.assertIn("opening_fraction", text)
            self.assertIn(",left,", text)
            self.assertIn(",right,", text)
            peak_path = recorder.export_peak_csv(
                Path(directory) / "grippers_peaks.csv"
            )
            peak_text = peak_path.read_text(encoding="utf-8-sig")
            self.assertIn("max_abs_torque_nm", peak_text)
            self.assertIn("left,1", peak_text)
        system.close()


if __name__ == "__main__":
    unittest.main()
