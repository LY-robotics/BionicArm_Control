"""Protocol framing and fast trajectory-command tests."""

import unittest
import time

from sanpo_arm_sdk.protocol.can_motor_arm_lib import (
    CanMotor,
    SerialUsbCanTransport,
)


class RecordingBus:
    def __init__(self) -> None:
        self.frames = []

    def send_std(self, can_id: int, data: bytes, flush_rx: bool = True) -> None:
        self.frames.append((can_id, data, flush_rx))


class FakeSerial:
    def __init__(self, response: bytes) -> None:
        self.is_open = True
        self.response = response
        self.rx = bytearray()
        self.writes: list[bytes] = []

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))
        self.rx.extend(self.response)

    def flush(self) -> None:
        pass

    def read(self, size: int) -> bytes:
        if not self.rx or size <= 0:
            return b""
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result


class ProtocolTests(unittest.TestCase):
    def test_arm_profile_swaps_original_j1_j2_for_motion_order(self) -> None:
        from sanpo_arm_sdk.config import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS

        self.assertEqual(
            [joint.motor_id for joint in RIGHT_ARM_JOINTS],
            [34, 35, 31, 32, 33],
        )
        self.assertEqual(
            [joint.motor_id for joint in LEFT_ARM_JOINTS],
            [1, 55, 15, 18, 23],
        )
        for profile in (RIGHT_ARM_JOINTS, LEFT_ARM_JOINTS):
            self.assertEqual(
                (profile[0].min_deg, profile[0].max_deg),
                (-170.0, 120.0),
            )
            self.assertEqual(
                (profile[1].min_deg, profile[1].max_deg),
                (-110.0, 110.0),
            )

    def test_v41_standard_frame_contains_channel(self) -> None:
        bus = SerialUsbCanTransport(
            "TEST",
            usb_mode="standard",
            channel=2,
        )
        packet = bus.build_packet(0x123, b"\x01\x02\x03")
        self.assertEqual(
            packet,
            b"ST\x02\x00\x00\x01\x23\x03\x01\x02\x03\r\n",
        )

        frames = bus._parse_standard_frames(packet)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].channel, 2)
        self.assertEqual(frames[0].can_id, 0x123)
        self.assertEqual(frames[0].data, b"\x01\x02\x03")

    def test_channel_endpoints_share_parent_but_build_distinct_packets(self) -> None:
        bus = SerialUsbCanTransport("TEST", usb_mode="standard")
        arm_endpoint = bus.channel_endpoint(1)
        gripper_endpoint = bus.channel_endpoint(2)

        arm_packet = bus.build_packet(0x123, b"\x01", channel=arm_endpoint.channel)
        gripper_packet = bus.build_packet(
            0x123,
            b"\x01",
            channel=gripper_endpoint.channel,
        )

        self.assertEqual(arm_packet[2], 1)
        self.assertEqual(gripper_packet[2], 2)
        self.assertIs(arm_endpoint.parent, gripper_endpoint.parent)

    def test_standard_stream_parser_retains_fragmented_frame(self) -> None:
        bus = SerialUsbCanTransport("TEST", usb_mode="standard", channel=2)
        packet = bus.build_packet(0x321, b"\x10\x20")
        buffer = bytearray(packet[:5])

        self.assertEqual(bus._consume_standard_buffer(buffer), [])
        self.assertEqual(bytes(buffer), packet[:5])
        buffer.extend(packet[5:])
        frames = bus._consume_standard_buffer(buffer)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].channel, 2)
        self.assertEqual(frames[0].can_id, 0x321)
        self.assertEqual(frames[0].data, b"\x10\x20")
        self.assertEqual(buffer, b"")

    def test_channel_request_returns_matching_frame_without_full_timeout(self) -> None:
        bus = SerialUsbCanTransport("TEST", usb_mode="standard")
        other = bus.build_packet(0x50, b"\x99", channel=1)
        expected = bus.build_packet(0x60, b"\x42", channel=2)
        bus.ser = FakeSerial(other + expected)
        endpoint = bus.channel_endpoint(2)

        started = time.monotonic()
        frame = endpoint.request(
            0x7FF,
            b"\x01",
            expect_reply_id=0x60,
            timeout=0.5,
        )
        elapsed = time.monotonic() - started

        self.assertIsNotNone(frame)
        self.assertEqual(frame.channel, 2)
        self.assertEqual(frame.can_id, 0x60)
        self.assertLess(elapsed, 0.1)

    def test_advanced_frame_round_trip(self) -> None:
        bus = SerialUsbCanTransport("TEST", usb_mode="advanced")
        packet = bus.build_packet(0x142, b"\xA3")
        frames = bus._parse_advanced_frames(packet)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].can_id, 0x142)
        self.assertEqual(frames[0].data, b"\xA3")

    def test_fast_position_command_does_not_wait_for_reply(self) -> None:
        bus = RecordingBus()
        motor = CanMotor(bus, motor_id=0x22)
        motor.command_absolute_motor_deg(90.0)

        self.assertEqual(len(bus.frames), 1)
        can_id, data, _ = bus.frames[0]
        self.assertEqual(can_id, 0x122)
        self.assertEqual(data[0], 0xC2)
        self.assertEqual(
            int.from_bytes(data[1:5], byteorder="little", signed=True),
            4096,
        )


if __name__ == "__main__":
    unittest.main()
