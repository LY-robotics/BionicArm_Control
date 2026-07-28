"""High-level PC API for one SANPO STM32 board."""

import struct
import time

import serial

from .protocol import (
    APP_RESPONSE_ID,
    BOARD_INFO,
    CLEAR_FAULT,
    GET_STATE,
    GROUP_STATUS,
    HEARTBEAT,
    HOME,
    MOVE_JOINT,
    EXECUTE_GROUP,
    STAGE_GROUP,
    STOP_ALL,
    StParser,
    app_request,
)


class SanpoBoard:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.serial = serial.Serial(port, baudrate, timeout=0.02)
        self.parser = StParser()

    def close(self) -> None:
        self.serial.close()

    def _request(self, command: int, payload: bytes = b"",
                 timeout: float = 0.5) -> bytes:
        self.serial.write(app_request(command, payload))
        self.serial.flush()
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            for frame in self.parser.feed(self.serial.read(128)):
                if frame.can_id != APP_RESPONSE_ID or len(frame.data) < 2:
                    continue
                if frame.data[0] != (command | 0x80):
                    continue
                status = frame.data[1]
                if status != 0:
                    raise RuntimeError(
                        f"{self.port}: command 0x{command:02X}, status={status}"
                    )
                return frame.data[2:]
        raise TimeoutError(f"{self.port}: command 0x{command:02X} timeout")

    def board_info(self) -> dict:
        data = self._request(BOARD_INFO)
        if len(data) != 6:
            raise RuntimeError("Invalid BOARD_INFO response")
        return {
            "board_id": data[0],
            "arm_id": data[1],
            "physical_can1": data[2],
            "physical_can2": data[3],
            "joint_count": data[4],
            "protocol_version": data[5],
        }

    def heartbeat(self) -> None:
        self._request(HEARTBEAT)

    def move_joint(self, joint_id: int, angle_deg: float,
                   speed_rpm: float) -> None:
        angle_raw = round(angle_deg * 100)
        speed_raw = round(speed_rpm * 100)
        payload = (
            bytes([joint_id])
            + struct.pack("<i", angle_raw)
            + struct.pack("<H", speed_raw)
        )
        self._request(MOVE_JOINT, payload)

    def get_state(self, joint_id: int) -> dict:
        data = self._request(GET_STATE, bytes([joint_id]))
        if len(data) != 6:
            raise RuntimeError("Invalid GET_STATE response")
        angle_raw, speed_raw = struct.unpack_from("<hh", data, 1)
        return {
            "joint_id": data[0],
            "angle_deg": angle_raw / 100.0,
            "speed_rpm": speed_raw / 100.0,
            "fault": data[5],
        }

    def stop_all(self) -> None:
        self._request(STOP_ALL)

    def home(self, joint_id: int) -> None:
        self._request(HOME, bytes([joint_id]))

    def clear_fault(self, joint_id: int) -> None:
        self._request(CLEAR_FAULT, bytes([joint_id]))

    def stage_group_joint(self, joint_id: int, angle_deg: float) -> None:
        payload = bytes([joint_id]) + struct.pack("<i", round(angle_deg * 100))
        self._request(STAGE_GROUP, payload)

    def execute_group(self, duration_ms: int) -> None:
        if not 200 <= duration_ms <= 60000:
            raise ValueError("duration_ms must be 200..60000")
        self._request(EXECUTE_GROUP, struct.pack("<H", duration_ms))

    def group_status(self) -> dict:
        data = self._request(GROUP_STATUS)
        if len(data) != 6:
            raise RuntimeError("Invalid GROUP_STATUS response")
        return {
            "staged_mask": data[0],
            "active_mask": data[1],
            "done_mask": data[2],
            "fault_mask": data[3],
            "sequence": data[4],
            "active": bool(data[5]),
        }

    def move_group(self, angles_deg: list[float], duration_ms: int,
                   wait: bool = True, timeout: float | None = None) -> dict:
        if len(angles_deg) != 5:
            raise ValueError("angles_deg must contain J1..J5")
        for joint_id, angle_deg in enumerate(angles_deg, start=1):
            self.stage_group_joint(joint_id, angle_deg)
        self.execute_group(duration_ms)

        if not wait:
            return self.group_status()
        deadline = time.monotonic() + (
            timeout if timeout is not None else duration_ms / 1000.0 + 5.0
        )
        while time.monotonic() < deadline:
            state = self.group_status()
            if state["fault_mask"]:
                raise RuntimeError(
                    f"Coordinated motion fault mask=0x{state['fault_mask']:02X}"
                )
            if not state["active"]:
                return state
            time.sleep(0.05)
        self.stop_all()
        raise TimeoutError("Coordinated motion timeout; STOP_ALL sent")
