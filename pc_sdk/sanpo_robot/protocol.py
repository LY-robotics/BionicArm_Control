"""SANPO factory ST frame plus board-application commands."""

from dataclasses import dataclass

APP_CHANNEL = 0xFE
APP_REQUEST_ID = 0x7F0
APP_RESPONSE_ID = 0x7F1

BOARD_INFO = 0x01
MOVE_JOINT = 0x02
GET_STATE = 0x03
STOP_ALL = 0x04
HEARTBEAT = 0x05
HOME = 0x06
CLEAR_FAULT = 0x07
STAGE_GROUP = 0x08
EXECUTE_GROUP = 0x09
GROUP_STATUS = 0x0A


@dataclass
class StFrame:
    channel: int
    can_id: int
    data: bytes

    def encode(self) -> bytes:
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("Standard CAN ID must be 0..0x7FF")
        if len(self.data) > 8:
            raise ValueError("ST frame payload cannot exceed 8 bytes")
        return (
            b"ST"
            + bytes([self.channel & 0xFF])
            + b"\x00\x00"
            + self.can_id.to_bytes(2, "big")
            + bytes([len(self.data)])
            + self.data
            + b"\r\n"
        )


class StParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[StFrame]:
        self.buffer += data
        frames: list[StFrame] = []

        while len(self.buffer) >= 10:
            if self.buffer[:2] != b"ST":
                del self.buffer[0]
                continue
            dlc = self.buffer[7]
            if dlc > 8:
                del self.buffer[0]
                continue
            length = 10 + dlc
            if len(self.buffer) < length:
                break
            if self.buffer[length - 2 : length] != b"\r\n":
                del self.buffer[0]
                continue
            frames.append(
                StFrame(
                    channel=self.buffer[2],
                    can_id=int.from_bytes(self.buffer[5:7], "big"),
                    data=bytes(self.buffer[8 : 8 + dlc]),
                )
            )
            del self.buffer[:length]
        return frames


def app_request(command: int, payload: bytes = b"") -> bytes:
    return StFrame(APP_CHANNEL, APP_REQUEST_ID, bytes([command]) + payload).encode()
