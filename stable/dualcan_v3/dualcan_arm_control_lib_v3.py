# -*- coding: utf-8 -*-
"""
dualcan_arm_control_lib_v3.py

专用于“同一个 COM 口 + 双 CAN 通道 + 双机械臂”的控制库。

本 V3 版本的关键修正：
    Normal/Passthrough 串口发送帧使用“可变长度 DATA 区”。
    即 DLC=真实 CAN 数据长度，串口帧尾 0D 0A 紧跟在 data 后面。

例如：
    A0 查询版本：
        53 54 01 00 00 01 22 01 A0 0D 0A

    C1 速度 5RPM：
        53 54 01 00 00 01 22 05 C1 F4 01 00 00 0D 0A

不要写成：
        53 54 01 00 00 01 22 05 C1 F4 01 00 00 00 00 00 0D 0A

硬件结构目标：
    COM17
      ├── Channel 1 / CAN1 -> 右臂 ARM0
      └── Channel 2 / CAN2 -> 左臂 ARM1

依赖：
    pip install pyserial
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union

import serial


HEADER_STD = b"\x53\x54"   # ST，标准帧
HEADER_EXT = b"\x45\x54"   # ET，扩展帧；本库暂不主动使用
TAIL = b"\x0D\x0A"


# ============================================================
# 工具函数
# ============================================================

def format_hex(data: bytes) -> str:
    return "0x" + " ".join(f"{b:02X}" for b in data) if data else "0x"


def u32_le(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=False)


def s32_le(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=True)


def read_u16_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=False)


def read_s16_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def read_u32_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=False)


def read_s32_le(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=True)


def float32_le(value: float) -> bytes:
    return struct.pack("<f", float(value))


def read_float32_le(data: bytes, offset: int) -> float:
    return struct.unpack("<f", data[offset:offset + 4])[0]


# ============================================================
# 异常
# ============================================================

class CanTransportError(RuntimeError):
    pass


class CanTimeoutError(CanTransportError):
    pass


class MotorProtocolError(RuntimeError):
    pass


class JointLimitError(ValueError):
    pass


# ============================================================
# CAN 帧
# ============================================================

@dataclass
class CanFrame:
    channel: int
    can_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False

    @property
    def dlc(self) -> int:
        return len(self.data)

    def __repr__(self) -> str:
        typ = "EXT" if self.is_extended else "STD"
        return f"CanFrame(ch={self.channel}, id=0x{self.can_id:X}, dlc={self.dlc}, data={format_hex(self.data)}, {typ})"


# ============================================================
# 底层：同 COM 多 CAN 通道 Normal/Passthrough 传输层
# ============================================================

class SerialDualCanNormalBus:
    """
    一个 COM 口下的多 CAN 通道总线控制器。

    发送标准帧格式：
        53 54 + channel + can_id(4B big-endian) + dlc + data(dlc bytes) + 0D 0A

    接收解析同时兼容两种格式：
        1. 可变长度：data(dlc) 后跟 0D 0A
        2. 固定 8 字节：data(8) 后跟 0D 0A
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        response_timeout: float = 0.35,
        debug: bool = False,
        enter_passthrough: bool = True,
    ):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.response_timeout = float(response_timeout)
        self.debug = bool(debug)
        self.enter_passthrough = bool(enter_passthrough)
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.RLock()

    def connect(self) -> None:
        if self.ser and self.ser.is_open:
            return

        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        if self.enter_passthrough:
            self.ser.write(b"AT+ET\r\n")
            self.ser.flush()
            rx = self._read_raw(total_timeout=0.5)
            if self.debug:
                print("[MODE TX] AT+ET")
                print("[MODE RX]", format_hex(rx))
            if b"OK" not in rx:
                raise CanTransportError(f"AT+ET 未收到 OK，RX={format_hex(rx)}")

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "SerialDualCanNormalBus":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def channel(self, channel: int) -> "CanChannel":
        return CanChannel(self, int(channel))

    def _ensure_open(self) -> serial.Serial:
        if not self.ser or not self.ser.is_open:
            raise CanTransportError("串口未打开，请先 connect()。")
        return self.ser

    def _read_raw(self, total_timeout: Optional[float] = None, idle_timeout: float = 0.03) -> bytes:
        ser = self._ensure_open()
        deadline = time.monotonic() + (self.response_timeout if total_timeout is None else float(total_timeout))
        last_data = time.monotonic()
        chunks: List[bytes] = []

        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            data = ser.read(waiting or 1)
            if data:
                chunks.append(data)
                last_data = time.monotonic()
            elif chunks and time.monotonic() - last_data > idle_timeout:
                break

        return b"".join(chunks)

    @staticmethod
    def build_std_payload(channel: int, can_id: int, data: bytes) -> bytes:
        """
        构建可变长度 Normal 标准帧。
        注意：data 后面不补 0，TAIL 紧跟真实 data。
        """
        if not 0 <= int(channel) <= 0xFF:
            raise ValueError("channel 必须是 0~255。通常 CAN1=1，CAN2=2。")
        if not 0 <= int(can_id) <= 0x7FF:
            raise ValueError("标准 CAN ID 必须在 0x000~0x7FF。")
        if not 0 <= len(data) <= 8:
            raise ValueError("CAN data 长度必须是 0~8。")

        return (
            HEADER_STD
            + bytes([int(channel)])
            + int(can_id).to_bytes(4, "big", signed=False)
            + bytes([len(data)])
            + data
            + TAIL
        )

    @staticmethod
    def parse_frames(raw: bytes) -> List[CanFrame]:
        """
        解析串口返回帧。

        兼容：
        - 可变长度返回：ST + ch + id4 + dlc + data(dlc) + tail
        - 固定8字节返回：ST + ch + id4 + dlc + data(8) + tail
        """
        frames: List[CanFrame] = []
        i = 0

        while i < len(raw):
            std_pos = raw.find(HEADER_STD, i)
            ext_pos = raw.find(HEADER_EXT, i)
            candidates = [p for p in (std_pos, ext_pos) if p >= 0]
            if not candidates:
                break

            s = min(candidates)
            header = raw[s:s + 2]
            is_ext = header == HEADER_EXT

            if s + 8 > len(raw):
                break

            ch = raw[s + 2]
            can_id = int.from_bytes(raw[s + 3:s + 7], "big", signed=False)
            if not is_ext:
                can_id &= 0x7FF
            dlc = raw[s + 7]

            # 1. 优先按可变长度解析
            e_var = s + 8 + dlc
            if e_var + 2 <= len(raw) and raw[e_var:e_var + 2] == TAIL:
                data = raw[s + 8:e_var]
                frames.append(CanFrame(ch, can_id, data, is_extended=is_ext))
                i = e_var + 2
                continue

            # 2. 再按固定8字节解析
            e_fix = s + 8 + 8
            if e_fix + 2 <= len(raw) and raw[e_fix:e_fix + 2] == TAIL:
                data_area = raw[s + 8:s + 16]
                frames.append(CanFrame(ch, can_id, data_area[:min(dlc, 8)], is_extended=is_ext))
                i = e_fix + 2
                continue

            i = s + 1

        return frames

    def send_frame(
        self,
        channel: int,
        can_id: int,
        data: bytes,
        expect_reply: bool = False,
        reply_can_id: Optional[int] = None,
        reply_cmd: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> Optional[CanFrame]:
        ser = self._ensure_open()
        payload = self.build_std_payload(channel, can_id, data)

        with self._lock:
            ser.reset_input_buffer()
            ser.write(payload)
            ser.flush()

            if self.debug:
                print(f"[TX ch={channel} id=0x{can_id:X} data={format_hex(data)}]")
                print("  RAW:", format_hex(payload))

            if not expect_reply:
                return None

            deadline = time.monotonic() + (self.response_timeout if timeout is None else float(timeout))
            while time.monotonic() < deadline:
                raw = self._read_raw(total_timeout=max(0.05, deadline - time.monotonic()))
                if self.debug:
                    print("  RX RAW:", format_hex(raw))

                frames = self.parse_frames(raw)
                if self.debug:
                    for f in frames:
                        print("  RX FRAME:", f)

                for frame in frames:
                    if int(frame.channel) != int(channel):
                        continue
                    if reply_can_id is not None and int(frame.can_id) != int(reply_can_id):
                        continue
                    if reply_cmd is not None and (not frame.data or frame.data[0] != int(reply_cmd)):
                        continue
                    return frame

            raise CanTimeoutError(
                f"等待回复超时: ch={channel}, tx_id=0x{can_id:X}, "
                f"reply_id={reply_can_id}, reply_cmd={reply_cmd}"
            )


class CanChannel:
    """固定某个 CAN 通道的轻量封装。"""

    def __init__(self, parent: SerialDualCanNormalBus, channel: int):
        if not 0 <= int(channel) <= 255:
            raise ValueError("channel 必须是 0~255。")
        self.parent = parent
        self.channel = int(channel)

    def send_frame(self, can_id: int, data: bytes) -> None:
        self.parent.send_frame(self.channel, can_id, data, expect_reply=False)

    def request_frame(
        self,
        can_id: int,
        data: bytes,
        reply_can_id: Optional[int] = None,
        reply_cmd: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> CanFrame:
        frame = self.parent.send_frame(
            self.channel,
            can_id,
            data,
            expect_reply=True,
            reply_can_id=reply_can_id,
            reply_cmd=reply_cmd,
            timeout=timeout,
        )
        assert frame is not None
        return frame


# ============================================================
# 中层：单电机协议
# ============================================================

@dataclass
class MotorVersion:
    boot: int
    app: int
    hardware: int
    can_protocol: int


@dataclass
class MotorStatus:
    bus_voltage_v: float
    bus_current_a: float
    temperature_c: int
    mode: int
    fault: int

    @property
    def has_fault(self) -> bool:
        return self.fault != 0

    def mode_name(self) -> str:
        return {
            0: "关闭",
            1: "电压",
            2: "Q轴电流",
            3: "速度",
            4: "位置",
        }.get(self.mode, f"未知({self.mode})")


@dataclass
class MotorAngle:
    single_turn_count: int
    multi_turn_count: int

    @property
    def single_turn_deg(self) -> float:
        return self.single_turn_count * 360.0 / 16384.0

    @property
    def multi_turn_deg(self) -> float:
        return self.multi_turn_count * 360.0 / 16384.0


@dataclass
class MotorQuickFeedback:
    temperature_c: int
    q_current_a: float
    speed_rpm: float
    single_turn_deg: float


class CanMotor:
    """
    单个电机对象。

    motor_id:
        电机地址，十进制。例如 ID=34 就写 34。
    tx_id:
        默认使用 0x100 | motor_id。
    rx_id:
        默认使用 motor_id。
    """

    def __init__(self, bus: CanChannel, motor_id: int, use_direction_id: bool = True):
        if not 1 <= int(motor_id) <= 254:
            raise ValueError("motor_id 必须为 1~254。")
        self.bus = bus
        self.motor_id = int(motor_id)
        self.tx_id = (0x100 | self.motor_id) if use_direction_id else self.motor_id
        self.rx_id = self.motor_id

    def _request(self, data: bytes, reply_cmd: Optional[int] = None, timeout: Optional[float] = None) -> bytes:
        if reply_cmd is None and data:
            reply_cmd = data[0]
        frame = self.bus.request_frame(
            can_id=self.tx_id,
            data=data,
            reply_can_id=self.rx_id,
            reply_cmd=reply_cmd,
            timeout=timeout,
        )
        return frame.data

    # ---- 读取 ----

    def read_version(self) -> MotorVersion:
        d = self._request(b"\xA0", 0xA0)
        if len(d) < 8:
            raise MotorProtocolError(f"A0 回复长度异常: {format_hex(d)}")
        return MotorVersion(read_u16_le(d, 1), read_u16_le(d, 3), read_u16_le(d, 5), d[7])

    def read_q_current_a(self) -> float:
        d = self._request(b"\xA1", 0xA1)
        if len(d) < 5:
            raise MotorProtocolError(f"A1 回复长度异常: {format_hex(d)}")
        return read_s32_le(d, 1) * 0.001

    def read_speed_rpm(self) -> float:
        d = self._request(b"\xA2", 0xA2)
        if len(d) < 5:
            raise MotorProtocolError(f"A2 回复长度异常: {format_hex(d)}")
        return read_s32_le(d, 1) * 0.01

    def read_angle(self) -> MotorAngle:
        d = self._request(b"\xA3", 0xA3)
        if len(d) < 7:
            raise MotorProtocolError(f"A3 回复长度异常: {format_hex(d)}")
        return MotorAngle(read_u16_le(d, 1), read_s32_le(d, 3))

    def read_quick_feedback(self) -> MotorQuickFeedback:
        d = self._request(b"\xA4", 0xA4)
        if len(d) < 8:
            raise MotorProtocolError(f"A4 回复长度异常: {format_hex(d)}")
        return MotorQuickFeedback(
            temperature_c=d[1],
            q_current_a=read_s16_le(d, 2) * 0.001,
            speed_rpm=read_s16_le(d, 4) * 0.01,
            single_turn_deg=read_u16_le(d, 6) * 360.0 / 16384.0,
        )

    def read_status(self) -> MotorStatus:
        d = self._request(b"\xAE", 0xAE)
        if len(d) < 8:
            raise MotorProtocolError(f"AE 回复长度异常: {format_hex(d)}")
        return MotorStatus(
            bus_voltage_v=read_u16_le(d, 1) * 0.01,
            bus_current_a=read_u16_le(d, 3) * 0.01,
            temperature_c=d[5],
            mode=d[6],
            fault=d[7],
        )

    def clear_fault(self) -> int:
        d = self._request(b"\xAF", 0xAF)
        if len(d) < 2:
            raise MotorProtocolError(f"AF 回复长度异常: {format_hex(d)}")
        return d[1]

    # ---- 参数配置 ----

    def set_zero(self) -> MotorAngle:
        """
        B1：设置当前为硬件零点。危险，会写入驱动器。
        """
        d = self._request(b"\xB1", 0xB1, timeout=0.6)
        if len(d) < 3:
            raise MotorProtocolError(f"B1 回复长度异常: {format_hex(d)}")
        time.sleep(0.05)
        return self.read_angle()

    def set_position_max_speed_rpm(self, rpm: float) -> float:
        raw = max(0, int(round(float(rpm) * 100.0)))
        d = self._request(b"\xB2" + u32_le(raw), 0xB2)
        if len(d) < 5:
            raise MotorProtocolError(f"B2 回复长度异常: {format_hex(d)}")
        return read_u32_le(d, 1) * 0.01

    def set_max_q_current_a(self, current_a: float) -> float:
        raw = max(0, int(round(float(current_a) * 1000.0)))
        d = self._request(b"\xB3" + u32_le(raw), 0xB3)
        if len(d) < 5:
            raise MotorProtocolError(f"B3 回复长度异常: {format_hex(d)}")
        return read_u32_le(d, 1) * 0.001

    def set_current_slope_a_per_s(self, slope: float) -> float:
        raw = max(0, int(round(float(slope) * 1000.0)))
        d = self._request(b"\xB4" + u32_le(raw), 0xB4)
        if len(d) < 5:
            raise MotorProtocolError(f"B4 回复长度异常: {format_hex(d)}")
        return read_u32_le(d, 1) * 0.001

    def set_speed_accel_rpm_s(self, accel: float) -> float:
        raw = max(0, int(round(float(accel) * 100.0)))
        d = self._request(b"\xB5" + u32_le(raw), 0xB5)
        if len(d) < 5:
            raise MotorProtocolError(f"B5 回复长度异常: {format_hex(d)}")
        return read_u32_le(d, 1) * 0.01

    def read_or_set_pid_float(self, cmd: int, value: Optional[float] = None) -> float:
        if cmd not in (0xB6, 0xB7, 0xB8, 0xB9):
            raise ValueError("PID 命令只能是 B6/B7/B8/B9。")
        payload = bytes([cmd]) if value is None else bytes([cmd]) + float32_le(float(value))
        d = self._request(payload, cmd)
        if len(d) < 5:
            raise MotorProtocolError(f"{cmd:02X} 回复长度异常: {format_hex(d)}")
        return read_float32_le(d, 1)

    # ---- 控制 ----

    def set_q_current_a(self, current_a: float) -> float:
        raw = int(round(float(current_a) * 1000.0))
        d = self._request(b"\xC0" + s32_le(raw), 0xC0)
        if len(d) < 5:
            raise MotorProtocolError(f"C0 回复长度异常: {format_hex(d)}")
        return read_s32_le(d, 1) * 0.001

    def set_speed_rpm(self, rpm: float) -> float:
        raw = int(round(float(rpm) * 100.0))
        d = self._request(b"\xC1" + s32_le(raw), 0xC1, timeout=0.8)
        if len(d) < 5:
            raise MotorProtocolError(f"C1 回复长度异常: {format_hex(d)}")
        return read_s32_le(d, 1) * 0.01

    def stop_speed(self) -> float:
        return self.set_speed_rpm(0.0)

    def move_absolute_count(self, count: int) -> MotorAngle:
        d = self._request(b"\xC2" + s32_le(int(count)), 0xC2, timeout=0.8)
        if len(d) < 7:
            raise MotorProtocolError(f"C2 回复长度异常: {format_hex(d)}")
        return MotorAngle(read_u16_le(d, 1), read_s32_le(d, 3))

    def move_relative_count(self, count: int) -> MotorAngle:
        d = self._request(b"\xC3" + s32_le(int(count)), 0xC3, timeout=0.8)
        if len(d) < 7:
            raise MotorProtocolError(f"C3 回复长度异常: {format_hex(d)}")
        return MotorAngle(read_u16_le(d, 1), read_s32_le(d, 3))

    def return_origin_shortest(self) -> MotorAngle:
        d = self._request(b"\xC4", 0xC4, timeout=0.8)
        if len(d) < 7:
            raise MotorProtocolError(f"C4 回复长度异常: {format_hex(d)}")
        return MotorAngle(read_u16_le(d, 1), read_s32_le(d, 3))

    def read_brake(self) -> int:
        d = self._request(b"\xCE\xFF", 0xCE)
        if len(d) < 2:
            raise MotorProtocolError(f"CE 回复长度异常: {format_hex(d)}")
        return d[1]

    def set_brake(self, closed: bool) -> int:
        d = self._request(b"\xCE" + bytes([0x01 if closed else 0x00]), 0xCE)
        if len(d) < 2:
            raise MotorProtocolError(f"CE 回复长度异常: {format_hex(d)}")
        return d[1]

    def disable(self) -> MotorStatus:
        d = self._request(b"\xCF", 0xCF)
        if len(d) < 8:
            raise MotorProtocolError(f"CF 回复长度异常: {format_hex(d)}")
        return MotorStatus(
            bus_voltage_v=read_u16_le(d, 1) * 0.01,
            bus_current_a=read_u16_le(d, 3) * 0.01,
            temperature_c=d[5],
            mode=d[6],
            fault=d[7],
        )


# ============================================================
# 上层：单臂
# ============================================================

@dataclass
class JointConfig:
    key: str
    motor_id: int
    name: str = ""
    ratio: float = 1.0
    direction: float = 1.0
    min_deg: float = -180.0
    max_deg: float = 180.0
    default_speed_rpm: float = 5.0
    default_accel_rpm_s: float = 10.0
    current_deg: float = 0.0

    def __post_init__(self) -> None:
        if not self.name:
            self.name = self.key
        if self.ratio <= 0:
            raise ValueError(f"{self.key}: ratio 必须 >0。")
        if self.direction not in (1, -1, 1.0, -1.0):
            raise ValueError(f"{self.key}: direction 建议使用 1 或 -1。")


@dataclass
class JointFeedback:
    key: str
    motor_id: int
    angle_deg: Optional[float] = None
    speed_rpm: Optional[float] = None
    current_a: Optional[float] = None
    temperature_c: Optional[int] = None
    voltage_v: Optional[float] = None
    mode: Optional[int] = None
    fault: Optional[int] = None
    error: Optional[str] = None


class CanArm:
    """一条机械臂，绑定一路 CAN 通道。"""

    def __init__(self, bus: CanChannel, joints: Sequence[JointConfig], name: str = "arm"):
        self.bus = bus
        self.name = name
        self.joints: Dict[str, JointConfig] = {j.key: j for j in joints}
        self.motors: Dict[str, CanMotor] = {
            j.key: CanMotor(bus, j.motor_id) for j in joints
        }

    def joint_keys(self) -> List[str]:
        return list(self.joints.keys())

    def get_joint(self, key: str) -> JointConfig:
        if key not in self.joints:
            raise KeyError(f"{self.name}: 不存在关节 {key}，可用：{self.joint_keys()}")
        return self.joints[key]

    def get_motor(self, key: str) -> CanMotor:
        if key not in self.motors:
            raise KeyError(f"{self.name}: 不存在关节 {key}")
        return self.motors[key]

    def logical_deg_to_count(self, cfg: JointConfig, logical_deg: float) -> int:
        physical_deg = float(logical_deg) * cfg.direction
        motor_deg = physical_deg * cfg.ratio
        return int(round(motor_deg / 360.0 * 16384.0))

    def count_to_logical_deg(self, cfg: JointConfig, count: int) -> float:
        motor_deg = int(count) * 360.0 / 16384.0
        physical_deg = motor_deg / cfg.ratio
        return physical_deg * cfg.direction

    def check_limit(self, cfg: JointConfig, deg: float) -> None:
        if not cfg.min_deg <= float(deg) <= cfg.max_deg:
            raise JointLimitError(
                f"{self.name}.{cfg.key} 目标角 {deg:.2f}° 超限 "
                f"[{cfg.min_deg:.2f}, {cfg.max_deg:.2f}]°"
            )

    # ---- 读取 ----

    def read_joint_feedback(self, key: str, with_status: bool = True) -> JointFeedback:
        cfg = self.get_joint(key)
        motor = self.get_motor(key)
        fb = JointFeedback(key=key, motor_id=cfg.motor_id)

        try:
            angle = motor.read_angle()
            speed = motor.read_speed_rpm()
            current = motor.read_q_current_a()

            fb.angle_deg = self.count_to_logical_deg(cfg, angle.multi_turn_count)
            fb.speed_rpm = speed / cfg.ratio * cfg.direction
            fb.current_a = current
            cfg.current_deg = fb.angle_deg

            if with_status:
                st = motor.read_status()
                fb.temperature_c = st.temperature_c
                fb.voltage_v = st.bus_voltage_v
                fb.mode = st.mode
                fb.fault = st.fault
        except Exception as exc:
            fb.error = str(exc)

        return fb

    def read_all_feedback(self, with_status: bool = True) -> Dict[str, JointFeedback]:
        return {key: self.read_joint_feedback(key, with_status=with_status) for key in self.joint_keys()}

    def read_all_status(self) -> Dict[str, Union[MotorStatus, str]]:
        out: Dict[str, Union[MotorStatus, str]] = {}
        for key in self.joint_keys():
            try:
                out[key] = self.get_motor(key).read_status()
            except Exception as exc:
                out[key] = str(exc)
        return out

    def read_all_versions(self) -> Dict[str, Union[MotorVersion, str]]:
        out: Dict[str, Union[MotorVersion, str]] = {}
        for key in self.joint_keys():
            try:
                out[key] = self.get_motor(key).read_version()
            except Exception as exc:
                out[key] = str(exc)
        return out

    # ---- 参数与安全 ----

    def clear_faults(self) -> Dict[str, Union[int, str]]:
        out: Dict[str, Union[int, str]] = {}
        for key in self.joint_keys():
            try:
                out[key] = self.get_motor(key).clear_fault()
            except Exception as exc:
                out[key] = str(exc)
            time.sleep(0.005)
        return out

    def configure_defaults(self) -> None:
        for key, cfg in self.joints.items():
            self.set_joint_speed_limit(key, cfg.default_speed_rpm)
            time.sleep(0.005)
            self.set_joint_accel(key, cfg.default_accel_rpm_s)
            time.sleep(0.005)

    def set_joint_speed_limit(self, key: str, joint_speed_rpm: float) -> float:
        cfg = self.get_joint(key)
        motor_speed = abs(float(joint_speed_rpm) * cfg.ratio)
        actual_motor = self.get_motor(key).set_position_max_speed_rpm(motor_speed)
        cfg.default_speed_rpm = abs(float(joint_speed_rpm))
        return actual_motor / cfg.ratio

    def set_joint_accel(self, key: str, joint_accel_rpm_s: float) -> float:
        cfg = self.get_joint(key)
        motor_accel = abs(float(joint_accel_rpm_s) * cfg.ratio)
        actual_motor = self.get_motor(key).set_speed_accel_rpm_s(motor_accel)
        cfg.default_accel_rpm_s = abs(float(joint_accel_rpm_s))
        return actual_motor / cfg.ratio

    def set_joint_current_limit(self, key: str, joint_current_a: float) -> float:
        return self.get_motor(key).set_max_q_current_a(float(joint_current_a))

    def set_zero(self, key: str) -> MotorAngle:
        return self.get_motor(key).set_zero()

    # ---- 运动 ----

    def move_absolute(self, key: str, target_deg: float) -> MotorAngle:
        cfg = self.get_joint(key)
        self.check_limit(cfg, target_deg)
        count = self.logical_deg_to_count(cfg, target_deg)
        ret = self.get_motor(key).move_absolute_count(count)
        cfg.current_deg = float(target_deg)
        return ret

    def move_relative(self, key: str, delta_deg: float) -> MotorAngle:
        cfg = self.get_joint(key)
        target = cfg.current_deg + float(delta_deg)
        return self.move_absolute(key, target)

    def move_many_absolute(self, targets: Dict[str, float], gap_s: float = 0.01) -> Dict[str, Union[MotorAngle, str]]:
        result: Dict[str, Union[MotorAngle, str]] = {}
        for key, deg in targets.items():
            try:
                result[key] = self.move_absolute(key, deg)
            except Exception as exc:
                result[key] = str(exc)
            time.sleep(gap_s)
        return result

    def home(self, gap_s: float = 0.02) -> Dict[str, Union[MotorAngle, str]]:
        return self.move_many_absolute({key: 0.0 for key in self.joint_keys()}, gap_s=gap_s)

    def speed_test(self, key: str, rpm: float, duration_s: float) -> None:
        motor = self.get_motor(key)
        motor.set_speed_rpm(float(rpm))
        time.sleep(max(0.0, float(duration_s)))
        motor.stop_speed()

    def stop_all_speed(self) -> None:
        for key in self.joint_keys():
            try:
                self.get_motor(key).stop_speed()
            except Exception:
                pass
            time.sleep(0.003)

    def disable_all(self) -> None:
        for key in self.joint_keys():
            try:
                self.get_motor(key).disable()
            except Exception:
                pass
            time.sleep(0.003)


# ============================================================
# 双 CAN 双臂系统
# ============================================================

class DualCanDualArmSystem:
    """
    ARM0：右臂，默认 CAN1
    ARM1：左臂，默认 CAN2
    """

    def __init__(
        self,
        port: str,
        right_arm_config: Sequence[JointConfig],
        left_arm_config: Sequence[JointConfig],
        baudrate: int = 1_000_000,
        right_channel: int = 1,
        left_channel: int = 2,
        debug: bool = False,
    ):
        self.bus = SerialDualCanNormalBus(port=port, baudrate=baudrate, debug=debug)
        self.right_channel = int(right_channel)
        self.left_channel = int(left_channel)
        self.right_arm = CanArm(self.bus.channel(self.right_channel), right_arm_config, name="right_arm")
        self.left_arm = CanArm(self.bus.channel(self.left_channel), left_arm_config, name="left_arm")
        self.arms: Dict[int, CanArm] = {
            0: self.right_arm,
            1: self.left_arm,
        }

    def connect(self) -> None:
        self.bus.connect()

    def close(self) -> None:
        self.bus.close()

    def __enter__(self) -> "DualCanDualArmSystem":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def get_arm(self, arm_id: int) -> CanArm:
        arm_id = int(arm_id)
        if arm_id not in self.arms:
            raise KeyError("arm_id 只能是 0=右臂 或 1=左臂")
        return self.arms[arm_id]

    def arm_channel(self, arm_id: int) -> int:
        return self.right_channel if int(arm_id) == 0 else self.left_channel

    def clear_faults_all(self) -> None:
        self.right_arm.clear_faults()
        self.left_arm.clear_faults()

    def configure_defaults_all(self) -> None:
        self.right_arm.configure_defaults()
        self.left_arm.configure_defaults()

    def stop_all_speed(self) -> None:
        self.right_arm.stop_all_speed()
        self.left_arm.stop_all_speed()

    def disable_all(self) -> None:
        self.right_arm.disable_all()
        self.left_arm.disable_all()


# ============================================================
# 默认机械臂配置
# ============================================================

def get_default_right_arm_config() -> List[JointConfig]:
    return [
        JointConfig("j1", 35, "J1_Shoulder_Pitch", ratio=3.0, direction=1.0, min_deg=-110.0, max_deg=110.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j2", 34, "J2_Shoulder_Yaw",   ratio=3.0, direction=1.0, min_deg=-170.0, max_deg=120.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j3", 31, "J3_Shoulder_Roll",  ratio=4.0, direction=1.0, min_deg=-180.0, max_deg=180.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j4", 32, "J4_Elbow_Pitch",    ratio=4.2, direction=1.0, min_deg=-90.0,  max_deg=150.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j5", 33, "J5_Wrist_Roll",     ratio=1.0, direction=1.0, min_deg=-120.0, max_deg=120.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
    ]


def get_default_left_arm_config() -> List[JointConfig]:
    return [
        JointConfig("j1", 55, "J1_Shoulder_Pitch", ratio=3.0, direction=-1.0, min_deg=-110.0, max_deg=110.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j2", 1,  "J2_Shoulder_Yaw",   ratio=3.0, direction=-1.0, min_deg=-170.0, max_deg=120.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j3", 15, "J3_Shoulder_Roll",  ratio=4.0, direction=-1.0, min_deg=-180.0, max_deg=180.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j4", 18, "J4_Elbow_Pitch",    ratio=4.2, direction=-1.0, min_deg=-90.0,  max_deg=150.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
        JointConfig("j5", 23, "J5_Wrist_Roll",     ratio=1.0, direction=-1.0, min_deg=-120.0, max_deg=120.0, default_speed_rpm=5.0, default_accel_rpm_s=10.0),
    ]
