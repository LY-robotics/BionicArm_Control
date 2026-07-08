"""
CAN电机 + 5自由度机械臂控制库，适用于SANPO USB2CAN板卡。

官方Demo测试的协议格式：
    Advanced USB数据包 = b'AT' + frame_id(4字节) + dlc(1字节) + data + b'\r\n'

电机协议摘要：
    主机发送标准CAN ID: 0x100 | motor_id
    电机回复标准CAN ID: motor_id

跨平台支持：
    Windows: COM17 / COMx
    Ubuntu : /dev/ttyACM0 或 /dev/ttyUSB0

依赖安装：
    pip install pyserial
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("Please install pyserial first: pip install pyserial") from exc


# -----------------------------
# USB2CAN传输层
# -----------------------------

HEADER_ADVANCED = bytes([0x41, 0x54])  # 'A' 'T' 高级模式包头
TAIL = bytes([0x0D, 0x0A])             # CR LF 包尾


def fmt_hex(data: bytes) -> str:
    """将字节数据转换为十六进制字符串格式"""
    return " ".join(f"{b:02X}" for b in data) if data else ""


def list_serial_ports() -> List[str]:
    """列出可用的串口，便于在Windows和Ubuntu之间切换使用"""
    return [p.device for p in list_ports.comports()]


@dataclass
class CanFrame:
    """CAN帧数据结构"""
    can_id: int            # CAN标识符
    data: bytes            # 数据负载
    is_extended: bool = False  # 是否为扩展帧(29位ID)
    is_remote: bool = False    # 是否为远程帧

    def __str__(self) -> str:
        kind = "EXT" if self.is_extended else "STD"
        return f"{kind} ID=0x{self.can_id:X} DLC={len(self.data)} DATA={fmt_hex(self.data)}"


class SerialUsbCanTransport:
    """
    SANPO USB2CAN串口传输类。

    该类仅负责通过USB2CAN板卡发送/接收CAN帧，不了解电机协议的具体含义。
    保持独立性便于未来迁移到其他硬件。
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 1_000_000,
        timeout: float = 0.05,
        debug: bool = False,
    ) -> None:
        self.port = port          # 串口号
        self.baudrate = baudrate  # 波特率(默认1Mbps)
        self.timeout = timeout    # 串口超时时间
        self.debug = debug        # 调试模式
        self.ser: Optional[serial.Serial] = None  # 串口对象
        self._lock = threading.Lock()             # 线程锁

    def connect(self) -> None:
        """连接到USB2CAN板卡并切换到高级模式"""
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.set_advanced_mode()

    def close(self) -> None:
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "SerialUsbCanTransport":
        """支持上下文管理器进入"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """支持上下文管理器退出时自动关闭"""
        self.close()

    def _require_serial(self) -> serial.Serial:
        """确保串口已打开，否则抛出异常"""
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口未打开，请先调用connect()")
        return self.ser

    @staticmethod
    def build_std_frame_identifier(can_id: int, is_remote: bool = False) -> bytes:
        """构建标准CAN帧标识符(4字节大端格式)"""
        if not 0 <= can_id <= 0x7FF:
            raise ValueError("标准CAN ID必须在0..0x7FF范围内")
        value = (can_id & 0x7FF) << 21
        if is_remote:
            value |= 1 << 1
        return value.to_bytes(4, byteorder="big", signed=False)

    @staticmethod
    def parse_frame_identifier(frame_id: bytes) -> Tuple[int, bool, bool]:
        """解析帧标识符，返回(can_id, 是否扩展帧, 是否远程帧)"""
        value = int.from_bytes(frame_id, byteorder="big", signed=False)
        is_extended = bool((value >> 2) & 0x1)
        is_remote = bool((value >> 1) & 0x1)
        if is_extended:
            can_id = (value >> 3) & 0x1FFFFFFF
        else:
            can_id = (value >> 21) & 0x7FF
        return can_id, is_extended, is_remote

    def set_advanced_mode(self) -> bytes:
        """将USB2CAN板卡切换到高级模式"""
        ser = self._require_serial()
        ser.reset_input_buffer()
        ser.write(b"AT+AT\r\n")
        ser.flush()
        time.sleep(0.1)
        resp = ser.read_all()
        if self.debug:
            print("模式切换响应:", resp)
        if b"OK" not in resp:
            # 某些固件可能已经处于高级模式，不一定每次都回复OK
            # 此处不强制失败，第一个CAN请求会验证是否工作正常
            if self.debug:
                print("警告: AT+AT未返回OK")
        return resp

    def build_advanced_packet(self, can_id: int, data: bytes) -> bytes:
        """构建高级模式USB数据包"""
        if not 0 <= len(data) <= 8:
            raise ValueError("CAN数据长度必须在0..8字节之间")
        frame_id = self.build_std_frame_identifier(can_id)
        return HEADER_ADVANCED + frame_id + bytes([len(data)]) + data + TAIL

    def send_std(self, can_id: int, data: bytes, flush_rx: bool = True) -> None:
        """发送一个标准CAN数据帧"""
        ser = self._require_serial()
        packet = self.build_advanced_packet(can_id, data)
        if flush_rx:
            ser.reset_input_buffer()
        if self.debug:
            print(f"发送USB: {fmt_hex(packet)}")
            print(f"发送CAN: STD ID=0x{can_id:X} DLC={len(data)} DATA={fmt_hex(data)}")
        ser.write(packet)
        ser.flush()

    def read_frames(self, timeout: float = 0.2) -> List[CanFrame]:
        """读取超时时间内收到的所有高级模式CAN帧"""
        ser = self._require_serial()
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            chunk = ser.read(waiting or 1)
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.002)
        frames = self._parse_advanced_frames(bytes(buf))
        if self.debug:
            if buf:
                print(f"接收USB: {fmt_hex(bytes(buf))}")
            for fr in frames:
                print(f"接收CAN: {fr}")
        return frames

    @staticmethod
    def _parse_advanced_frames(raw: bytes) -> List[CanFrame]:
        """解析原始字节流中的CAN帧"""
        frames: List[CanFrame] = []
        i = 0
        n = len(raw)
        while i < n:
            start = raw.find(HEADER_ADVANCED, i)
            if start < 0:
                break
            # 至少需要包头+帧ID+数据长度字节
            if start + 2 + 4 + 1 > n:
                break
            frame_id = raw[start + 2 : start + 6]
            dlc = raw[start + 6]
            end_data = start + 7 + dlc
            end_tail = end_data + 2
            if dlc > 8 or end_tail > n:
                i = start + 1
                continue
            if raw[end_data:end_tail] != TAIL:
                i = start + 1
                continue
            data = raw[start + 7 : end_data]
            can_id, is_ext, is_remote = SerialUsbCanTransport.parse_frame_identifier(frame_id)
            frames.append(CanFrame(can_id=can_id, data=data, is_extended=is_ext, is_remote=is_remote))
            i = end_tail
        return frames

    def request(
        self,
        can_id: int,
        data: bytes,
        expect_reply_id: Optional[int] = None,
        expect_cmd: Optional[int] = None,
        timeout: float = 0.25,
    ) -> Optional[CanFrame]:
        """
        发送CAN帧并返回第一个匹配的回复。
        如果未指定过滤条件，返回收到的第一个帧。
        """
        with self._lock:
            self.send_std(can_id, data, flush_rx=True)
            frames = self.read_frames(timeout=timeout)

        for fr in frames:
            if expect_reply_id is not None and fr.can_id != expect_reply_id:
                continue
            if expect_cmd is not None:
                if not fr.data or fr.data[0] != expect_cmd:
                    continue
            return fr
        return frames[0] if frames and expect_reply_id is None and expect_cmd is None else None


# -----------------------------
# 底层电机协议层
# -----------------------------

MODE_TEXT = {
    0: "禁用/自由",
    1: "电压模式",
    2: "q轴电流模式",
    3: "速度模式",
    4: "位置模式",
}

FAULT_BITS = {
    0: "电压故障",
    1: "电流故障",
    2: "温度故障",
    3: "编码器故障",
    6: "硬件故障",
    7: "软件故障",
}


def _u16_le(data: bytes) -> int:
    """解析小端序无符号16位整数"""
    return int.from_bytes(data, byteorder="little", signed=False)


def _s16_le(data: bytes) -> int:
    """解析小端序有符号16位整数"""
    return int.from_bytes(data, byteorder="little", signed=True)


def _u32_le(data: bytes) -> int:
    """解析小端序无符号32位整数"""
    return int.from_bytes(data, byteorder="little", signed=False)


def _s32_le(data: bytes) -> int:
    """解析小端序有符号32位整数"""
    return int.from_bytes(data, byteorder="little", signed=True)


def _f32_le(data: bytes) -> float:
    """解析小端序32位浮点数"""
    return struct.unpack("<f", data)[0]


def _pack_s32(value: int) -> bytes:
    """打包为小端序有符号32位整数"""
    return int(value).to_bytes(4, byteorder="little", signed=True)


def _pack_u32(value: int) -> bytes:
    """打包为小端序无符号32位整数"""
    return int(value).to_bytes(4, byteorder="little", signed=False)


def _pack_f32(value: float) -> bytes:
    """打包为小端序32位浮点数"""
    return struct.pack("<f", float(value))


def rpm_to_raw(rpm: float) -> int:
    """将RPM转换为原始值(单位: 0.01 RPM)"""
    return int(round(rpm * 100.0))


def raw_to_rpm(raw: int) -> float:
    """将原始值转换为RPM"""
    return raw * 0.01


def amp_to_raw(amp: float) -> int:
    """将安培转换为原始值(单位: 0.001 A)"""
    return int(round(amp * 1000.0))


def raw_to_amp(raw: int) -> float:
    """将原始值转换为安培"""
    return raw * 0.001


def counts_to_deg(counts: int) -> float:
    """将编码器计数转换为角度(编码器分辨率16384)"""
    return counts * 360.0 / 16384.0


def deg_to_counts(deg: float) -> int:
    """将角度转换为编码器计数"""
    return int(round(deg * 16384.0 / 360.0))


def decode_faults(fault_code: int) -> List[str]:
    """解码故障码，返回故障名称列表"""
    return [name for bit, name in FAULT_BITS.items() if fault_code & (1 << bit)]


class CanMotor:
    """
    单电机控制类，一个电机对应一个CAN ID。

    motor_id应为电机驱动器的真实地址，例如十进制34 == 0x22。
    默认情况下，该类发送命令到 0x100 | motor_id，并期望从 motor_id 接收回复。
    """

    def __init__(
        self,
        bus: SerialUsbCanTransport,
        motor_id: int,
        name: str = "motor",
        use_host_id_offset: bool = True,
        response_timeout: float = 0.25,
    ) -> None:
        if not 0 <= motor_id <= 0xFF:
            raise ValueError("motor_id必须在0..255范围内")
        self.bus = bus                    # CAN总线传输对象
        self.motor_id = int(motor_id)     # 电机ID
        self.name = name                  # 电机名称
        self.use_host_id_offset = use_host_id_offset  # 是否使用主机ID偏移
        self.response_timeout = response_timeout      # 响应超时时间

    @property
    def tx_can_id(self) -> int:
        """发送命令时使用的CAN ID"""
        return (0x100 | self.motor_id) if self.use_host_id_offset else self.motor_id

    def _send(self, data: bytes, expect_reply: bool = True, timeout: Optional[float] = None) -> Optional[CanFrame]:
        """发送电机命令，内部方法"""
        if not data:
            raise ValueError("电机命令数据不能为空")
        if expect_reply:
            return self.bus.request(
                self.tx_can_id,
                data,
                expect_reply_id=self.motor_id,
                expect_cmd=data[0],
                timeout=self.response_timeout if timeout is None else timeout,
            )
        self.bus.send_std(self.tx_can_id, data)
        return None

    def _cmd1(self, cmd: int, expect_reply: bool = True) -> Optional[CanFrame]:
        """发送单字节命令"""
        return self._send(bytes([cmd]), expect_reply=expect_reply)

    def _cmd_s32(self, cmd: int, value: int, expect_reply: bool = True) -> Optional[CanFrame]:
        """发送带32位有符号整数参数的命令"""
        return self._send(bytes([cmd]) + _pack_s32(value), expect_reply=expect_reply)

    def _cmd_u32(self, cmd: int, value: int, expect_reply: bool = True) -> Optional[CanFrame]:
        """发送带32位无符号整数参数的命令"""
        return self._send(bytes([cmd]) + _pack_u32(value), expect_reply=expect_reply)

    def _cmd_f32(self, cmd: int, value: float, expect_reply: bool = True) -> Optional[CanFrame]:
        """发送带32位浮点数参数的命令"""
        return self._send(bytes([cmd]) + _pack_f32(value), expect_reply=expect_reply)

    # ---- 系统/读取命令 ----

    def read_version(self) -> Optional[Dict[str, int]]:
        """读取电机版本信息(A0)"""
        fr = self._cmd1(0xA0)
        if not fr or len(fr.data) < 8:
            return None
        d = fr.data
        return {
            "boot_version_raw": _u16_le(d[1:3]),     # 引导程序版本
            "app_version_raw": _u16_le(d[3:5]),      # 应用程序版本
            "hardware_version_raw": _u16_le(d[5:7]), # 硬件版本
            "can_protocol_version_raw": d[7],        # CAN协议版本
        }

    def read_q_current(self) -> Optional[float]:
        """读取Q轴电流(A1)"""
        fr = self._cmd1(0xA1)
        if not fr or len(fr.data) < 5:
            return None
        return raw_to_amp(_s32_le(fr.data[1:5]))

    def read_speed_rpm(self) -> Optional[float]:
        """读取转速(A2)"""
        fr = self._cmd1(0xA2)
        if not fr or len(fr.data) < 5:
            return None
        return raw_to_rpm(_s32_le(fr.data[1:5]))

    def read_angle(self) -> Optional[Dict[str, float]]:
        """读取角度(A3)"""
        fr = self._cmd1(0xA3)
        if not fr or len(fr.data) < 7:
            return None
        single_count = _u16_le(fr.data[1:3])   # 单圈计数
        multi_count = _s32_le(fr.data[3:7])    # 多圈计数
        return {
            "single_count": single_count,
            "single_deg": counts_to_deg(single_count),
            "multi_count": multi_count,
            "multi_deg": counts_to_deg(multi_count),
        }

    def read_compact(self) -> Optional[Dict[str, float]]:
        """读取紧凑状态(A4): 温度、Q轴电流、速度、单圈角度"""
        fr = self._cmd1(0xA4)
        if not fr or len(fr.data) < 8:
            return None
        d = fr.data
        single_count = _u16_le(d[6:8])
        return {
            "temperature_c": float(d[1]),
            "q_current_a": raw_to_amp(_s16_le(d[2:4])),
            "speed_rpm": raw_to_rpm(_s16_le(d[4:6])),
            "single_count": single_count,
            "single_deg": counts_to_deg(single_count),
        }

    def read_status(self) -> Optional[Dict[str, Union[float, int, str, List[str]]]]:
        """读取电机状态(AE)"""
        fr = self._cmd1(0xAE)
        if not fr or len(fr.data) < 8:
            return None
        d = fr.data
        mode = d[6]
        fault = d[7]
        return {
            "bus_voltage_v": _u16_le(d[1:3]) * 0.01,  # 母线电压(V)
            "bus_current_a": _u16_le(d[3:5]) * 0.01,  # 母线电流(A)
            "temperature_c": d[5],                     # 温度(℃)
            "mode": mode,                              # 当前模式
            "mode_text": MODE_TEXT.get(mode, f"未知({mode})"),
            "fault_code": fault,                       # 故障码
            "faults": decode_faults(fault),            # 故障列表
        }

    def clear_fault(self) -> Optional[int]:
        """清除故障(AF)"""
        fr = self._cmd1(0xAF)
        if not fr or len(fr.data) < 2:
            return None
        return fr.data[1]

    def read_motor_params(self) -> Optional[Dict[str, Union[int, float]]]:
        """读取电机参数(B0)"""
        fr = self._cmd1(0xB0)
        if not fr or len(fr.data) < 7:
            return None
        d = fr.data
        return {
            "pole_pairs": d[1],                    # 极对数
            "torque_constant_n_per_a": _f32_le(d[2:6]),  # 扭矩常数(N/A)
            "reduction_ratio_raw": d[6],           # 减速比
        }

    # ---- 参数设置命令 ----

    def set_zero(self) -> Optional[Dict[str, float]]:
        """设置当前位置为零点(B1)"""
        fr = self._cmd1(0xB1, expect_reply=True)
        if not fr or len(fr.data) < 3:
            return None
        offset = _u16_le(fr.data[1:3])
        return {"mechanical_offset_count": offset, "mechanical_offset_deg": counts_to_deg(offset)}

    def set_position_max_speed(self, rpm: float) -> bool:
        """设置位置模式最大速度(B2)"""
        return self._cmd_u32(0xB2, rpm_to_raw(abs(rpm))) is not None

    def set_max_q_current(self, amp: float) -> bool:
        """设置最大Q轴电流(B3)"""
        return self._cmd_u32(0xB3, amp_to_raw(abs(amp))) is not None

    def set_q_current_slope(self, amp_per_sec: float) -> bool:
        """设置Q轴电流斜率(B4)"""
        return self._cmd_u32(0xB4, int(round(abs(amp_per_sec) * 1000.0))) is not None

    def set_speed_acceleration(self, rpm_per_sec: float) -> bool:
        """设置速度加速度(B5)"""
        return self._cmd_u32(0xB5, int(round(abs(rpm_per_sec) * 100.0))) is not None

    def read_or_set_position_kp(self, value: Optional[float] = None) -> Optional[float]:
        """读取或设置位置环比例系数Kp(B6)"""
        fr = self._cmd1(0xB6) if value is None else self._cmd_f32(0xB6, value)
        if not fr or len(fr.data) < 5:
            return None
        return _f32_le(fr.data[1:5])

    def read_or_set_position_ki(self, value: Optional[float] = None) -> Optional[float]:
        """读取或设置位置环积分系数Ki(B7)"""
        fr = self._cmd1(0xB7) if value is None else self._cmd_f32(0xB7, value)
        if not fr or len(fr.data) < 5:
            return None
        return _f32_le(fr.data[1:5])

    def read_or_set_speed_kp(self, value: Optional[float] = None) -> Optional[float]:
        """读取或设置速度环比例系数Kp(B8)"""
        fr = self._cmd1(0xB8) if value is None else self._cmd_f32(0xB8, value)
        if not fr or len(fr.data) < 5:
            return None
        return _f32_le(fr.data[1:5])

    def read_or_set_speed_ki(self, value: Optional[float] = None) -> Optional[float]:
        """读取或设置速度环积分系数Ki(B9)"""
        fr = self._cmd1(0xB9) if value is None else self._cmd_f32(0xB9, value)
        if not fr or len(fr.data) < 5:
            return None
        return _f32_le(fr.data[1:5])

    # ---- 控制命令 ----

    def set_q_current(self, amp: float) -> Optional[float]:
        """设置Q轴电流(C0)"""
        fr = self._cmd_s32(0xC0, amp_to_raw(amp))
        if not fr or len(fr.data) < 5:
            return None
        return raw_to_amp(_s32_le(fr.data[1:5]))

    def set_speed(self, rpm: float) -> Optional[float]:
        """设置速度模式目标值(C1)，如果有回复则返回反馈速度"""
        fr = self._cmd_s32(0xC1, rpm_to_raw(rpm))
        if not fr or len(fr.data) < 5:
            return None
        return raw_to_rpm(_s32_le(fr.data[1:5]))

    def move_absolute_counts(self, counts: int) -> Optional[Dict[str, float]]:
        """绝对位置移动(以编码器计数为单位)(C2)"""
        fr = self._cmd_s32(0xC2, counts)
        if not fr or len(fr.data) < 7:
            return None
        single_count = _u16_le(fr.data[1:3])
        multi_count = _s32_le(fr.data[3:7])
        return {
            "single_count": single_count,
            "single_deg": counts_to_deg(single_count),
            "multi_count": multi_count,
            "multi_deg": counts_to_deg(multi_count),
        }

    def move_absolute_motor_deg(self, motor_deg: float) -> Optional[Dict[str, float]]:
        """绝对位置移动(以电机角度为单位)"""
        return self.move_absolute_counts(deg_to_counts(motor_deg))

    def move_relative_counts(self, counts: int) -> Optional[Dict[str, float]]:
        """相对位置移动(以编码器计数为单位)(C3)"""
        fr = self._cmd_s32(0xC3, counts)
        if not fr or len(fr.data) < 7:
            return None
        single_count = _u16_le(fr.data[1:3])
        multi_count = _s32_le(fr.data[3:7])
        return {
            "single_count": single_count,
            "single_deg": counts_to_deg(single_count),
            "multi_count": multi_count,
            "multi_deg": counts_to_deg(multi_count),
        }

    def move_relative_motor_deg(self, motor_delta_deg: float) -> Optional[Dict[str, float]]:
        """相对位置移动(以电机角度为单位)"""
        return self.move_relative_counts(deg_to_counts(motor_delta_deg))

    def go_home_shortest(self) -> Optional[Dict[str, float]]:
        """最短路径回零(C4)"""
        fr = self._cmd1(0xC4)
        if not fr or len(fr.data) < 7:
            return None
        single_count = _u16_le(fr.data[1:3])
        multi_count = _s32_le(fr.data[3:7])
        return {
            "single_count": single_count,
            "single_deg": counts_to_deg(single_count),
            "multi_count": multi_count,
            "multi_deg": counts_to_deg(multi_count),
        }

    def set_brake(self, closed: bool) -> Optional[bool]:
        """设置刹车状态(CE): False=刹车断开, True=刹车闭合"""
        op = 0x01 if closed else 0x00
        fr = self._send(bytes([0xCE, op]), expect_reply=True)
        if not fr or len(fr.data) < 2:
            return None
        return bool(fr.data[1])

    def read_brake(self) -> Optional[bool]:
        """读取刹车状态"""
        fr = self._send(bytes([0xCE, 0xFF]), expect_reply=True)
        if not fr or len(fr.data) < 2:
            return None
        return bool(fr.data[1])

    def disable(self) -> Optional[Dict[str, Union[float, int, str, List[str]]]]:
        """禁用电机(CF)"""
        fr = self._cmd1(0xCF)
        if not fr or len(fr.data) < 8:
            return None
        d = fr.data
        mode = d[6]
        fault = d[7]
        return {
            "bus_voltage_v": _u16_le(d[1:3]) * 0.01,
            "bus_current_a": _u16_le(d[3:5]) * 0.01,
            "temperature_c": d[5],
            "mode": mode,
            "mode_text": MODE_TEXT.get(mode, f"未知({mode})"),
            "fault_code": fault,
            "faults": decode_faults(fault),
        }


# -----------------------------
# 机械臂抽象层
# -----------------------------

@dataclass
class JointConfig:
    """关节配置数据类"""
    key: str                      # 关节唯一标识(如"j1")
    motor_id: int                 # 对应的电机ID
    name: str = ""                # 关节名称(如"J1_Shoulder_Pitch")
    ratio: float = 1.0            # 减速比(电机转数/关节转数)
    direction: float = 1.0        # 方向系数(1或-1，用于统一左右臂逻辑)
    min_deg: float = -180.0       # 最小角度限制(度)
    max_deg: float = 180.0        # 最大角度限制(度)
    default_speed_rpm: float = 5.0        # 默认速度(关节侧RPM，位置模式速度限制)
    default_accel_rpm_s: float = 10.0     # 默认加速度(关节侧RPM/s，速度模式加速度)
    max_current_a: Optional[float] = None  # 最大电流限制(A)


@dataclass
class JointFeedback:
    """关节反馈数据类"""
    key: str                      # 关节唯一标识
    motor_id: int                 # 电机ID
    angle_deg: Optional[float]    # 当前角度(度)
    speed_rpm: Optional[float]    # 当前速度(RPM)
    q_current_a: Optional[float]  # Q轴电流(A)
    status: Optional[Dict[str, Union[float, int, str, List[str]]]]  # 电机状态


class CanArm:
    """
    由多个独立电机对象组成的机械臂控制类。

    设计原则：
    - 单电机库完整且可复用。
    - 机械臂库仅处理ID数组/配置、关节减速比、方向、限位和分组命令。
    """

    def __init__(
        self,
        bus: SerialUsbCanTransport,
        joints: Sequence[JointConfig],
        name: str = "arm",
        use_host_id_offset: bool = True,
    ) -> None:
        if len(joints) == 0:
            raise ValueError("关节列表不能为空")
        self.bus = bus                    # CAN总线传输对象
        self.name = name                  # 机械臂名称
        self.joints: Dict[str, JointConfig] = {j.key: j for j in joints}  # 关节配置字典
        self.motors: Dict[str, CanMotor] = {
            j.key: CanMotor(bus, j.motor_id, name=f"{name}.{j.key}", use_host_id_offset=use_host_id_offset)
            for j in joints
        }  # 电机对象字典
        self.current_deg: Dict[str, float] = {j.key: 0.0 for j in joints}  # 当前角度缓存

    def keys(self) -> List[str]:
        """返回所有关节的key列表"""
        return list(self.joints.keys())

    def _cfg(self, key: str) -> JointConfig:
        """获取关节配置"""
        if key not in self.joints:
            raise KeyError(f"未知关节 {key}，可用关节: {self.keys()}")
        return self.joints[key]

    def _motor(self, key: str) -> CanMotor:
        """获取关节对应的电机对象"""
        self._cfg(key)
        return self.motors[key]

    @staticmethod
    def _check_dir(direction: float) -> float:
        """标准化方向系数为1或-1"""
        return -1.0 if direction < 0 else 1.0

    def logic_deg_to_motor_deg(self, key: str, logic_deg: float) -> float:
        """将逻辑角度(关节角度)转换为电机角度"""
        cfg = self._cfg(key)
        direction = self._check_dir(cfg.direction)
        return logic_deg * direction * cfg.ratio

    def motor_deg_to_logic_deg(self, key: str, motor_deg: float) -> float:
        """将电机角度转换为逻辑角度(关节角度)"""
        cfg = self._cfg(key)
        direction = self._check_dir(cfg.direction)
        return (motor_deg / cfg.ratio) * direction

    def check_limit(self, key: str, logic_deg: float) -> None:
        """检查目标角度是否在限位范围内"""
        cfg = self._cfg(key)
        if not (cfg.min_deg <= logic_deg <= cfg.max_deg):
            raise ValueError(
                f"{self.name}.{key} 目标角度 {logic_deg:.2f} deg 超出限位范围 "
                f"[{cfg.min_deg}, {cfg.max_deg}]"
            )

    # ---- 初始化/安全 ----

    def clear_faults(self, delay_s: float = 0.02) -> None:
        """清除所有关节的故障"""
        for key in self.keys():
            self._motor(key).clear_fault()
            time.sleep(delay_s)

    def configure_defaults(self, delay_s: float = 0.02) -> None:
        """应用JointConfig中的默认速度/电流配置"""
        for key, cfg in self.joints.items():
            self.set_joint_speed_limit(key, cfg.default_speed_rpm)
            time.sleep(delay_s)
            self.set_joint_accel_limit(key, cfg.default_accel_rpm_s)
            time.sleep(delay_s)
            if cfg.max_current_a is not None:
                self._motor(key).set_max_q_current(cfg.max_current_a)
                time.sleep(delay_s)

    def disable_all(self, delay_s: float = 0.02) -> None:
        """禁用所有关节电机"""
        for key in self.keys():
            self._motor(key).disable()
            time.sleep(delay_s)

    def set_zero(self, key: str) -> Optional[Dict[str, float]]:
        """设置单个关节的零点"""
        return self._motor(key).set_zero()

    def set_zero_all(self, delay_s: float = 0.2) -> Dict[str, Optional[Dict[str, float]]]:
        """设置所有关节的零点"""
        ret: Dict[str, Optional[Dict[str, float]]] = {}
        for key in self.keys():
            ret[key] = self.set_zero(key)
            time.sleep(delay_s)
        return ret

    # ---- 读取 ----

    def read_joint_feedback(self, key: str) -> JointFeedback:
        """读取单个关节的反馈数据"""
        cfg = self._cfg(key)
        motor = self._motor(key)

        angle_deg: Optional[float] = None
        speed_rpm: Optional[float] = None
        q_current_a: Optional[float] = None

        angle = motor.read_angle()
        if angle is not None:
            angle_deg = self.motor_deg_to_logic_deg(key, float(angle["multi_deg"]))
            self.current_deg[key] = angle_deg

        motor_speed = motor.read_speed_rpm()
        if motor_speed is not None:
            speed_rpm = self.motor_deg_to_logic_deg(key, motor_speed * 360.0) / 360.0

        q_current_a = motor.read_q_current()
        status = motor.read_status()

        return JointFeedback(
            key=key,
            motor_id=cfg.motor_id,
            angle_deg=angle_deg,
            speed_rpm=speed_rpm,
            q_current_a=q_current_a,
            status=status,
        )

    def read_all_feedback(self, delay_s: float = 0.01) -> Dict[str, JointFeedback]:
        """读取所有关节的反馈数据"""
        ret: Dict[str, JointFeedback] = {}
        for key in self.keys():
            ret[key] = self.read_joint_feedback(key)
            time.sleep(delay_s)
        return ret

    def read_all_status(self, delay_s: float = 0.01) -> Dict[str, Optional[Dict[str, Union[float, int, str, List[str]]]]]:
        """读取所有关节的状态"""
        ret = {}
        for key in self.keys():
            ret[key] = self._motor(key).read_status()
            time.sleep(delay_s)
        return ret

    # ---- 参数设置 ----

    def set_joint_speed_limit(self, key: str, joint_rpm: float) -> bool:
        """设置关节速度限制"""
        cfg = self._cfg(key)
        motor_rpm = abs(joint_rpm * cfg.ratio)
        return self._motor(key).set_position_max_speed(motor_rpm)

    def set_joint_accel_limit(self, key: str, joint_rpm_s: float) -> bool:
        """设置关节加速度限制"""
        cfg = self._cfg(key)
        motor_rpm_s = abs(joint_rpm_s * cfg.ratio)
        return self._motor(key).set_speed_acceleration(motor_rpm_s)

    def set_all_speed_limit(self, joint_rpm: float, delay_s: float = 0.01) -> None:
        """设置所有关节的速度限制"""
        for key in self.keys():
            self.set_joint_speed_limit(key, joint_rpm)
            time.sleep(delay_s)

    # ---- 运动控制 ----

    def move_absolute(self, key: str, target_deg: float) -> Optional[Dict[str, float]]:
        """绝对位置移动"""
        self.check_limit(key, target_deg)
        motor_deg = self.logic_deg_to_motor_deg(key, target_deg)
        ret = self._motor(key).move_absolute_motor_deg(motor_deg)
        self.current_deg[key] = target_deg
        return ret

    def move_relative(self, key: str, delta_deg: float) -> Optional[Dict[str, float]]:
        """相对位置移动"""
        start = self.current_deg.get(key, 0.0)
        # 先尝试刷新当前角度；如果超时则使用缓存角度
        fb = self.read_joint_feedback(key)
        if fb.angle_deg is not None:
            start = fb.angle_deg
        return self.move_absolute(key, start + delta_deg)

    def move_many_absolute(self, targets: Dict[str, float], delay_s: float = 0.02) -> Dict[str, Optional[Dict[str, float]]]:
        """批量移动多个关节。示例: {'j1': 20, 'j4': 30}"""
        for key, deg in targets.items():
            self.check_limit(key, deg)

        ret: Dict[str, Optional[Dict[str, float]]] = {}
        for key, deg in targets.items():
            ret[key] = self.move_absolute(key, deg)
            time.sleep(delay_s)
        return ret

    def home(self, delay_s: float = 0.05) -> Dict[str, Optional[Dict[str, float]]]:
        """所有关节回零(移动到0度)"""
        return self.move_many_absolute({key: 0.0 for key in self.keys()}, delay_s=delay_s)

    def set_joint_speed_mode(self, key: str, joint_rpm: float) -> Optional[float]:
        """设置单个关节的速度模式。如果有回复则返回反馈速度(RPM)"""
        cfg = self._cfg(key)
        motor_rpm = joint_rpm * cfg.ratio * self._check_dir(cfg.direction)
        motor_fb = self._motor(key).set_speed(motor_rpm)
        if motor_fb is None:
            return None
        return motor_fb / cfg.ratio * self._check_dir(cfg.direction)

    def stop_speed_mode(self, key: str) -> Optional[float]:
        """停止单个关节的速度模式"""
        return self.set_joint_speed_mode(key, 0.0)

    def stop_all_speed_mode(self, delay_s: float = 0.01) -> None:
        """停止所有关节的速度模式"""
        for key in self.keys():
            self.stop_speed_mode(key)
            time.sleep(delay_s)


# -----------------------------
# 示例配置
# -----------------------------

def make_arm_config_from_ids(
    ids: Sequence[int],
    ratios: Sequence[float] = (1, 1, 1, 1, 1),
    directions: Sequence[float] = (1, 1, 1, 1, 1),
    name_prefix: str = "J",
) -> List[JointConfig]:
    """根据电机ID快速创建机械臂配置，适用于新收到机械臂时的初始化"""
    if not (len(ids) == len(ratios) == len(directions)):
        raise ValueError("ids、ratios、directions长度必须一致")
    return [
        JointConfig(
            key=f"j{i+1}",
            motor_id=int(ids[i]),
            name=f"{name_prefix}{i+1}",
            ratio=float(ratios[i]),
            direction=float(directions[i]),
        )
        for i in range(len(ids))
    ]


# 基于旧版demo结构配置的右臂示例。请根据新机械臂调整ID/减速比/限位。
RIGHT_ARM_EXAMPLE = [
    JointConfig("j1", 35, "J1_肩关节俯仰", ratio=3.0, direction=1, min_deg=-110, max_deg=110),
    JointConfig("j2", 34, "J2_肩关节偏转", ratio=3.0, direction=1, min_deg=-170, max_deg=120),
    JointConfig("j3", 31, "J3_肩关节旋转", ratio=4.0, direction=1, min_deg=-180, max_deg=180),
    JointConfig("j4", 32, "J4_肘关节俯仰", ratio=4.2, direction=1, min_deg=-90,  max_deg=150),
    JointConfig("j5", 33, "J5_腕关节旋转", ratio=1.0, direction=1, min_deg=-120, max_deg=120),
]

# 左臂示例，方向系数为-1以统一控制逻辑
LEFT_ARM_EXAMPLE = [
    JointConfig("j1", 55, "J1_肩关节俯仰", ratio=3.0, direction=-1, min_deg=-110, max_deg=110),
    JointConfig("j2", 1,  "J2_肩关节偏转", ratio=3.0, direction=-1, min_deg=-170, max_deg=120),
    JointConfig("j3", 15, "J3_肩关节旋转", ratio=4.0, direction=-1, min_deg=-180, max_deg=180),
    JointConfig("j4", 18, "J4_肘关节俯仰", ratio=4.2, direction=-1, min_deg=-90,  max_deg=150),
    JointConfig("j5", 23, "J5_腕关节旋转", ratio=1.0, direction=-1, min_deg=-120, max_deg=120),
]
