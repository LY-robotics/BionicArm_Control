"""
Interactive CLI menu for a CAN-bus robotic arm using can_motor_arm_lib.py.

Put this file in the same folder as can_motor_arm_lib.py, then run:
    python arm_cli_menu.py

Windows port example:
    PORT = "COM17"
Ubuntu port example:
    PORT = "/dev/ttyACM0" or "/dev/ttyUSB0"

Design:
    - SerialUsbCanTransport: USB-CAN transport layer
    - CanMotor: single motor protocol layer
    - CanArm: 5-motor arm abstraction layer
    - This file: terminal menu / test console only
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import serial
except Exception:
    serial = None

from can_motor_arm_lib import (
    CanArm,
    JointConfig,
    LEFT_ARM_EXAMPLE,
    RIGHT_ARM_EXAMPLE,
    SerialUsbCanTransport,
    list_serial_ports,
)


# =========================================================
# 1. User configuration area
# =========================================================

# Windows: "COM17"
# Ubuntu : "/dev/ttyACM0" or "/dev/ttyUSB0"
PORT = "COM17"
BAUD = 1_000_000
DEBUG_CAN = False

# Startup behavior. These commands do not move the arm, but they do send CAN frames.
AUTO_CLEAR_FAULTS = True
AUTO_CONFIGURE_DEFAULTS = True

# Optional head controller. Disable by default.
HEAD_ENABLED = False
HEAD_COM_PORT = "COM3"
HEAD_BAUDRATE = 115200

# Choose the arms you actually have connected on this CAN bus.
# For a single new arm, keep only arm 0 and edit its five JointConfig entries.
ARM_CONFIGS: Dict[int, Tuple[str, List[JointConfig]]] = {
    0: ("right_arm", RIGHT_ARM_EXAMPLE),

    # If a second arm is also on this same CAN bus, uncomment this line and ensure IDs are unique.
    # 1: ("left_arm", LEFT_ARM_EXAMPLE),
}

# Example of creating a new single-arm config quickly.
# Replace ARM_CONFIGS above with this when you receive a new arm.
# ARM_CONFIGS = {
#     0: (
#         "new_arm",
#         [
#             JointConfig("j1", 34, "J1_Shoulder_Pitch", ratio=3.0, direction=1, min_deg=-110, max_deg=110),
#             JointConfig("j2", 35, "J2_Shoulder_Yaw",   ratio=3.0, direction=1, min_deg=-170, max_deg=120),
#             JointConfig("j3", 36, "J3_Shoulder_Roll",  ratio=4.0, direction=1, min_deg=-180, max_deg=180),
#             JointConfig("j4", 37, "J4_Elbow_Pitch",    ratio=4.2, direction=1, min_deg=-90,  max_deg=150),
#             JointConfig("j5", 38, "J5_Wrist_Roll",     ratio=1.0, direction=1, min_deg=-120, max_deg=120),
#         ],
#     )
# }

JOINT_ORDER = ["j1", "j2", "j3", "j4", "j5"]


# =========================================================
# 2. Optional head controller
# =========================================================

class HeadController:
    def __init__(self, port: str = HEAD_COM_PORT, baudrate: int = HEAD_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def connect(self) -> bool:
        if not HEAD_ENABLED:
            return False
        if serial is None:
            print("[头部云台] pyserial 未安装，跳过头部连接。")
            return False
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"[头部云台] 串口 {self.port} 连接成功，等待控制器重启...")
            time.sleep(2.0)
            return True
        except Exception as exc:
            print(f"[头部云台] 串口未能连接: {exc}")
            self.ser = None
            return False

    def send_cmd(self, cmd_str: str) -> bool:
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd_str}\n".encode("utf-8"))
            return True
        print("[头部云台] 未连接。可在文件顶部设置 HEAD_ENABLED=True 并配置 HEAD_COM_PORT。")
        return False

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()


# =========================================================
# 3. CLI helpers
# =========================================================

def clear_screen() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def parse_arm_selector(text: str, available: Iterable[int]) -> List[int]:
    text = text.strip().lower()
    available_list = sorted(list(available))
    if text == "all":
        return available_list
    try:
        arm_id = int(text)
    except ValueError as exc:
        raise ValueError("手臂号必须是 0/1/... 或 all") from exc
    if arm_id not in available_list:
        raise ValueError(f"手臂 {arm_id} 不存在，当前可用: {available_list}")
    return [arm_id]


def fmt_num(value: Optional[float], unit: str = "", width: int = 9, precision: int = 2) -> str:
    if value is None:
        return "---".ljust(width)
    return f"{value:.{precision}f}{unit}".ljust(width)


def ask_confirm(prompt: str) -> bool:
    ans = input(f"{prompt} 输入 yes 确认: ").strip().lower()
    return ans == "yes"


def parse_pose_tokens(tokens: List[str]) -> Dict[str, float]:
    """Parse tokens like ['j1=0', 'j2=30'] into {'j1': 0.0, 'j2': 30.0}."""
    targets: Dict[str, float] = {}
    for token in tokens:
        if "=" not in token:
            raise ValueError(f"姿态参数格式错误: {token}，应该类似 j1=0")
        key, val = token.split("=", 1)
        key = key.lower().strip()
        if key not in JOINT_ORDER:
            raise ValueError(f"未知关节 {key}，可用: {JOINT_ORDER}")
        targets[key] = float(val)
    return targets


# =========================================================
# 4. Console class
# =========================================================

class ArmConsole:
    def __init__(self):
        self.bus: Optional[SerialUsbCanTransport] = None
        self.arms: Dict[int, CanArm] = {}
        self.head = HeadController()

    # ---------------- startup / shutdown ----------------

    def connect(self) -> None:
        print("\n========== CAN 机械臂控制台启动 ==========")
        print("可用串口:", list_serial_ports())
        print(f"当前配置: PORT={PORT}, BAUD={BAUD}, DEBUG_CAN={DEBUG_CAN}")
        print("如果端口不对，请先修改本文件顶部 PORT。\n")

        self.bus = SerialUsbCanTransport(PORT, BAUD, debug=DEBUG_CAN)
        self.bus.connect()
        print(f"[CAN] USB-CAN 已连接: {PORT} @ {BAUD}")

        for arm_id, (name, cfg) in ARM_CONFIGS.items():
            self.arms[arm_id] = CanArm(self.bus, cfg, name=name)
            print(f"[ARM {arm_id}] {name} 已创建，关节数: {len(cfg)}")

        self.head.connect()

        if AUTO_CLEAR_FAULTS:
            print("[启动] 清除各关节故障...")
            for arm in self.arms.values():
                arm.clear_faults()

        if AUTO_CONFIGURE_DEFAULTS:
            print("[启动] 下发默认限速/加速度/电流限制配置...")
            for arm in self.arms.values():
                arm.configure_defaults()

        print("[启动] 完成。输入 help 查看指令。")

    def close(self, disable: bool = True) -> None:
        if disable:
            try:
                self.disable_all()
            except Exception as exc:
                print(f"[关闭] 失能时出现异常: {exc}")
        self.head.close()
        if self.bus:
            self.bus.close()
            self.bus = None
        print("[系统] 已关闭。")

    # ---------------- status display ----------------

    def print_help(self) -> None:
        arm_list = sorted(self.arms.keys())
        print(f"""
================ 单/双臂机器人控制台 ================
当前可用手臂: {arm_list}，手臂号可填 0/1/... 或 all

[机械臂控制指令]
1. 静态配置查询   : status <0|1|all>
                  例: status 0
2. 实时动态监控   : watch [0|1|all]
                  例: watch all       Ctrl+C 退出监控
3. 单关节绝对运动 : move <0|1> <关节名> <角度deg>
                  例: move 0 j1 30
4. 单关节微调     : jog <0|1> <关节名> <增量deg>
                  例: jog 0 j4 -10
5. 多关节姿态运动 : pose <0|1> j1=角度 j2=角度 ...
                  例: pose 0 j1=0 j2=20 j4=30
6. 设置位置限速   : speed <0|1> <关节名|all> <RPM>
                  例: speed 0 all 5
7. 设置加速度限制 : accel <0|1> <关节名|all> <RPM/s>
                  例: accel 0 j1 10
8. 一键回零位     : home <0|1|all>
                  例: home all
9. 设定硬件原点   : setzero <0|1|all> <关节名|all>
                  例: setzero 0 j1
                  危险：会写入驱动板，必须确认机械姿态正确

[单电机/调试指令]
10. 读取版本      : version <0|1> <关节名|all>
                  例: version 0 all
11. 读取母线状态  : bus <0|1|all>
                  例: bus all
12. 清除故障      : clear <0|1|all>
                  例: clear all
13. 抱闸控制      : brake <0|1> <关节名> <on|off|read>
                  例: brake 0 j1 read
14. 速度模式测试  : vel <0|1> <关节名> <RPM> [持续秒]
                  例: vel 0 j2 1 2
                  危险：这是 C1 速度模式，会直接让该关节转动
15. 停止速度模式  : velstop <0|1> <关节名|all>
                  例: velstop 0 all

[系统控制]
16. 紧急失能/释放 : stop
17. 串口列表      : ports
18. 清屏          : cls
19. 获取帮助菜单  : help
20. 退出控制台    : quit / exit

[可选云台及动作展示]
21. 云台角度      : head x <0-270> / head y <0-180>
                  例: head x 90
22. 仿人摆臂Demo  : demo1
23. 对称展开Demo  : demo3
======================================================
说明：speed 是位置模式最大速度限制，不会直接让电机转；vel 才是直接速度控制。
""")

    def print_static_config(self, arm_ids: List[int]) -> None:
        for arm_id in arm_ids:
            arm = self.arms[arm_id]
            print(f"\n==================== [ARM {arm_id} / {arm.name}] 静态配置 ====================")
            print(f"{'关节':<6} | {'ID':<5} | {'名称':<22} | {'减速比':<7} | {'Dir':<5} | {'限位deg':<20} | {'默认速度'}")
            print("-" * 100)
            for key in arm.keys():
                cfg = arm.joints[key]
                print(
                    f"{key.upper():<6} | {cfg.motor_id:<5} | {cfg.name:<22} | "
                    f"{cfg.ratio:<7.2f} | {cfg.direction:<5.1f} | "
                    f"{cfg.min_deg:>7.1f} ~ {cfg.max_deg:<7.1f} | {cfg.default_speed_rpm:.2f} RPM"
                )
            print("=" * 100)

    def print_feedback_once(self, arm_ids: List[int]) -> None:
        for arm_id in arm_ids:
            arm = self.arms[arm_id]
            print(f"\n============== [ARM {arm_id} / {arm.name}] 实时反馈 ==============")
            print(f"{'关节':<6} | {'ID':<5} | {'角度deg':<12} | {'速度RPM':<12} | {'电流A':<10} | {'模式':<14} | {'故障'}")
            print("-" * 100)
            fb = arm.read_all_feedback()
            for key in arm.keys():
                item = fb[key]
                mode = "---"
                faults = "---"
                if item.status:
                    mode = str(item.status.get("mode_text", "---"))
                    f = item.status.get("faults", [])
                    faults = "none" if not f else ",".join(str(x) for x in f)
                print(
                    f"{key.upper():<6} | {item.motor_id:<5} | "
                    f"{fmt_num(item.angle_deg, '', 12, 2)} | "
                    f"{fmt_num(item.speed_rpm, '', 12, 2)} | "
                    f"{fmt_num(item.q_current_a, '', 10, 3)} | "
                    f"{mode:<14} | {faults}"
                )

    def watch(self, arm_ids: List[int]) -> None:
        print("进入实时监控模式，按 Ctrl+C 退出。")
        time.sleep(0.8)
        try:
            while True:
                clear_screen()
                print("实时监控中，按 Ctrl+C 退出。")
                self.print_feedback_once(arm_ids)
                time.sleep(0.15)
        except KeyboardInterrupt:
            print("\n[监控] 已退出。")

    # ---------------- operations ----------------

    def disable_all(self) -> None:
        for arm_id, arm in self.arms.items():
            print(f"[STOP] ARM {arm_id} 失能释放...")
            arm.disable_all()

    def clear_faults(self, arm_ids: List[int]) -> None:
        for arm_id in arm_ids:
            self.arms[arm_id].clear_faults()
            print(f"[ARM {arm_id}] 故障清除命令已下发。")

    def show_bus_status(self, arm_ids: List[int]) -> None:
        for arm_id in arm_ids:
            arm = self.arms[arm_id]
            print(f"\n[ARM {arm_id} / {arm.name}] 母线状态")
            for key in arm.keys():
                st = arm.motors[key].read_status()
                if not st:
                    print(f"  {key.upper()} ID={arm.joints[key].motor_id}: 无响应")
                    continue
                print(
                    f"  {key.upper()} ID={arm.joints[key].motor_id:<3} "
                    f"Vbus={st['bus_voltage_v']:.2f}V "
                    f"Ibus={st['bus_current_a']:.2f}A "
                    f"Temp={st['temperature_c']}C "
                    f"Mode={st['mode_text']} "
                    f"Fault={st['faults'] or 'none'}"
                )

    def show_versions(self, arm_id: int, joint: str) -> None:
        arm = self.arms[arm_id]
        keys = arm.keys() if joint == "all" else [joint]
        print(f"\n[ARM {arm_id} / {arm.name}] 版本信息")
        for key in keys:
            ver = arm.motors[key].read_version()
            print(f"  {key.upper()} ID={arm.joints[key].motor_id}: {ver}")

    def set_speed_limit(self, arm_id: int, joint: str, rpm: float) -> None:
        arm = self.arms[arm_id]
        keys = arm.keys() if joint == "all" else [joint]
        for key in keys:
            ok = arm.set_joint_speed_limit(key, rpm)
            print(f"[ARM {arm_id} {key.upper()}] 位置限速 {rpm} RPM -> {'OK' if ok else 'FAILED'}")

    def set_accel_limit(self, arm_id: int, joint: str, rpm_s: float) -> None:
        arm = self.arms[arm_id]
        keys = arm.keys() if joint == "all" else [joint]
        for key in keys:
            ok = arm.set_joint_accel_limit(key, rpm_s)
            print(f"[ARM {arm_id} {key.upper()}] 加速度限制 {rpm_s} RPM/s -> {'OK' if ok else 'FAILED'}")

    def move_absolute(self, arm_id: int, joint: str, deg: float) -> None:
        arm = self.arms[arm_id]
        ret = arm.move_absolute(joint, deg)
        print(f"[MOVE] ARM {arm_id} {joint.upper()} -> {deg:.2f} deg, ret={ret}")

    def move_relative(self, arm_id: int, joint: str, delta: float) -> None:
        arm = self.arms[arm_id]
        ret = arm.move_relative(joint, delta)
        print(f"[JOG] ARM {arm_id} {joint.upper()} += {delta:.2f} deg, ret={ret}")

    def move_pose(self, arm_id: int, targets: Dict[str, float]) -> None:
        arm = self.arms[arm_id]
        print(f"[POSE] ARM {arm_id} targets={targets}")
        ret = arm.move_many_absolute(targets)
        print(f"[POSE] ret={ret}")

    def home(self, arm_ids: List[int]) -> None:
        if not ask_confirm(f"即将让 ARM {arm_ids} 回到逻辑零位，确认机械空间安全吗？"):
            print("[HOME] 已取消。")
            return
        for arm_id in arm_ids:
            ret = self.arms[arm_id].home()
            print(f"[HOME] ARM {arm_id} ret={ret}")

    def set_zero(self, arm_ids: List[int], joint: str) -> None:
        msg = f"即将给 ARM {arm_ids} 的 {joint} 写入硬件原点。这个操作会保存到驱动板，断电不丢失。确认当前姿态就是零点吗？"
        if not ask_confirm(msg):
            print("[SETZERO] 已取消。")
            return
        for arm_id in arm_ids:
            arm = self.arms[arm_id]
            keys = arm.keys() if joint == "all" else [joint]
            for key in keys:
                ret = arm.set_zero(key)
                print(f"[SETZERO] ARM {arm_id} {key.upper()} ret={ret}")
                time.sleep(0.2)
        print("[SETZERO] 完成。建议重新上电后再读取状态确认。")

    def brake(self, arm_id: int, joint: str, op: str) -> None:
        arm = self.arms[arm_id]
        motor = arm.motors[joint]
        if op == "read":
            ret = motor.read_brake()
        elif op == "on":
            ret = motor.set_brake(True)
        elif op == "off":
            ret = motor.set_brake(False)
        else:
            raise ValueError("brake 操作必须是 on/off/read")
        print(f"[BRAKE] ARM {arm_id} {joint.upper()} {op} -> {ret}")

    def speed_mode_test(self, arm_id: int, joint: str, rpm: float, duration_s: Optional[float]) -> None:
        arm = self.arms[arm_id]
        if not ask_confirm(f"即将让 ARM {arm_id} {joint.upper()} 进入速度模式 {rpm} RPM，确认该关节可自由转动吗？"):
            print("[VEL] 已取消。")
            return
        fb = arm.set_joint_speed_mode(joint, rpm)
        print(f"[VEL] 已下发 {rpm} RPM，反馈速度={fb}")
        if duration_s is not None and duration_s > 0:
            time.sleep(duration_s)
            fb_stop = arm.stop_speed_mode(joint)
            print(f"[VEL] 持续 {duration_s:.2f}s 后停止，反馈速度={fb_stop}")

    def velstop(self, arm_id: int, joint: str) -> None:
        arm = self.arms[arm_id]
        keys = arm.keys() if joint == "all" else [joint]
        for key in keys:
            ret = arm.stop_speed_mode(key)
            print(f"[VELSTOP] ARM {arm_id} {key.upper()} -> {ret}")

    # ---------------- demo functions ----------------

    def demo1(self) -> None:
        if len(self.arms) < 2:
            print("[DEMO1] 需要两条手臂。当前只有一条时跳过。")
            return
        if not ask_confirm("Demo1 会执行双臂摆臂动作，确认机械空间安全吗？"):
            return
        print("[DEMO1] 仿人走路摆臂开始...")
        try:
            self.head.send_cmd("X90")
            self.head.send_cmd("Y90")
            for arm_id in sorted(self.arms.keys()):
                arm = self.arms[arm_id]
                arm.move_many_absolute({"j1": 0, "j2": 0, "j3": -20, "j4": 10, "j5": 0})
            time.sleep(1.5)
            for i in range(3):
                self.arms[1].move_many_absolute({"j1": 45, "j4": 40})
                self.arms[0].move_many_absolute({"j1": -20, "j4": 10})
                self.head.send_cmd("X80")
                time.sleep(1.2)
                self.arms[1].move_many_absolute({"j1": -20, "j4": 10})
                self.arms[0].move_many_absolute({"j1": 45, "j4": 40})
                self.head.send_cmd("X100")
                time.sleep(1.2)
            for arm_id in sorted(self.arms.keys()):
                self.arms[arm_id].move_many_absolute({"j1": 0, "j4": 10})
            self.head.send_cmd("X90")
            print("[DEMO1] 完成。")
        except KeyboardInterrupt:
            print("[DEMO1] 中断。")

    def demo3(self) -> None:
        if len(self.arms) < 2:
            print("[DEMO3] 需要两条手臂。当前只有一条时跳过。")
            return
        if not ask_confirm("Demo3 会执行双臂对称展开动作，确认机械空间安全吗？"):
            return
        print("[DEMO3] 双臂对称展开开始...")
        try:
            self.head.send_cmd("X90")
            self.head.send_cmd("Y90")
            for arm_id in sorted(self.arms.keys()):
                self.arms[arm_id].move_absolute("j2", 70)
            time.sleep(2.5)
            for arm_id in sorted(self.arms.keys()):
                self.arms[arm_id].move_many_absolute({"j1": 30, "j3": -60, "j4": 20})
            self.head.send_cmd("X45")
            time.sleep(3.0)
            for arm_id in sorted(self.arms.keys()):
                self.arms[arm_id].move_many_absolute({"j1": 0, "j3": 0, "j4": 0, "j5": 0})
            self.head.send_cmd("X135")
            time.sleep(3.0)
            for arm_id in sorted(self.arms.keys()):
                self.arms[arm_id].move_absolute("j2", 0)
            self.head.send_cmd("X90")
            print("[DEMO3] 完成。")
        except KeyboardInterrupt:
            print("[DEMO3] 中断。")

    # ---------------- command dispatcher ----------------

    def run(self) -> None:
        self.print_help()
        while True:
            try:
                raw = input("\nArm-CLI > ").strip()
                if not raw:
                    continue
                parts = raw.split()
                cmd = parts[0].lower()

                if cmd in {"quit", "exit"}:
                    break

                if cmd == "help":
                    self.print_help()

                elif cmd == "ports":
                    print("可用串口:", list_serial_ports())

                elif cmd in {"cls", "clear_screen"}:
                    clear_screen()

                elif cmd == "status":
                    target = parts[1] if len(parts) > 1 else "all"
                    self.print_static_config(parse_arm_selector(target, self.arms.keys()))
                    self.print_feedback_once(parse_arm_selector(target, self.arms.keys()))

                elif cmd == "watch":
                    target = parts[1] if len(parts) > 1 else "all"
                    self.watch(parse_arm_selector(target, self.arms.keys()))

                elif cmd == "move" and len(parts) == 4:
                    self.move_absolute(int(parts[1]), parts[2].lower(), float(parts[3]))

                elif cmd == "jog" and len(parts) == 4:
                    self.move_relative(int(parts[1]), parts[2].lower(), float(parts[3]))

                elif cmd == "pose" and len(parts) >= 3:
                    self.move_pose(int(parts[1]), parse_pose_tokens(parts[2:]))

                elif cmd == "speed" and len(parts) == 4:
                    self.set_speed_limit(int(parts[1]), parts[2].lower(), float(parts[3]))

                elif cmd == "accel" and len(parts) == 4:
                    self.set_accel_limit(int(parts[1]), parts[2].lower(), float(parts[3]))

                elif cmd == "home":
                    target = parts[1] if len(parts) > 1 else "all"
                    self.home(parse_arm_selector(target, self.arms.keys()))

                elif cmd == "setzero" and len(parts) >= 2:
                    target = parts[1]
                    joint = parts[2].lower() if len(parts) >= 3 else "all"
                    self.set_zero(parse_arm_selector(target, self.arms.keys()), joint)

                elif cmd == "version" and len(parts) == 3:
                    self.show_versions(int(parts[1]), parts[2].lower())

                elif cmd == "bus":
                    target = parts[1] if len(parts) > 1 else "all"
                    self.show_bus_status(parse_arm_selector(target, self.arms.keys()))

                elif cmd == "clear":
                    target = parts[1] if len(parts) > 1 else "all"
                    self.clear_faults(parse_arm_selector(target, self.arms.keys()))

                elif cmd == "brake" and len(parts) == 4:
                    self.brake(int(parts[1]), parts[2].lower(), parts[3].lower())

                elif cmd == "vel" and len(parts) in {4, 5}:
                    duration = float(parts[4]) if len(parts) == 5 else None
                    self.speed_mode_test(int(parts[1]), parts[2].lower(), float(parts[3]), duration)

                elif cmd == "velstop" and len(parts) == 3:
                    self.velstop(int(parts[1]), parts[2].lower())

                elif cmd == "stop":
                    self.disable_all()
                    print("[STOP] 所有已配置手臂均已失能释放。")

                elif cmd == "head" and len(parts) >= 3:
                    axis = parts[1].lower()
                    val = parts[2]
                    if axis == "x":
                        self.head.send_cmd(f"X{val}")
                    elif axis == "y":
                        self.head.send_cmd(f"Y{val}")
                    else:
                        print("head 指令只支持: head x <角度> / head y <角度>")

                elif cmd == "demo1":
                    self.demo1()

                elif cmd == "demo3":
                    self.demo3()

                else:
                    print("输入错误或参数数量不对。输入 help 查看支持的指令。")

            except KeyboardInterrupt:
                print("\n[中断] 已捕获 Ctrl+C。输入 stop 可失能，输入 quit 退出。")
            except Exception as exc:
                print(f"[错误] {exc}")


def main() -> None:
    console = ArmConsole()
    try:
        console.connect()
        console.run()
    except KeyboardInterrupt:
        print("\n[系统] Ctrl+C 退出。")
    except Exception as exc:
        print(f"\n[系统异常] {exc}")
    finally:
        console.close(disable=True)


if __name__ == "__main__":
    main()
