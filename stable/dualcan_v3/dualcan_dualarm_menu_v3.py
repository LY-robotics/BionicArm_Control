# -*- coding: utf-8 -*-
"""
dualcan_dualarm_menu_v3.py

同一个 COM 口 + CAN1/CAN2 分别控制两条机械臂的菜单测试程序。

默认结构：
    ARM0 右臂 -> CAN1
    ARM1 左臂 -> CAN2

运行：
    python dualcan_dualarm_menu_v3.py
"""

from __future__ import annotations

import sys
import time
from typing import Dict, List

import serial

from dualcan_arm_control_lib_v3 import (
    CanArm,
    DualCanDualArmSystem,
    JointConfig,
    get_default_left_arm_config,
    get_default_right_arm_config,
)


# ============================================================
# 1. 用户配置区
# ============================================================

PORT = "COM17"
BAUDRATE = 1_000_000

RIGHT_ARM_CHANNEL = 1
LEFT_ARM_CHANNEL = 2

DEBUG_CAN = False

# 头部云台，可选
HEAD_ENABLE = False
HEAD_COM_PORT = "COM3"
HEAD_BAUDRATE = 115200

RIGHT_ARM_CONFIG = get_default_right_arm_config()
LEFT_ARM_CONFIG = get_default_left_arm_config()


# ============================================================
# 2. 头部云台控制，可选
# ============================================================

class HeadController:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(1.5)
            print(f"[头部云台] 已连接 {self.port}")
            return True
        except Exception as exc:
            print(f"[头部云台] 连接失败: {exc}")
            return False

    def send_cmd(self, cmd: str) -> bool:
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode("utf-8"))
            return True
        print("[头部云台] 未连接，忽略命令。")
        return False

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()


# ============================================================
# 3. 菜单与打印
# ============================================================

def print_help() -> None:
    print("""
================ 双 CAN 双机械臂控制台 V3 ================

硬件结构：
    一个 COM 口，通过 Normal/Passthrough 可变长度帧通信
    ARM 0 = 右臂 = CAN1
    ARM 1 = 左臂 = CAN2

[基础查询]
    help
        显示帮助。

    config <0|1|all>
        查看静态配置：ID、方向、减速比、限位、默认速度/加速度。
        例：config all

    version <0|1|all>
        读取各关节电机版本 A0。
        例：version 0

    status <0|1|all>
        静态配置 + 实时反馈：角度、速度、电流、温度、电压、模式、故障。
        例：status all

    bus <0|1|all>
        只读取 AE 状态：电压、电流、温度、模式、故障。
        例：bus 1

    watch <0|1|all>
        实时监控，Ctrl+C 退出。
        例：watch all

[运动控制]
    move <0|1> <j1~j5> <角度deg>
        单关节绝对运动，带软件限位。
        例：move 0 j2 10

    jog <0|1> <j1~j5> <增量deg>
        单关节相对微调。
        例：jog 1 j4 -5

    pose <0|1> j1=角度 j2=角度 ...
        多关节姿态下发，可只写部分关节。
        例：pose 0 j1=0 j2=20 j4=30

    home <0|1|all>
        一键回软件零位，即所有关节运动到 0°。
        例：home all

    vel <0|1> <j1~j5> <RPM> <seconds>
        单关节速度模式测试，执行后自动 C1 0 停止。
        例：vel 0 j2 5 2

[参数配置]
    speed <0|1> <j1~j5|all> <RPM>
        设置位置模式最大速度限制 B2。注意：不会直接转。
        例：speed 0 all 5

    accel <0|1> <j1~j5|all> <RPM/s>
        设置速度/位置运动加速度 B5。
        例：accel 1 all 10

    current <0|1> <j1~j5|all> <A>
        设置最大 Q 轴电流 B3。
        例：current 0 all 1.5

    clear <0|1|all>
        清故障 AF。
        例：clear all

    setzero <0|1> <j1~j5|all>
        设置当前位置为硬件零点 B1，会写入驱动板，危险操作。
        例：setzero 0 j2

[安全控制]
    holdstop <0|1|all>
        速度置 0，不 CF 失能。

    disable <0|1|all>
        CF 失能释放力矩。

    stop
        全系统停止：速度置 0 + CF 失能。

[头部云台，可选]
    head x <角度>
    head y <角度>
    head shake
    head nod

[动作 Demo]
    demo1
        仿人走路摆臂，使用双臂。

    demo3
        双臂对称展开协同动作。

[系统]
    quit / exit
        退出前自动 stop。

===========================================================
""")


def parse_arm_list(token: str) -> List[int]:
    token = token.lower()
    if token == "all":
        return [0, 1]
    arm_id = int(token)
    if arm_id not in (0, 1):
        raise ValueError("手臂号只能是 0、1 或 all。")
    return [arm_id]


def arm_desc(arm_id: int) -> str:
    return "右臂" if arm_id == 0 else "左臂"


def print_config(system: DualCanDualArmSystem, arm_id: int) -> None:
    arm = system.get_arm(arm_id)
    ch = system.arm_channel(arm_id)
    print(f"\n================ [ARM {arm_id} / {arm_desc(arm_id)} / CAN{ch}] 静态配置 ================")
    print(f"{'关节':<5} | {'ID':<4} | {'名称':<22} | {'ratio':<6} | {'dir':<5} | {'限位(deg)':<18} | {'speed':<7} | {'accel'}")
    print("-" * 105)
    for key in arm.joint_keys():
        cfg = arm.get_joint(key)
        print(
            f"{key:<5} | {cfg.motor_id:<4} | {cfg.name:<22} | {cfg.ratio:<6.2f} | "
            f"{cfg.direction:<5.1f} | {cfg.min_deg:>6.1f} ~ {cfg.max_deg:<6.1f} | "
            f"{cfg.default_speed_rpm:<7.2f} | {cfg.default_accel_rpm_s:.2f}"
        )


def print_versions(system: DualCanDualArmSystem, arm_id: int) -> None:
    arm = system.get_arm(arm_id)
    ch = system.arm_channel(arm_id)
    print(f"\n================ [ARM {arm_id} / {arm_desc(arm_id)} / CAN{ch}] 电机版本 A0 ================")
    print(f"{'关节':<5} | {'ID':<4} | {'Boot':<8} | {'App':<8} | {'HW':<8} | {'CAN协议'}")
    print("-" * 70)
    versions = arm.read_all_versions()
    for key, ver in versions.items():
        cfg = arm.get_joint(key)
        if isinstance(ver, str):
            print(f"{key:<5} | {cfg.motor_id:<4} | ERROR: {ver}")
        else:
            print(f"{key:<5} | {cfg.motor_id:<4} | {ver.boot:<8} | {ver.app:<8} | {ver.hardware:<8} | {ver.can_protocol}")


def print_bus_status(system: DualCanDualArmSystem, arm_id: int) -> None:
    arm = system.get_arm(arm_id)
    ch = system.arm_channel(arm_id)
    print(f"\n================ [ARM {arm_id} / {arm_desc(arm_id)} / CAN{ch}] AE 状态 ================")
    print(f"{'关节':<5} | {'ID':<4} | {'电压(V)':<10} | {'电流(A)':<10} | {'温度':<6} | {'模式':<10} | {'故障'}")
    print("-" * 85)
    states = arm.read_all_status()
    for key, st in states.items():
        cfg = arm.get_joint(key)
        if isinstance(st, str):
            print(f"{key:<5} | {cfg.motor_id:<4} | ERROR: {st}")
        else:
            print(
                f"{key:<5} | {cfg.motor_id:<4} | {st.bus_voltage_v:<10.2f} | "
                f"{st.bus_current_a:<10.2f} | {st.temperature_c:<6} | "
                f"{st.mode_name():<10} | 0x{st.fault:02X}"
            )


def print_feedback(system: DualCanDualArmSystem, arm_id: int, with_status: bool = True) -> None:
    arm = system.get_arm(arm_id)
    ch = system.arm_channel(arm_id)
    print(f"\n================ [ARM {arm_id} / {arm_desc(arm_id)} / CAN{ch}] 实时反馈 ================")
    print(f"{'关节':<5} | {'ID':<4} | {'角度(deg)':<12} | {'速度(RPM)':<12} | {'电流(A)':<10} | {'电压':<7} | {'温度':<6} | {'模式':<5} | {'故障'}")
    print("-" * 105)
    fbs = arm.read_all_feedback(with_status=with_status)
    for key, fb in fbs.items():
        if fb.error:
            print(f"{key:<5} | {fb.motor_id:<4} | ERROR: {fb.error}")
            continue
        angle = f"{fb.angle_deg:.2f}" if fb.angle_deg is not None else "---"
        speed = f"{fb.speed_rpm:.2f}" if fb.speed_rpm is not None else "---"
        current = f"{fb.current_a:.3f}" if fb.current_a is not None else "---"
        voltage = f"{fb.voltage_v:.2f}" if fb.voltage_v is not None else "---"
        temp = str(fb.temperature_c) if fb.temperature_c is not None else "---"
        mode = str(fb.mode) if fb.mode is not None else "---"
        fault = f"0x{fb.fault:02X}" if fb.fault is not None else "---"
        print(f"{key:<5} | {fb.motor_id:<4} | {angle:<12} | {speed:<12} | {current:<10} | {voltage:<7} | {temp:<6} | {mode:<5} | {fault}")


def confirm(prompt: str) -> bool:
    print(prompt)
    return input("输入 yes 确认 > ").strip().lower() == "yes"


def apply_to_joints(arm: CanArm, joint_token: str, func) -> None:
    keys = arm.joint_keys() if joint_token == "all" else [joint_token]
    for key in keys:
        func(key)
        time.sleep(0.01)


def watch(system: DualCanDualArmSystem, arm_ids: List[int]) -> None:
    print("进入实时监控，Ctrl+C 退出。")
    try:
        while True:
            print("\033c", end="")
            for arm_id in arm_ids:
                print_feedback(system, arm_id, with_status=True)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\n退出监控。")


# ============================================================
# 4. Demo 动作
# ============================================================

def live_monitor_short(system: DualCanDualArmSystem, duration_s: float) -> None:
    t0 = time.time()
    while time.time() - t0 < duration_s:
        for arm_id in [0, 1]:
            print_feedback(system, arm_id, with_status=False)
        time.sleep(0.4)


def demo1(system: DualCanDualArmSystem, head: HeadController | None = None, loop_count: int = 3, duration_s: float = 1.2) -> None:
    """
    仿人走路自然摆臂。
    """
    print("\n[Demo1] 仿人走路摆臂开始。")
    if head:
        head.send_cmd("X90")
        head.send_cmd("Y90")

    # 初始下垂姿态
    for a_id in [0, 1]:
        arm = system.get_arm(a_id)
        arm.move_many_absolute({"j1": 0.0, "j2": 0.0, "j3": -20.0, "j4": 10.0, "j5": 0.0})
    time.sleep(1.5)

    for i in range(loop_count):
        print(f"[Demo1] 循环 {i + 1}/{loop_count} A")
        system.left_arm.move_many_absolute({"j1": 45.0, "j4": 40.0})
        system.right_arm.move_many_absolute({"j1": -20.0, "j4": 10.0})
        if head:
            head.send_cmd("X80")
        time.sleep(duration_s)

        print(f"[Demo1] 循环 {i + 1}/{loop_count} B")
        system.left_arm.move_many_absolute({"j1": -20.0, "j4": 10.0})
        system.right_arm.move_many_absolute({"j1": 45.0, "j4": 40.0})
        if head:
            head.send_cmd("X100")
        time.sleep(duration_s)

    print("[Demo1] 恢复中位。")
    for a_id in [0, 1]:
        system.get_arm(a_id).move_many_absolute({"j1": 0.0, "j4": 10.0})
    if head:
        head.send_cmd("X90")
    print("[Demo1] 完成。")


def smooth_head_sync(system: DualCanDualArmSystem, head: HeadController | None, axis: str, start: int, target: int, duration_s: float, steps: int) -> None:
    if not head:
        time.sleep(duration_s)
        return
    dt = duration_s / max(1, steps)
    for i in range(steps):
        val = int(start + (target - start) * (i + 1) / steps)
        head.send_cmd(f"{axis.upper()}{val}")
        time.sleep(dt)


def demo3(system: DualCanDualArmSystem, head: HeadController | None = None) -> None:
    """
    双臂对称展开协同动作。
    """
    print("\n[Demo3] 双臂对称展开开始。")
    if head:
        head.send_cmd("X90")
        head.send_cmd("Y90")

    print("[Demo3] 阶段1：双臂 J2 向外展开。")
    system.right_arm.move_absolute("j2", 70.0)
    system.left_arm.move_absolute("j2", 70.0)
    time.sleep(2.0)

    print("[Demo3] 阶段2：其他关节就位，头部看右侧。")
    for a_id in [0, 1]:
        system.get_arm(a_id).move_many_absolute({"j1": 30.0, "j3": -60.0, "j4": 20.0})
    smooth_head_sync(system, head, "X", 90, 45, 2.0, 20)

    print("[Demo3] 阶段3：恢复 J1/J3/J4/J5，头部看左侧。")
    for a_id in [0, 1]:
        system.get_arm(a_id).move_many_absolute({"j1": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0})
    smooth_head_sync(system, head, "X", 45, 135, 2.5, 25)

    print("[Demo3] 阶段4：J2 回零。")
    system.right_arm.move_absolute("j2", 0.0)
    system.left_arm.move_absolute("j2", 0.0)
    time.sleep(2.0)

    print("[Demo3] 阶段5：头部回中。")
    smooth_head_sync(system, head, "X", 135, 90, 1.5, 15)
    print("[Demo3] 完成。")


# ============================================================
# 5. 主程序
# ============================================================

def main() -> int:
    print("\n[启动] 双 CAN 双机械臂控制台 V3")
    print(f"PORT={PORT}, BAUDRATE={BAUDRATE}")
    print(f"ARM0 右臂 -> CAN{RIGHT_ARM_CHANNEL}")
    print(f"ARM1 左臂 -> CAN{LEFT_ARM_CHANNEL}")
    print(f"DEBUG_CAN={DEBUG_CAN}")

    system = DualCanDualArmSystem(
        port=PORT,
        baudrate=BAUDRATE,
        right_channel=RIGHT_ARM_CHANNEL,
        left_channel=LEFT_ARM_CHANNEL,
        right_arm_config=RIGHT_ARM_CONFIG,
        left_arm_config=LEFT_ARM_CONFIG,
        debug=DEBUG_CAN,
    )

    head = None
    try:
        system.connect()
        print("[OK] USB-CAN 已进入 Normal/Passthrough 模式。")

        if HEAD_ENABLE:
            head = HeadController(HEAD_COM_PORT, HEAD_BAUDRATE)
            head.connect()

        print_help()

        while True:
            raw = input("\nDualCAN-Arm-V3> ").strip()
            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            try:
                if cmd in ("quit", "exit"):
                    print("退出前执行 stop...")
                    system.stop_all_speed()
                    system.disable_all()
                    break

                elif cmd == "help":
                    print_help()

                elif cmd == "config":
                    target = parts[1] if len(parts) > 1 else "all"
                    for arm_id in parse_arm_list(target):
                        print_config(system, arm_id)

                elif cmd == "version":
                    target = parts[1] if len(parts) > 1 else "all"
                    for arm_id in parse_arm_list(target):
                        print_versions(system, arm_id)

                elif cmd == "status":
                    target = parts[1] if len(parts) > 1 else "all"
                    for arm_id in parse_arm_list(target):
                        print_config(system, arm_id)
                        print_feedback(system, arm_id, with_status=True)

                elif cmd == "bus":
                    target = parts[1] if len(parts) > 1 else "all"
                    for arm_id in parse_arm_list(target):
                        print_bus_status(system, arm_id)

                elif cmd == "watch":
                    target = parts[1] if len(parts) > 1 else "all"
                    watch(system, parse_arm_list(target))

                elif cmd == "move" and len(parts) == 4:
                    a_id = int(parts[1])
                    joint = parts[2]
                    deg = float(parts[3])
                    system.get_arm(a_id).move_absolute(joint, deg)
                    print(f"[OK] ARM{a_id}.{joint} -> {deg:.2f}°")

                elif cmd == "jog" and len(parts) == 4:
                    a_id = int(parts[1])
                    joint = parts[2]
                    delta = float(parts[3])
                    system.get_arm(a_id).move_relative(joint, delta)
                    print(f"[OK] ARM{a_id}.{joint} jog {delta:+.2f}°")

                elif cmd == "pose" and len(parts) >= 3:
                    a_id = int(parts[1])
                    targets: Dict[str, float] = {}
                    for item in parts[2:]:
                        if "=" not in item:
                            raise ValueError("pose 参数格式必须是 j1=10")
                        k, v = item.split("=", 1)
                        targets[k] = float(v)
                    result = system.get_arm(a_id).move_many_absolute(targets)
                    print("[OK] pose 下发结果：")
                    for k, v in result.items():
                        print(f"  {k}: {v}")

                elif cmd == "home":
                    target = parts[1] if len(parts) > 1 else "all"
                    if not confirm("home 会让目标机械臂所有关节回 0°，请确认空间安全。"):
                        print("已取消。")
                        continue
                    for a_id in parse_arm_list(target):
                        system.get_arm(a_id).home()
                        print(f"[OK] ARM{a_id} home 下发完成。")

                elif cmd == "speed" and len(parts) == 4:
                    a_id = int(parts[1])
                    joint_token = parts[2]
                    rpm = float(parts[3])
                    arm = system.get_arm(a_id)
                    apply_to_joints(arm, joint_token, lambda key: print(f"[OK] {key}: {arm.set_joint_speed_limit(key, rpm):.2f} RPM"))

                elif cmd == "accel" and len(parts) == 4:
                    a_id = int(parts[1])
                    joint_token = parts[2]
                    acc = float(parts[3])
                    arm = system.get_arm(a_id)
                    apply_to_joints(arm, joint_token, lambda key: print(f"[OK] {key}: {arm.set_joint_accel(key, acc):.2f} RPM/s"))

                elif cmd == "current" and len(parts) == 4:
                    a_id = int(parts[1])
                    joint_token = parts[2]
                    current = float(parts[3])
                    arm = system.get_arm(a_id)
                    apply_to_joints(arm, joint_token, lambda key: print(f"[OK] {key}: {arm.set_joint_current_limit(key, current):.3f} A"))

                elif cmd == "clear":
                    target = parts[1] if len(parts) > 1 else "all"
                    for a_id in parse_arm_list(target):
                        print(f"[OK] ARM{a_id} clear: {system.get_arm(a_id).clear_faults()}")

                elif cmd == "setzero" and len(parts) == 3:
                    a_id = int(parts[1])
                    joint_token = parts[2]
                    arm = system.get_arm(a_id)
                    if not confirm("setzero 会把当前位置写为硬件零点，断电不丢。确认姿态正确。"):
                        print("已取消。")
                        continue
                    apply_to_joints(arm, joint_token, lambda key: print(f"[OK] {key}: {arm.set_zero(key)}"))

                elif cmd == "vel" and len(parts) == 5:
                    a_id = int(parts[1])
                    joint = parts[2]
                    rpm = float(parts[3])
                    sec = float(parts[4])
                    if not confirm(f"速度模式测试：ARM{a_id}.{joint} 以 {rpm}RPM 转动 {sec}s。确认空间安全。"):
                        print("已取消。")
                        continue
                    system.get_arm(a_id).speed_test(joint, rpm, sec)
                    print("[OK] 速度测试完成并已停止。")

                elif cmd == "holdstop":
                    target = parts[1] if len(parts) > 1 else "all"
                    for a_id in parse_arm_list(target):
                        system.get_arm(a_id).stop_all_speed()
                        print(f"[OK] ARM{a_id} 速度置 0。")

                elif cmd == "disable":
                    target = parts[1] if len(parts) > 1 else "all"
                    for a_id in parse_arm_list(target):
                        system.get_arm(a_id).disable_all()
                        print(f"[OK] ARM{a_id} CF 失能。")

                elif cmd == "stop":
                    print("[STOP] 全系统速度置 0 + CF 失能...")
                    system.stop_all_speed()
                    system.disable_all()
                    print("[OK] 全系统已停止并失能。")

                elif cmd == "head" and len(parts) >= 2:
                    if not head:
                        print("[头部云台] 未启用。请设置 HEAD_ENABLE=True。")
                        continue
                    if parts[1] == "shake":
                        head.send_cmd("1")
                    elif parts[1] == "nod":
                        head.send_cmd("2")
                    elif parts[1] == "x" and len(parts) == 3:
                        head.send_cmd(f"X{parts[2]}")
                    elif parts[1] == "y" and len(parts) == 3:
                        head.send_cmd(f"Y{parts[2]}")
                    else:
                        print("head 命令格式：head x 90 / head y 90 / head shake / head nod")

                elif cmd == "demo1":
                    if not confirm("demo1 会同时运动双臂，请确认空间安全。"):
                        print("已取消。")
                        continue
                    demo1(system, head)

                elif cmd == "demo3":
                    if not confirm("demo3 会同时运动双臂，请确认空间安全。"):
                        print("已取消。")
                        continue
                    demo3(system, head)

                else:
                    print("命令格式错误，输入 help 查看菜单。")

            except Exception as exc:
                print(f"[命令异常] {exc}")

    except KeyboardInterrupt:
        print("\n[Ctrl+C] 执行 stop...")
        try:
            system.stop_all_speed()
            system.disable_all()
        except Exception:
            pass

    except Exception as exc:
        print(f"[启动/运行异常] {exc}")
        return 1

    finally:
        if head:
            head.close()
        system.close()
        print("已关闭。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
