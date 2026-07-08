# -*- coding: utf-8 -*-
"""
test_single_motor_channel_v3.py

单电机 Channel 模式测试。
使用 V3 可变长度 Normal/Passthrough 帧。

只读：
    python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --no-move --debug

速度测试：
    python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --rpm 5 --seconds 2 --debug
"""

from __future__ import annotations

import argparse
import sys
import time

from dualcan_arm_control_lib_v3 import CanMotor, SerialDualCanNormalBus


def main() -> int:
    ap = argparse.ArgumentParser(description="单电机 Channel 模式测试 V3")
    ap.add_argument("--port", default="COM17")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--channel", type=int, default=1)
    ap.add_argument("--motor-id", type=int, default=34)
    ap.add_argument("--rpm", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--no-move", action="store_true")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    print("\n========== 单电机 Channel 模式测试 V3 ==========")
    print(f"COM       : {args.port}")
    print(f"Baudrate  : {args.baudrate}")
    print(f"CAN通道   : {args.channel}")
    print(f"Motor ID  : {args.motor_id} / 0x{args.motor_id:X}")
    print(f"TX CAN ID : 0x{0x100 | args.motor_id:X}")
    print("===============================================\n")

    try:
        with SerialDualCanNormalBus(args.port, args.baudrate, debug=args.debug) as bus:
            motor = CanMotor(bus.channel(args.channel), args.motor_id)

            print("[1] 读取版本 A0")
            ver = motor.read_version()
            print(f"    OK: boot={ver.boot}, app={ver.app}, hw={ver.hardware}, can_proto={ver.can_protocol}")

            print("\n[2] 读取状态 AE")
            st = motor.read_status()
            print(
                f"    OK: voltage={st.bus_voltage_v:.2f}V, current={st.bus_current_a:.2f}A, "
                f"temp={st.temperature_c}℃, mode={st.mode}({st.mode_name()}), fault=0x{st.fault:02X}"
            )

            print("\n[3] 清故障 AF")
            fault = motor.clear_fault()
            print(f"    clear_fault 返回: 0x{fault:02X}")

            print("\n[4] 读取角度/速度/电流")
            angle = motor.read_angle()
            speed = motor.read_speed_rpm()
            current = motor.read_q_current_a()
            print(
                f"    angle_single={angle.single_turn_deg:.2f}°, "
                f"angle_multi={angle.multi_turn_deg:.2f}°, "
                f"speed={speed:.2f}RPM, current={current:.3f}A"
            )

            if args.no_move:
                print("\n--no-move：只读测试完成。")
                return 0

            print("\n[5] 速度模式测试")
            print(f"    目标速度: {args.rpm} RPM")
            print(f"    持续时间: {args.seconds} s")
            print("    输入 yes 才会执行：")
            if input("confirm> ").strip().lower() != "yes":
                print("已取消。")
                return 0

            try:
                actual = motor.set_speed_rpm(args.rpm)
                print(f"    C1 回复实际速度: {actual:.2f} RPM")
                time.sleep(max(0.0, args.seconds))
            finally:
                print("    发送 C1 0 停止...")
                try:
                    stop_rpm = motor.stop_speed()
                    print(f"    停止回复速度: {stop_rpm:.2f} RPM")
                except Exception as exc:
                    print(f"    停止异常: {exc}")

            print("\n[6] 是否 CF 失能？输入 yes 失能，否则保持当前状态。")
            if input("disable?> ").strip().lower() == "yes":
                st2 = motor.disable()
                print(f"    已失能: mode={st2.mode}({st2.mode_name()}), fault=0x{st2.fault:02X}")

            print("\n测试完成。")
            return 0

    except Exception as exc:
        print(f"\n[异常] {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
