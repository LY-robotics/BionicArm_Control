"""Interactive menu for the complete arm-control chain.

The menu is intentionally kept outside the SDK package. It demonstrates how an
application should call the public controller API without depending on CAN
frame details.
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, Iterable, Optional

from sanpo_arm_sdk import (
    JOINT_KEYS,
    OK,
    ArmController,
    create_can_controller,
    create_simulated_controller,
    list_serial_ports,
)
from sanpo_arm_sdk.errors import err_text


def prompt_values(
    prompt: str,
    *,
    allowed_counts: Optional[Iterable[int]] = None,
) -> list[float]:
    raw = input(prompt).strip().replace(",", " ").split()
    values = [float(item) for item in raw]
    if allowed_counts is not None and len(values) not in set(allowed_counts):
        expected = "/".join(str(value) for value in allowed_counts)
        raise ValueError(f"需要输入 {expected} 个数值")
    return values


def prompt_bool(prompt: str, *, default: bool = True) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    value = input(f"{prompt} {suffix}: ").strip().lower()
    if not value:
        return default
    if value in {"y", "yes", "1", "true", "是"}:
        return True
    if value in {"n", "no", "0", "false", "否"}:
        return False
    raise ValueError("请输入 y 或 n")


def print_result(action: str, err: int) -> None:
    print(f"{action}: {err} ({err_text(err)})")


def choose_serial_port(label: str) -> str:
    ports = list_serial_ports()
    print(f"\n{label} 可用串口:")
    if ports:
        for index, port in enumerate(ports, start=1):
            print(f"  {index}. {port}")
    else:
        print("  未自动发现串口，可直接输入 COMx 或 /dev/ttyACMx")
    value = input(f"{label} 串口: ").strip()
    if value.isdigit() and ports:
        index = int(value) - 1
        if 0 <= index < len(ports):
            return ports[index]
    if not value:
        raise ValueError("串口不能为空")
    return value


def select_profiles(args: argparse.Namespace) -> list[str]:
    if args.arm:
        return ["left", "right"] if args.arm == "dual" else [args.arm]
    if args.simulate:
        return ["right"]

    print(
        """
启动方式
  1. 右臂（一个 F4 串口）
  2. 左臂（一个 F4 串口）
  3. 双臂（两个 F4 串口）
"""
    )
    choice = input("选择: ").strip()
    if choice == "1":
        return ["right"]
    if choice == "2":
        return ["left"]
    if choice == "3":
        return ["left", "right"]
    raise ValueError("无效启动方式")


def create_controllers(args: argparse.Namespace) -> Dict[str, ArmController]:
    controllers: Dict[str, ArmController] = {}
    for profile in select_profiles(args):
        if args.simulate:
            controller = create_simulated_controller(profile=profile)
        else:
            configured_port = (
                args.left_port if profile == "left" else args.right_port
            )
            port = configured_port or choose_serial_port(
                "左臂/STM32(1)" if profile == "left" else "右臂/STM32(2)"
            )
            channel = (
                args.left_channel if profile == "left" else args.right_channel
            )
            controller = create_can_controller(
                port,
                profile=profile,
                baudrate=args.baudrate,
                serial_timeout_s=args.serial_timeout,
                response_timeout_s=args.response_timeout,
                usb_mode=args.usb_mode,
                channel=channel,
                use_host_id_offset=not args.no_host_id_offset,
                debug=args.debug,
            )
        controllers[profile] = controller
    return controllers


def connect_controllers(controllers: Dict[str, ArmController]) -> bool:
    connected: list[ArmController] = []
    for profile, controller in controllers.items():
        err = controller.connect()
        print_result(f"{profile} connect", err)
        if err != OK:
            for item in connected:
                item.close()
            return False
        connected.append(controller)

        sync_err = controller.sync_state()
        print_result(f"{profile} sync joint angles", sync_err)
        if sync_err != OK:
            print("  警告: 未取得全部关节角度，MoveJ/MoveCart 将拒绝执行。")
    return True


def show_arm_config(profile: str, arm: ArmController) -> None:
    backend = arm.hardware
    port = getattr(backend, "port", "simulation")
    bus = getattr(backend, "bus", None)
    usb_mode = getattr(bus, "usb_mode", "simulation")
    channel = getattr(bus, "channel", "-")
    print(
        f"\n机械臂={profile}  名称={arm.name}  串口={port}  "
        f"USB模式={usb_mode}  CAN通道={channel}"
    )
    print("关节  电机ID  减速比  方向  逻辑限位(deg)")
    for key in JOINT_KEYS:
        cfg = arm.joints[key]
        print(
            f"{key.upper():<5} {cfg['id']:<7} {cfg['ratio']:<7.2f} "
            f"{cfg['direction']:<5.0f} {cfg['min']:>7.1f} .. {cfg['max']:<7.1f}"
        )


def show_feedback(arm: ArmController) -> None:
    err, feedback = arm.refresh_all()
    print_result("refresh all", err)
    print("关节  角度(deg)    速度(rpm)    Q轴电流(A)")
    for key in JOINT_KEYS:
        data = feedback.get(key)
        if not data:
            print(f"{key.upper():<5} ---          ---          ---")
            continue
        values = (
            data.get("angle_deg"),
            data.get("speed_rpm"),
            data.get("current_a"),
        )
        texts = ["---" if value is None else f"{value:.3f}" for value in values]
        print(f"{key.upper():<5} {texts[0]:<12} {texts[1]:<12} {texts[2]:<12}")


def show_current_pose(arm: ArmController) -> None:
    err, pose = arm.forward_pose()
    print_result("current TCP pose", err)
    if err == OK and pose is not None:
        print(
            f"  position_mm = [{pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}]\n"
            f"  pitch_deg   = {pose.pitch_deg:.3f}"
        )


def show_motor_status(arm: ArmController) -> None:
    key = input("关节 j1..j5，输入 all 查看全部: ").strip().lower()
    keys = JOINT_KEYS if key == "all" else (key,)
    for item in keys:
        err, status = arm.read_motor_status(item)
        print_result(f"{item} motor status", err)
        if status:
            for name, value in status.items():
                print(f"  {name}: {value}")


def move_cartesian(arm: ArmController) -> None:
    target = prompt_values(
        "输入 x y z [pitch] [j5]，位置单位 mm、角度单位 deg: ",
        allowed_counts=(3, 4, 5),
    )
    speed = float(input("关节轨迹速度上限 deg/s [10]: ").strip() or "10")
    accel = float(input("关节轨迹加速度上限 deg/s^2 [20]: ").strip() or "20")
    blocking = prompt_bool("阻塞等待运动完成", default=True)
    if not prompt_bool("确认执行笛卡尔坐标运动", default=False):
        print("已取消")
        return
    err = arm.MoveCart(target, speed=speed, accel=accel, blocking=blocking)
    print_result("MoveCart", err)
    if arm.last_ik_result is not None:
        joints = ", ".join(f"{value:.3f}" for value in arm.last_ik_result.q_deg)
        print(f"  IK关节角 = [{joints}]")
    if arm.last_trajectory is not None:
        print(
            f"  轨迹点数={arm.last_trajectory.point_count}, "
            f"规划时长={arm.last_trajectory.time_s[-1]:.3f}s"
        )


def move_joint_trajectory(arm: ArmController) -> None:
    target = prompt_values(
        "输入 j1 j2 j3 j4 j5 (deg): ",
        allowed_counts=(5,),
    )
    speed = float(input("关节轨迹速度上限 deg/s [10]: ").strip() or "10")
    accel = float(input("关节轨迹加速度上限 deg/s^2 [20]: ").strip() or "20")
    blocking = prompt_bool("阻塞等待运动完成", default=True)
    if not prompt_bool("确认执行关节轨迹运动", default=False):
        print("已取消")
        return
    err = arm.MoveJ(target, speed=speed, accel=accel, blocking=blocking)
    print_result("MoveJ", err)
    if arm.last_trajectory is not None:
        print(
            f"  轨迹点数={arm.last_trajectory.point_count}, "
            f"规划时长={arm.last_trajectory.time_s[-1]:.3f}s"
        )


def move_single_joint(arm: ArmController, *, relative: bool) -> None:
    key = input("关节 j1..j5: ").strip().lower()
    label = "增量角度" if relative else "目标角度"
    value = float(input(f"{label} deg: ").strip())
    if not prompt_bool("确认执行单关节运动", default=False):
        print("已取消")
        return
    if relative:
        err = arm.move_relative(key, value)
        print_result("move_relative", err)
    else:
        err = arm.move_absolute(key, value)
        print_result("move_absolute", err)


def direct_pose(arm: ArmController) -> None:
    target = prompt_values(
        "输入 j1 j2 j3 j4 j5 (deg): ",
        allowed_counts=(5,),
    )
    if not prompt_bool("直接下发五关节目标（不做轨迹插补）", default=False):
        print("已取消")
        return
    print_result("set_pose", arm.set_pose(*target))


def set_joint_parameter(arm: ArmController, *, acceleration: bool) -> None:
    key = input("关节 j1..j5: ").strip().lower()
    unit = "rpm/s" if acceleration else "rpm"
    value = float(input(f"数值 ({unit}): ").strip())
    if acceleration:
        print_result("set_accel", arm.set_accel(key, value))
    else:
        print_result("set_speed", arm.set_speed(key, value))


def preview_ik(arm: ArmController) -> None:
    target = prompt_values(
        "输入 x y z [pitch] [j5]，仅计算不运动: ",
        allowed_counts=(3, 4, 5),
    )
    err, result = arm.preview_ik(target)
    print_result("IK preview", err)
    if result is not None:
        joints = ", ".join(f"{value:.3f}" for value in result.q_deg)
        print(f"  success={result.success}")
        print(f"  q_deg=[{joints}]")
        print(f"  position_error_norm_mm={result.error_norm_mm:.6f}")
        print(f"  message={result.message}")


def preview_fk(arm: ArmController) -> None:
    value = input("输入 j1..j5，直接回车则读取当前关节: ").strip()
    joints = None
    if value:
        joints = [float(item) for item in value.replace(",", " ").split()]
    err, pose = arm.forward_pose(joints)
    print_result("FK preview", err)
    if pose is not None:
        print(
            f"  position_mm=[{pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}]\n"
            f"  pitch_deg={pose.pitch_deg:.3f}"
        )


def set_zero_menu(arm: ArmController) -> None:
    key = input("输入 j1..j5 或 all: ").strip().lower()
    confirmation = input(
        "设零会改变电机坐标系。确认机械位置正确后输入 ZERO: "
    ).strip()
    if confirmation != "ZERO":
        print("已取消")
        return
    err = arm.set_zero_all() if key == "all" else arm.set_zero(key)
    print_result("set_zero", err)


def brake_menu(arm: ArmController) -> None:
    key = input("关节 j1..j5: ").strip().lower()
    closed = prompt_bool("闭合刹车", default=True)
    print_result("set_brake", arm.set_brake(key, closed))


def read_brake_menu(arm: ArmController) -> None:
    key = input("关节 j1..j5: ").strip().lower()
    err, closed = arm.read_brake(key)
    print_result("read_brake", err)
    if closed is not None:
        print("  brake =", "闭合" if closed else "断开")


def print_mapping(data: dict) -> None:
    for name, value in data.items():
        print(f"  {name}: {value}")


def motor_protocol_menu(arm: ArmController) -> None:
    """Expose protocol-library maintenance commands in one contained submenu."""
    while True:
        print(
            """
------------- 电机协议高级功能 -------------
 1. 读取版本 A0
 2. 读取电机参数 B0
 3. 读取紧凑反馈 A4
 4. 读取/设置位置环和速度环 PID
 5. 设置最大 Q 轴电流 B3
 6. 设置 Q 轴电流斜率 B4
 7. 设置 Q 轴电流模式 C0（会产生力矩）
 8. 设置关节速度模式 C1（会持续运动，输入 0 停止）
 9. 单电机最短路径回零 C4（会运动）
 0. 返回主菜单
--------------------------------------------
"""
        )
        choice = input("选择高级功能: ").strip()
        if choice == "0":
            return
        try:
            key = input("关节 j1..j5: ").strip().lower()
            if choice in {"1", "2", "3"}:
                kind = {"1": "version", "2": "params", "3": "compact"}[choice]
                err, data = arm.read_motor_info(key, kind)
                print_result(f"read {kind}", err)
                if data:
                    print_mapping(data)
            elif choice == "4":
                print(
                    "增益名称: position_kp / position_ki / speed_kp / speed_ki"
                )
                gain = input("增益名称: ").strip().lower()
                raw = input("新值，直接回车表示只读取: ").strip()
                value = None if not raw else float(raw)
                err, result = arm.read_or_set_gain(key, gain, value)
                print_result("read/set gain", err)
                if result is not None:
                    print(f"  {gain}={result}")
            elif choice == "5":
                value = float(input("最大 Q 轴电流 A: ").strip())
                if prompt_bool("确认修改最大电流限制", default=False):
                    print_result("set max current", arm.set_max_current(key, value))
            elif choice == "6":
                value = float(input("Q 轴电流斜率 A/s: ").strip())
                if prompt_bool("确认修改电流斜率", default=False):
                    print_result(
                        "set current slope",
                        arm.set_current_slope(key, value),
                    )
            elif choice == "7":
                value = float(input("目标 Q 轴电流 A: ").strip())
                confirm = input("该命令会直接产生力矩，输入 CURRENT 确认: ").strip()
                if confirm != "CURRENT":
                    print("已取消")
                    continue
                err, feedback = arm.set_q_current(key, value)
                print_result("set q current", err)
                if feedback is not None:
                    print(f"  feedback_current_a={feedback}")
            elif choice == "8":
                value = float(input("目标关节速度 rpm，输入 0 停止: ").strip())
                confirm = input("速度模式会持续运动，输入 SPEED 确认: ").strip()
                if confirm != "SPEED":
                    print("已取消")
                    continue
                err, feedback = arm.set_speed_mode(key, value)
                print_result("set speed mode", err)
                if feedback is not None:
                    print(f"  feedback_joint_rpm={feedback}")
            elif choice == "9":
                confirm = input("单电机会立即运动，输入 HOME 确认: ").strip()
                if confirm != "HOME":
                    print("已取消")
                    continue
                print_result("motor shortest home", arm.go_home_shortest(key))
            else:
                print("无效高级功能编号")
        except (TypeError, ValueError) as exc:
            print(f"输入错误: {exc}")


def print_menu(active: str, arm: ArmController, arm_count: int) -> None:
    moving = "运动中" if arm.is_moving else "空闲"
    switch_line = "  1. 切换当前机械臂\n" if arm_count > 1 else ""
    print(
        f"""
================ SANPO 机械臂控制菜单 ================
当前机械臂: {active}    状态: {moving}
{switch_line}  2. 查看串口、ID、减速比与限位
  3. 列出系统串口

 10. 刷新全部关节反馈
 11. 查看当前 TCP 坐标（正运动学）
 12. 查看电机详细状态/故障

 20. 输入 TCP 坐标运动（坐标 -> IK -> 轨迹 -> CAN）
 21. 五关节 MoveJ 轨迹运动
 22. 单关节绝对运动
 23. 单关节相对运动
 24. 五关节回零位
 25. 直接下发五关节目标（无轨迹插补）
 26. 停止轨迹并失能全部电机

 30. 设置单关节位置速度上限
 31. 设置单关节加速度上限
 32. 应用全部默认速度/加速度

 40. 逆运动学预览（不运动）
 41. 正运动学预览（不运动）

 50. 清除全部电机故障
 51. 设置单关节/全部关节零点
 52. 设置单关节刹车
 53. 读取单关节刹车
  60. 电机协议高级功能（版本/参数/PID/电流/速度模式）

  0. 失能、关闭串口并退出
=======================================================
"""
    )


def run_menu(controllers: Dict[str, ArmController]) -> None:
    active = "right" if "right" in controllers else next(iter(controllers))
    while True:
        arm = controllers[active]
        print_menu(active, arm, len(controllers))
        choice = input("选择功能: ").strip()
        try:
            if choice == "0":
                return
            if choice == "1" and len(controllers) > 1:
                requested = input("输入 left 或 right: ").strip().lower()
                if requested not in controllers:
                    raise ValueError("该机械臂未在启动时连接")
                active = requested
            elif choice == "2":
                show_arm_config(active, arm)
            elif choice == "3":
                print("可用串口:", list_serial_ports())
            elif choice == "10":
                show_feedback(arm)
            elif choice == "11":
                show_current_pose(arm)
            elif choice == "12":
                show_motor_status(arm)
            elif choice == "20":
                move_cartesian(arm)
            elif choice == "21":
                move_joint_trajectory(arm)
            elif choice == "22":
                move_single_joint(arm, relative=False)
            elif choice == "23":
                move_single_joint(arm, relative=True)
            elif choice == "24":
                if prompt_bool("确认执行五关节回零位运动", default=False):
                    print_result("home", arm.home())
            elif choice == "25":
                direct_pose(arm)
            elif choice == "26":
                print_result("stop and disable", arm.disable_all())
            elif choice == "30":
                set_joint_parameter(arm, acceleration=False)
            elif choice == "31":
                set_joint_parameter(arm, acceleration=True)
            elif choice == "32":
                if prompt_bool("确认写入全部默认速度/加速度", default=False):
                    print_result("configure defaults", arm.configure_defaults())
            elif choice == "40":
                preview_ik(arm)
            elif choice == "41":
                preview_fk(arm)
            elif choice == "50":
                if prompt_bool("确认清除全部电机故障", default=False):
                    print_result("clear faults", arm.clear_faults())
            elif choice == "51":
                set_zero_menu(arm)
            elif choice == "52":
                brake_menu(arm)
            elif choice == "53":
                read_brake_menu(arm)
            elif choice == "60":
                motor_protocol_menu(arm)
            else:
                print("无效功能编号")
        except (TypeError, ValueError) as exc:
            print(f"输入错误: {exc}")
        except KeyboardInterrupt:
            print("\n当前操作已取消")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SANPO 双 F4 五轴机械臂菜单")
    parser.add_argument("--arm", choices=("left", "right", "dual"))
    parser.add_argument("--left-port", help="左臂对应 F4 的 USB CDC 串口")
    parser.add_argument("--right-port", help="右臂对应 F4 的 USB CDC 串口")
    parser.add_argument(
        "--usb-mode",
        choices=("advanced", "standard"),
        default="advanced",
        help="advanced=沿用 can_motor_arm_lib；standard=V4.1+ ST 帧",
    )
    parser.add_argument("--left-channel", type=int, default=0)
    parser.add_argument("--right-channel", type=int, default=0)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--serial-timeout", type=float, default=0.02)
    parser.add_argument("--response-timeout", type=float, default=0.08)
    parser.add_argument("--no-host-id-offset", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="不打开串口，用仿真后端验证运动控制链路",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        controllers = create_controllers(args)
    except (TypeError, ValueError) as exc:
        print(f"启动配置错误: {exc}")
        return 2

    if not connect_controllers(controllers):
        return 1

    try:
        run_menu(controllers)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在失能并关闭串口")
    finally:
        for profile, arm in controllers.items():
            if arm.connected:
                print_result(f"{profile} disable", arm.disable_all())
                print_result(f"{profile} close", arm.close())
    return 0


if __name__ == "__main__":
    sys.exit(main())
