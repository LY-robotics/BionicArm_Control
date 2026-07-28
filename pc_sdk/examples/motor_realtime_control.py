"""SANPO单关节交互控制与实时状态监测程序。

启动示例：
    python examples/motor_realtime_control.py COM32

支持功能：
    1. 实时读取指定关节的角度、速度和故障状态；
    2. 输入目标角度和速度控制电机；
    3. 发送停止、回零和清故障命令；
    4. 后台周期发送心跳，避免STM32触发通信超时保护。

首次测试请让电机脱离机械负载，并准备急停或断电措施。
"""

import argparse
import threading
import time

from sanpo_robot import SanpoBoard


class RealtimeController:
    """封装后台状态监测和PC心跳。"""

    def __init__(self, board: SanpoBoard, joint_id: int) -> None:
        self.board = board
        self.joint_id = joint_id
        self.running = True
        self.io_lock = threading.Lock()
        self.worker = threading.Thread(
            target=self._monitor_loop,
            name="sanpo-monitor",
            daemon=True,
        )

    def start(self) -> None:
        self.worker.start()

    def stop(self) -> None:
        self.running = False
        self.worker.join(timeout=1.0)

    def _monitor_loop(self) -> None:
        while self.running:
            try:
                # 心跳周期小于STM32默认1秒超时时间。
                with self.io_lock:
                    self.board.heartbeat()
                    state = self.board.get_state(self.joint_id)

                print(
                    "\r"
                    f"[实时状态] J{state['joint_id']} | "
                    f"角度={state['angle_deg']:.2f}° | "
                    f"速度={state['speed_rpm']:.2f}rpm | "
                    f"故障=0x{state['fault']:02X}",
                    end="",
                    flush=True,
                )
            except RuntimeError as error:
                print(f"\r[实时状态] {error}", end="", flush=True)
            except Exception as error:
                print(f"\r[通信异常] {error}", end="", flush=True)

            time.sleep(0.25)

    def move(self, angle_deg: float, speed_rpm: float) -> None:
        with self.io_lock:
            self.board.move_joint(
                joint_id=self.joint_id,
                angle_deg=angle_deg,
                speed_rpm=speed_rpm,
            )

    def stop_all(self) -> None:
        with self.io_lock:
            self.board.stop_all()

    def home(self) -> None:
        with self.io_lock:
            self.board.home(self.joint_id)

    def clear_fault(self) -> None:
        with self.io_lock:
            self.board.clear_fault(self.joint_id)


def print_help() -> None:
    print()
    print("可用命令：")
    print("  m <角度> <速度>   控制当前关节运动，例如 m 1 0.2")
    print("  s                 停止全部电机")
    print("  h                 当前关节回零")
    print("  c                 清除当前关节故障")
    print("  j <编号>           切换监测关节，例如 j 2")
    print("  q                 停止并退出")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SANPO单关节实时控制与状态监测"
    )
    parser.add_argument(
        "port",
        help="板卡COM口，例如 COM32",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=1,
        help="启动时监测的关节编号，默认1",
    )
    args = parser.parse_args()

    board = SanpoBoard(args.port)
    controller = RealtimeController(board, args.joint)

    try:
        print("BOARD_INFO:", board.board_info())
        print(f"当前监测关节：J{args.joint}")
        print_help()
        controller.start()

        while True:
            command_line = input("\n请输入命令：").strip()
            if not command_line:
                continue

            fields = command_line.split()
            command = fields[0].lower()

            try:
                if command == "m":
                    if len(fields) != 3:
                        print("格式：m <角度> <速度>")
                        continue

                    angle = float(fields[1])
                    speed = float(fields[2])

                    if abs(angle) > 5.0:
                        print("安全限制：实时测试角度绝对值不能超过5°。")
                        continue
                    if speed <= 0.0 or speed > 1.0:
                        print("安全限制：实时测试速度必须为0~1rpm。")
                        continue

                    print(
                        f"即将控制J{controller.joint_id}移动到"
                        f"{angle:.2f}°，速度{speed:.2f}rpm。"
                    )
                    confirm = input("确认执行？输入 YES 执行：").strip()
                    if confirm != "YES":
                        print("已取消。")
                        continue

                    controller.move(angle, speed)
                    print("运动命令已发送。")

                elif command == "s":
                    controller.stop_all()
                    print("STOP_ALL 已发送。")

                elif command == "h":
                    controller.home()
                    print(f"J{controller.joint_id} 回零命令已发送。")

                elif command == "c":
                    controller.clear_fault()
                    print(f"J{controller.joint_id} 清故障命令已发送。")

                elif command == "j":
                    if len(fields) != 2:
                        print("格式：j <关节编号>")
                        continue
                    joint_id = int(fields[1])
                    if not 1 <= joint_id <= 5:
                        print("关节编号必须为1~5。")
                        continue
                    controller.joint_id = joint_id
                    print(f"已切换到监测关节J{joint_id}。")

                elif command == "q":
                    break

                elif command in {"help", "?"}:
                    print_help()

                else:
                    print("未知命令，输入 help 查看帮助。")

            except (ValueError, RuntimeError) as error:
                print(f"\n命令执行失败：{error}")

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        try:
            controller.stop_all()
            print("\nSTOP_ALL 已发送。")
        except Exception as error:
            print(f"\nSTOP_ALL 发送失败：{error}")
        controller.stop()
        board.close()


if __name__ == "__main__":
    main()
