"""低速小角度单关节测试程序。

默认参数：
    关节1
    目标角度：1.0°
    速度：0.2 rpm

使用 --execute 才会真正发送运动命令。
首次测试前请确认电机已脱离机械负载，并准备急停或断电措施。
"""

import argparse
import time

from sanpo_robot import SanpoBoard


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SANPO单关节低速小角度测试"
    )
    parser.add_argument(
        "port",
        help="板卡COM口，例如 COM32",
    )
    parser.add_argument(
        "--joint",
        type=int,
        default=1,
        help="关节编号，默认1",
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=1.0,
        help="目标角度，单位度，默认1.0",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.2,
        help="运动速度，单位rpm，默认0.2",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="确认发送运动命令；不加此参数时只进行检查",
    )
    args = parser.parse_args()

    if abs(args.angle) > 5.0:
        parser.error("小角度测试要求目标角度绝对值不超过5度")
    if args.speed <= 0.0 or args.speed > 1.0:
        parser.error("小角度测试要求速度范围为0<speed<=1.0rpm")

    board = SanpoBoard(args.port)

    try:
        print("BOARD_INFO:", board.board_info())
        board.heartbeat()
        print("HEARTBEAT: OK")

        try:
            print("BEFORE STATE:", board.get_state(args.joint))
        except RuntimeError as error:
            print("BEFORE STATE:", error)

        if not args.execute:
            print()
            print("当前为检查模式，没有发送运动命令。")
            print("确认机械和电源安全后，追加 --execute 才会执行运动。")
            return

        print()
        print(
            f"即将控制关节{args.joint}移动到"
            f"{args.angle:.2f}°，速度{args.speed:.2f}rpm。"
        )
        input("确认安全后按Enter继续，按Ctrl+C取消：")

        board.move_joint(
            joint_id=args.joint,
            angle_deg=args.angle,
            speed_rpm=args.speed,
        )
        print("MOVE_JOINT: command accepted")

        # 控制期间持续发送心跳，避免STM32触发通信超时保护。
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            board.heartbeat()
            try:
                state = board.get_state(args.joint)
                print("STATE:", state)
            except RuntimeError as error:
                print("STATE:", error)
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n用户取消测试。")
    finally:
        # 无论测试成功、失败还是Ctrl+C，都发送全停命令。
        try:
            board.stop_all()
            print("STOP_ALL: sent")
        except Exception as error:
            print("STOP_ALL failed:", error)
        board.close()


if __name__ == "__main__":
    main()
