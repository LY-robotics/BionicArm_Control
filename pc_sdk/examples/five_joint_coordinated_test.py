"""五关节协同动作测试。先不带 --execute 运行可只检查连接和目标。"""

import argparse

from sanpo_robot import SanpoBoard


def main() -> None:
    parser = argparse.ArgumentParser(description="SANPO五关节协同动作测试")
    parser.add_argument("port", help="STM32 USB CDC串口，例如 COM33")
    parser.add_argument(
        "--angles", nargs=5, type=float,
        metavar=("J1", "J2", "J3", "J4", "J5"),
        default=[0.0, 0.0, 0.0, 0.0, 0.0],
        help="五关节目标角度（度）",
    )
    parser.add_argument("--duration", type=int, default=5000,
                        help="期望协同运动时间，单位ms")
    parser.add_argument("--execute", action="store_true",
                        help="确认实际发送运动命令")
    args = parser.parse_args()

    board = SanpoBoard(args.port)
    try:
        print("BOARD_INFO:", board.board_info())
        for joint_id in range(1, 6):
            print(f"J{joint_id}:", board.get_state(joint_id))
        print("目标:", args.angles, f"时长: {args.duration} ms")
        if not args.execute:
            print("安全预览完成；确认后加 --execute 执行。")
            return
        result = board.move_group(args.angles, args.duration, wait=True)
        print("协同动作完成:", result)
    except KeyboardInterrupt:
        board.stop_all()
        print("已中断并发送STOP_ALL。")
    finally:
        board.close()


if __name__ == "__main__":
    main()
