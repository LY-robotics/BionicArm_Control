"""Read-only board test. Run this before connecting mechanical load."""

import argparse

from sanpo_robot import SanpoBoard


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="For example COM30")
    args = parser.parse_args()

    board = SanpoBoard(args.port)
    try:
        print("BOARD_INFO:", board.board_info())
        board.heartbeat()
        for joint in range(1, 6):
            try:
                print("STATE", joint, board.get_state(joint))
            except RuntimeError as error:
                print("STATE", joint, error)
    finally:
        board.close()


if __name__ == "__main__":
    main()
