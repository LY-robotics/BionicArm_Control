"""Probe a Gloria-M CAN route without enabling or moving the gripper.

Close the dashboard before running this script because one COM port can only be
owned by one process. The probe sends only the Gloria active-status request.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sanpo_arm_sdk import GloriaGripper, GloriaGripperConfig
from sanpo_arm_sdk.protocol.can_motor_arm_lib import SerialUsbCanTransport


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe Gloria-M channels and report its feedback CAN ID"
    )
    parser.add_argument("port", help="F4 USB serial port, for example COM25")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument(
        "--channels",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4],
        help="ST channel numbers to probe; default: 1 2 3 4",
    )
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--debug", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    transport = SerialUsbCanTransport(
        args.port,
        baudrate=args.baudrate,
        timeout=min(args.timeout, 0.05),
        debug=args.debug,
        usb_mode="standard",
    )
    found = False
    try:
        transport.connect()
        print(
            f"Probing {args.port}, serial baud={args.baudrate}, "
            f"Motor ID=0x{args.motor_id:X}"
        )
        for channel in args.channels:
            gripper = GloriaGripper(
                transport.channel_endpoint(channel),
                name=f"probe_ch{channel}",
                config=GloriaGripperConfig(
                    motor_id=args.motor_id,
                    timeout_s=args.timeout,
                    startup_control_mode=None,
                ),
            )
            gripper.connect()
            try:
                state = gripper.refresh_state()
            except Exception as exc:
                print(f"  TX channel {channel}: no matching feedback ({exc})")
                continue
            found = True
            print(
                f"  TX channel {channel}: FOUND, "
                f"RX channel={gripper.feedback_channel}, "
                f"Master CAN ID=0x{gripper.feedback_can_id:03X}, "
                f"status={state.status}"
            )
    finally:
        transport.close()

    if not found:
        print(
            "No Gloria feedback. Check gripper power, CAN_H/CAN_L/GND, "
            "termination, Motor ID, and the physical CAN bitrate."
        )


if __name__ == "__main__":
    main()
