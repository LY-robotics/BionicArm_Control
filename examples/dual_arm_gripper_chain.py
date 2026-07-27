#!/usr/bin/env python3
"""Coordinate-to-arm motion followed by one independent gripper command."""

from __future__ import annotations

import argparse

from sanpo_arm_sdk import (
    GloriaGripperConfig,
    GripperCalibration,
    OK,
    create_dual_f4_system,
    create_dual_simulated_system,
)
from sanpo_arm_sdk.kinematics import forward_kinematics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SANPO 单侧坐标运动 + Gloria-M 夹爪整链路示例"
    )
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--left-port", default="COM8")
    parser.add_argument("--right-port", default="COM9")
    parser.add_argument("--left-arm-channel", type=int, default=1)
    parser.add_argument("--left-gripper-channel", type=int, default=2)
    parser.add_argument("--right-arm-channel", type=int, default=1)
    parser.add_argument("--right-gripper-channel", type=int, default=2)
    parser.add_argument(
        "--target",
        type=float,
        nargs=5,
        metavar=("X", "Y", "Z", "YAW", "J5"),
    )
    parser.add_argument("--speed", type=float, default=10.0)
    parser.add_argument("--accel", type=float, default=20.0)
    parser.add_argument(
        "--opening",
        type=float,
        default=0.5,
        help="0=fully closed, 1=fully open",
    )
    parser.add_argument("--gripper-speed", type=float, default=0.2)
    parser.add_argument("--gripper-id", type=int, default=1)
    parser.add_argument("--master-can-id", type=int, default=0)
    parser.add_argument("--closed-position", type=float, default=0.0)
    parser.add_argument("--open-position", type=float, default=2.7)
    return parser


def default_target() -> list[float]:
    joints = [-10.0, 15.0, 20.0, 30.0, 5.0]
    pose = forward_kinematics(joints)
    return [pose.x, pose.y, pose.z, pose.yaw_deg, joints[4]]


def main() -> None:
    args = build_parser().parse_args()
    if not args.execute:
        raise SystemExit(
            "未执行运动。确认环境和急停后，显式增加 --execute。"
        )
    calibration = GripperCalibration(
        closed_position_rad=args.closed_position,
        open_position_rad=args.open_position,
    )
    gripper_config = GloriaGripperConfig(
        motor_id=args.gripper_id,
        master_can_id=args.master_can_id,
        calibration=calibration,
    )
    system = (
        create_dual_simulated_system(
            left_gripper_config=gripper_config,
            right_gripper_config=gripper_config,
        )
        if args.simulate
        else create_dual_f4_system(
            args.left_port,
            args.right_port,
            left_arm_channel=args.left_arm_channel,
            left_gripper_channel=args.left_gripper_channel,
            right_arm_channel=args.right_arm_channel,
            right_gripper_channel=args.right_gripper_channel,
            left_gripper_config=gripper_config,
            right_gripper_config=gripper_config,
        )
    )
    connection = system.connect()
    if not connection.success:
        system.close()
        raise SystemExit(f"连接失败: {connection}")

    arm = system.arms.left if args.side == "left" else system.arms.right
    gripper = (
        system.grippers.left
        if args.side == "left"
        else system.grippers.right
    )
    try:
        sync = system.arms.sync_state()
        if not sync.success:
            raise RuntimeError(f"机械臂反馈同步失败: {sync}")
        gripper.refresh_state()
        target = list(args.target) if args.target is not None else default_target()
        result = arm.MoveCartRecommended(
            target,
            speed=args.speed,
            accel=args.accel,
            blocking=True,
        )
        if result != OK:
            raise RuntimeError(f"坐标运动失败，错误码 {result}")
        gripper.enable()
        state = gripper.move_normalized(
            args.opening,
            args.gripper_speed,
        )
        print("目标坐标:", target)
        print("夹爪状态:", state)
    finally:
        system.arms.stop(disable=True)
        system.close()


if __name__ == "__main__":
    main()
