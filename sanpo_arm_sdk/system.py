"""Composition root for two F4 boards, two arms, and two separate grippers."""

from __future__ import annotations

from dataclasses import dataclass

from .end_effectors import (
    DualGripperController,
    DualGripperResult,
    GloriaGripper,
    GloriaGripperConfig,
    SimulatedGripper,
    UnavailableGripper,
)
from .factory import (
    create_can_controller,
    create_dual_simulated_controller,
)
from .motion.dual_arm_controller import DualArmController, DualMotionResult
from .protocol.can_motor_arm_lib import SerialUsbCanTransport


@dataclass(frozen=True)
class SystemConnectionResult:
    """Connection outcome while keeping arm and gripper results separate."""

    arms: DualMotionResult
    grippers: DualGripperResult

    @property
    def success(self) -> bool:
        """The system is usable whenever both arm controllers are connected."""
        return self.arms.success

    @property
    def all_requested_devices_connected(self) -> bool:
        return self.arms.success and self.grippers.success


class DualF4System:
    """Own shared F4 transports and expose independent device controllers.

    ``arms`` contains only motion-control behavior.  ``grippers`` contains only
    end-effector behavior.  This class exists solely to open and close the two
    physical F4 USB links in the correct order.
    """

    def __init__(
        self,
        arms: DualArmController,
        grippers: DualGripperController,
        *,
        transports: tuple[SerialUsbCanTransport, ...] = (),
    ) -> None:
        self.arms = arms
        self.grippers = grippers
        self._transports = transports

    def connect(self) -> SystemConnectionResult:
        arm_result = self.arms.connect()
        if not arm_result.success:
            return SystemConnectionResult(
                arms=arm_result,
                grippers=DualGripperResult(
                    False,
                    False,
                    "机械臂连接失败",
                    "机械臂连接失败",
                    left_available=bool(
                        getattr(self.grippers.left, "available", True)
                    ),
                    right_available=bool(
                        getattr(self.grippers.right, "available", True)
                    ),
                ),
            )

        # A shared F4 serial port can open even when its gripper CAN bus is
        # empty. Probe each configured gripper, but keep both arms usable when
        # one or both optional end effectors do not answer.
        gripper_connection = self.grippers.connect()
        gripper_probe, _states = self.grippers.refresh_both()
        gripper_result = DualGripperResult(
            left_success=(
                gripper_connection.left_success and gripper_probe.left_success
            ),
            right_success=(
                gripper_connection.right_success and gripper_probe.right_success
            ),
            left_error=(
                gripper_connection.left_error or gripper_probe.left_error
            ),
            right_error=(
                gripper_connection.right_error or gripper_probe.right_error
            ),
            left_available=gripper_connection.left_available,
            right_available=gripper_connection.right_available,
        )
        return SystemConnectionResult(arms=arm_result, grippers=gripper_result)

    def close(self) -> None:
        try:
            self.grippers.disconnect(disable=True)
        finally:
            try:
                self.arms.close()
            finally:
                for transport in self._transports:
                    transport.close()


def create_dual_f4_system(
    left_port: str,
    right_port: str,
    *,
    baudrate: int = 1_000_000,
    left_arm_channel: int = 1,
    left_gripper_channel: int = 2,
    right_arm_channel: int = 3,
    right_gripper_channel: int = 4,
    left_gripper_enabled: bool = True,
    right_gripper_enabled: bool = True,
    left_gripper_config: GloriaGripperConfig | None = None,
    right_gripper_config: GloriaGripperConfig | None = None,
    serial_timeout_s: float = 0.02,
    response_timeout_s: float = 0.08,
    use_host_id_offset: bool = True,
    debug: bool = False,
) -> DualF4System:
    """Create the production dual-F4 topology using ST channel routing."""

    if left_gripper_enabled and left_arm_channel == left_gripper_channel:
        raise ValueError("左臂和左夹爪必须使用不同 CAN 通道")
    if right_gripper_enabled and right_arm_channel == right_gripper_channel:
        raise ValueError("右臂和右夹爪必须使用不同 CAN 通道")

    left_transport = SerialUsbCanTransport(
        left_port,
        baudrate=baudrate,
        timeout=serial_timeout_s,
        debug=debug,
        usb_mode="standard",
    )
    right_transport = SerialUsbCanTransport(
        right_port,
        baudrate=baudrate,
        timeout=serial_timeout_s,
        debug=debug,
        usb_mode="standard",
    )

    left_arm = create_can_controller(
        left_port,
        profile="left",
        baudrate=baudrate,
        serial_timeout_s=serial_timeout_s,
        response_timeout_s=response_timeout_s,
        usb_mode="standard",
        channel=left_arm_channel,
        use_host_id_offset=use_host_id_offset,
        debug=debug,
        transport=left_transport,
    )
    right_arm = create_can_controller(
        right_port,
        profile="right",
        baudrate=baudrate,
        serial_timeout_s=serial_timeout_s,
        response_timeout_s=response_timeout_s,
        usb_mode="standard",
        channel=right_arm_channel,
        use_host_id_offset=use_host_id_offset,
        debug=debug,
        transport=right_transport,
    )
    arms = DualArmController(left_arm, right_arm)

    left_gripper = (
        GloriaGripper(
            left_transport.channel_endpoint(left_gripper_channel),
            name="left_gloria_gripper",
            config=left_gripper_config,
        )
        if left_gripper_enabled
        else UnavailableGripper(
            name="left_gripper",
            reason="左夹爪未启用或尚未安装",
        )
    )
    right_gripper = (
        GloriaGripper(
            right_transport.channel_endpoint(right_gripper_channel),
            name="right_gloria_gripper",
            config=right_gripper_config,
        )
        if right_gripper_enabled
        else UnavailableGripper(
            name="right_gripper",
            reason="右夹爪未启用或尚未安装",
        )
    )
    grippers = DualGripperController(left_gripper, right_gripper)
    return DualF4System(
        arms,
        grippers,
        transports=(left_transport, right_transport),
    )


def create_dual_simulated_system(
    *,
    left_gripper_enabled: bool = True,
    right_gripper_enabled: bool = True,
    left_gripper_config: GloriaGripperConfig | None = None,
    right_gripper_config: GloriaGripperConfig | None = None,
) -> DualF4System:
    """Create separate simulated arm and gripper controllers for UI tests."""

    arms = create_dual_simulated_controller()
    grippers = DualGripperController(
        (
            SimulatedGripper(
                name="simulated_left_gripper",
                config=left_gripper_config,
            )
            if left_gripper_enabled
            else UnavailableGripper(
                name="simulated_left_gripper",
                reason="左夹爪未启用或尚未安装",
            )
        ),
        (
            SimulatedGripper(
                name="simulated_right_gripper",
                config=right_gripper_config,
            )
            if right_gripper_enabled
            else UnavailableGripper(
                name="simulated_right_gripper",
                reason="右夹爪未启用或尚未安装",
            )
        ),
    )
    return DualF4System(arms, grippers)


__all__ = [
    "DualF4System",
    "SystemConnectionResult",
    "create_dual_f4_system",
    "create_dual_simulated_system",
]
