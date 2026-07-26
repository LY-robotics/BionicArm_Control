"""Convenience constructors for real and simulated arm controllers."""

from .config import get_arm_joint_configs
from .hardware.can_backend import SanpoCanBackend
from .hardware.simulated_backend import SimulatedArmBackend
from .motion.arm_controller import ArmController
from .motion.dual_arm_controller import DualArmController
from .protocol.can_motor_arm_lib import SerialUsbCanTransport


def create_can_controller(
    port: str,
    *,
    profile: str,
    baudrate: int = 1_000_000,
    serial_timeout_s: float = 0.02,
    response_timeout_s: float = 0.08,
    usb_mode: str = "advanced",
    channel: int = 0,
    use_host_id_offset: bool = True,
    debug: bool = False,
    transport: SerialUsbCanTransport | None = None,
) -> ArmController:
    """Create one controller for one arm/F4 serial port."""
    profile_key = profile.strip().lower()
    backend = SanpoCanBackend(
        port=port,
        joints=get_arm_joint_configs(profile_key),
        name=f"{profile_key}_arm",
        baudrate=baudrate,
        serial_timeout_s=serial_timeout_s,
        response_timeout_s=response_timeout_s,
        usb_mode=usb_mode,
        channel=channel,
        use_host_id_offset=use_host_id_offset,
        debug=debug,
        transport=transport,
    )
    return ArmController(backend)


def create_simulated_controller(*, profile: str) -> ArmController:
    """Create an API-compatible controller that never touches serial hardware."""
    profile_key = profile.strip().lower()
    backend = SimulatedArmBackend(
        get_arm_joint_configs(profile_key),
        name=f"simulated_{profile_key}_arm",
    )
    return ArmController(backend)


def create_dual_can_controller(
    left_port: str,
    right_port: str,
    *,
    baudrate: int = 1_000_000,
    usb_mode: str = "advanced",
    left_channel: int = 0,
    right_channel: int = 0,
    serial_timeout_s: float = 0.02,
    response_timeout_s: float = 0.08,
    use_host_id_offset: bool = True,
    debug: bool = False,
) -> DualArmController:
    """Create one high-level controller for both independent F4 serial ports."""

    left = create_can_controller(
        left_port,
        profile="left",
        baudrate=baudrate,
        serial_timeout_s=serial_timeout_s,
        response_timeout_s=response_timeout_s,
        usb_mode=usb_mode,
        channel=left_channel,
        use_host_id_offset=use_host_id_offset,
        debug=debug,
    )
    right = create_can_controller(
        right_port,
        profile="right",
        baudrate=baudrate,
        serial_timeout_s=serial_timeout_s,
        response_timeout_s=response_timeout_s,
        usb_mode=usb_mode,
        channel=right_channel,
        use_host_id_offset=use_host_id_offset,
        debug=debug,
    )
    return DualArmController(left, right)


def create_dual_simulated_controller() -> DualArmController:
    """Create an in-memory dual-arm controller for UI and algorithm testing."""

    return DualArmController(
        create_simulated_controller(profile="left"),
        create_simulated_controller(profile="right"),
    )


__all__ = [
    "create_can_controller",
    "create_dual_can_controller",
    "create_dual_simulated_controller",
    "create_simulated_controller",
]
