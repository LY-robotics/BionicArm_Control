"""SANPO dual-F4 five-axis arm motion-control SDK."""

__version__ = "1.1.0"

from .config import (
    ARM_PROFILES,
    JOINT_KEYS,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    get_arm_joint_configs,
)
from .errors import *
from .factory import (
    create_can_controller,
    create_dual_can_controller,
    create_dual_simulated_controller,
    create_simulated_controller,
)
from .end_effectors import (
    DualGripperController,
    DualGripperResult,
    GloriaGripper,
    GloriaGripperConfig,
    GloriaRegister,
    GripperCalibration,
    GripperControlMode,
    GripperLimits,
    GripperPeakSummary,
    GripperState,
    GripperTelemetryRecorder,
    GripperTelemetrySample,
    SimulatedGripper,
    UnavailableGripper,
)
from .motion import ArmController, DualArmController, DualMotionResult, PreparedMotion
from .monitoring import PeakSummary, TelemetryRecorder, TelemetrySample
from .protocol import list_serial_ports
from .system import (
    DualF4System,
    SystemConnectionResult,
    create_dual_f4_system,
    create_dual_simulated_system,
)

__all__ = [
    "__version__",
    "ARM_PROFILES",
    "ArmController",
    "DualArmController",
    "DualMotionResult",
    "DualF4System",
    "DualGripperController",
    "DualGripperResult",
    "GloriaGripper",
    "GloriaGripperConfig",
    "GloriaRegister",
    "GripperCalibration",
    "GripperControlMode",
    "GripperLimits",
    "GripperPeakSummary",
    "GripperState",
    "GripperTelemetryRecorder",
    "GripperTelemetrySample",
    "JOINT_KEYS",
    "LEFT_ARM_JOINTS",
    "PeakSummary",
    "PreparedMotion",
    "RIGHT_ARM_JOINTS",
    "TelemetryRecorder",
    "TelemetrySample",
    "SystemConnectionResult",
    "SimulatedGripper",
    "UnavailableGripper",
    "create_can_controller",
    "create_dual_can_controller",
    "create_dual_simulated_controller",
    "create_dual_f4_system",
    "create_dual_simulated_system",
    "create_simulated_controller",
    "get_arm_joint_configs",
    "list_serial_ports",
]
