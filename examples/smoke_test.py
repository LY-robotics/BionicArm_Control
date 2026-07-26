"""Installed-package smoke test using only simulated hardware."""

from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import arm_dashboard
import arm_menu
import matplotlib
import numpy
import serial

from sanpo_arm_sdk import __version__, create_dual_simulated_system


def main() -> None:
    # Importing these modules above verifies that GUI and runtime dependencies
    # are available before the simulated control chain starts.
    system = create_dual_simulated_system(
        left_gripper_enabled=False,
        right_gripper_enabled=True,
    )
    try:
        connection = system.connect()
        if not connection.success:
            raise RuntimeError(f"simulated connection failed: {connection}")
        joint_sync = system.arms.sync_state()
        if not joint_sync.success:
            raise RuntimeError(f"simulated arm sync failed: {joint_sync}")

        system.grippers.right.enable()
        state = system.grippers.right.move_normalized(0.5, 0.2)
        if state is None or abs(float(state.opening_fraction) - 0.5) > 1e-9:
            raise RuntimeError("simulated gripper command did not reach 50%")
    finally:
        system.close()

    print(f"SANPO Arm Control v{__version__}: smoke test passed")


if __name__ == "__main__":
    main()
