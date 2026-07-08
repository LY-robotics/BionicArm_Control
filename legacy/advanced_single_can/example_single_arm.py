"""
Example usage for can_motor_arm_lib.py

Windows:
    PORT = "COM17"
Ubuntu:
    PORT = "/dev/ttyACM0"   # or /dev/ttyUSB0

Before running movement commands, make sure the arm is fixed safely.
"""

import time

from can_motor_arm_lib import (
    CanArm,
    CanMotor,
    JointConfig,
    RIGHT_ARM_EXAMPLE,
    SerialUsbCanTransport,
    list_serial_ports,
)


PORT = "COM17"      # Windows: COM17; Ubuntu: /dev/ttyACM0 or /dev/ttyUSB0
BAUD = 1000000


# If you receive a new arm, the first thing to modify is this list.
# motor_id uses decimal by default. Example: motor id 34 decimal == 0x22.
ARM_CONFIG = RIGHT_ARM_EXAMPLE

# Or create a new arm quickly:
# ARM_CONFIG = [
#     JointConfig("j1", 34, "J1", ratio=3.0, direction=1, min_deg=-110, max_deg=110),
#     JointConfig("j2", 35, "J2", ratio=3.0, direction=1, min_deg=-170, max_deg=120),
#     JointConfig("j3", 36, "J3", ratio=4.0, direction=1, min_deg=-180, max_deg=180),
#     JointConfig("j4", 37, "J4", ratio=4.2, direction=1, min_deg=-90,  max_deg=150),
#     JointConfig("j5", 38, "J5", ratio=1.0, direction=1, min_deg=-120, max_deg=120),
# ]


def print_feedback(arm: CanArm):
    fb = arm.read_all_feedback()
    print("\n===== ARM FEEDBACK =====")
    for key, item in fb.items():
        angle = "---" if item.angle_deg is None else f"{item.angle_deg:.2f} deg"
        speed = "---" if item.speed_rpm is None else f"{item.speed_rpm:.2f} rpm"
        cur = "---" if item.q_current_a is None else f"{item.q_current_a:.3f} A"
        mode = "---"
        fault = "---"
        if item.status:
            mode = str(item.status.get("mode_text"))
            fault = str(item.status.get("faults"))
        print(f"{key}: id={item.motor_id:<3} angle={angle:<12} speed={speed:<12} current={cur:<10} mode={mode:<12} fault={fault}")


def main():
    print("Available ports:", list_serial_ports())

    with SerialUsbCanTransport(PORT, BAUD, debug=True) as bus:
        # Single motor smoke test. Use your tested motor id=34 decimal.
        m = CanMotor(bus, motor_id=34, name="test_motor")
        print("\nSingle motor version:", m.read_version())
        print("Single motor status :", m.read_status())

        # Arm object.
        arm = CanArm(bus, ARM_CONFIG, name="right_arm")
        arm.clear_faults()
        arm.configure_defaults()

        print_feedback(arm)

        # Safe small movement demo. Uncomment after mechanical safety check.
        # arm.move_absolute("j2", 5.0)
        # time.sleep(1.0)
        # arm.move_absolute("j2", 0.0)

        # Direct speed mode smoke test. Uncomment only for one free motor, not a mounted arm.
        # arm.set_joint_speed_mode("j2", 1.0)
        # time.sleep(1.0)
        # arm.stop_speed_mode("j2")

        # Emergency release / free state:
        # arm.disable_all()


if __name__ == "__main__":
    main()
