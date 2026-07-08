from can_motor_arm_lib import SerialUsbCanTransport, CanMotor
import time
 
with SerialUsbCanTransport("COM17", 1000000, debug=True) as bus:
    m = CanMotor(bus, motor_id=34)

    print(m.read_version())
    print(m.read_status())

    m.clear_fault()
    m.set_speed(1.0)      # 1 RPM
    time.sleep(5)
    m.set_speed(0.0)      # 停止
    m.disable()           # 失能进入自由态