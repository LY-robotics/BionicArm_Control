import serial
import time


# USB-CAN协议
HEADER = bytes([0x41, 0x54])   # AT
TAIL = bytes([0x0D, 0x0A])


COM = "COM17"
BAUD = 1000000


# ==========
# 电机地址
# ==========
# 你的电机显示34
# 如果软件显示十进制34：
MOTOR_ID = 0x22

# 如果发现不通，再改成：
# MOTOR_ID = 0x34



def hexprint(data):
    print(" ".join(f"{x:02X}" for x in data))



# 标准CAN ID编码
def build_std_id(can_id):

    value = (can_id & 0x7FF) << 21

    return value.to_bytes(
        4,
        byteorder="big"
    )



def build_packet(can_id, data):

    frame_id = build_std_id(can_id)

    packet = (
        HEADER
        +
        frame_id
        +
        bytes([len(data)])
        +
        data
        +
        TAIL
    )

    return packet



def send_can(ser, can_id, data):

    packet = build_packet(
        can_id,
        data
    )


    print("\nTX:")
    hexprint(packet)


    ser.write(packet)
    ser.flush()


    time.sleep(0.2)


    rx = ser.read_all()


    print("RX:")

    if rx:
        hexprint(rx)

    else:
        print("NO RESPONSE")



def main():

    ser = serial.Serial(
        COM,
        BAUD,
        timeout=0.2
    )


    print("open", COM)


    # 切换高级模式
    ser.write(
        b"AT+AT\r\n"
    )

    time.sleep(0.2)

    print(
        "MODE:"
    )

    print(
        ser.read_all()
    )



    #
    # 1.读取版本
    #
    print("\n===== READ VERSION =====")

    send_can(
        ser,
        0x122,
        bytes([
            0xA0
        ])
    )



    #
    # 2.读取状态
    #
    print("\n===== READ STATUS =====")

    send_can(
        ser,
        0x122,
        bytes([
            0xCE,
            0x01
        ])
    )



    #
    # 3.速度控制
    #
    # 10RPM
    #
    # 10/0.01=1000
    #
    # 小端:
    # E8 03 00 00
    #

    print("\n===== SPEED 10RPM =====")

    send_can(
        ser,
        0x122,
        bytes([
            0xC1,
            0xE8,
            0x03,
            0x00,
            0x00
        ])
    )
    time.sleep(5)
    # print("\n===== SPEED 1RPM =====")

    # send_can(
    #     ser,
    #     0x122,
    #     bytes([
    #         0xC1,
    #         0x64,
    #         0x00,
    #         0x00,
    #         0x00
    #     ])
    # )
    # time.sleep(5)
    print("\n===== SPEED 0RPM =====")
    send_can(
        ser,
        0x122,
        bytes([
            0xA2
        ])
    )
    ser.close()



if __name__=="__main__":

    main()