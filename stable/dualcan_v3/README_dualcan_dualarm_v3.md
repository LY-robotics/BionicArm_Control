# 双 CAN 双机械臂控制 V3

## 这版修正了什么

V3 使用 Normal/Passthrough 可变长度串口帧：

```text
53 54 + Channel + CANID(4B) + DLC + DATA(DLC字节) + 0D 0A
```

例如：

```text
A0: 53 54 01 00 00 01 22 01 A0 0D 0A
C1: 53 54 01 00 00 01 22 05 C1 F4 01 00 00 0D 0A
```

不要在 DLC=1/5 的情况下继续补 8 字节，否则板子可能无法识别帧尾。

## 文件

- `dualcan_arm_control_lib_v3.py`  
  底层 COM 多 CAN 通道库 + 单电机协议 + 单臂 + 双臂系统。

- `test_single_motor_channel_v3.py`  
  单电机通道测试脚本。

- `dualcan_dualarm_menu_v3.py`  
  双 CAN 双机械臂菜单测试程序。

## 单电机测试

只读：

```bash
python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --no-move --debug
```

速度测试：

```bash
python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --rpm 5 --seconds 2 --debug
```

测试 CAN2：

```bash
python test_single_motor_channel_v3.py --port COM17 --channel 2 --motor-id 34 --no-move --debug
```

## 双臂菜单

修改 `dualcan_dualarm_menu_v3.py` 顶部：

```python
PORT = "COM17"
RIGHT_ARM_CHANNEL = 1
LEFT_ARM_CHANNEL = 2
```

运行：

```bash
python dualcan_dualarm_menu_v3.py
```

常用命令：

```text
help
config all
version 0
bus all
status all
watch all

move 0 j2 10
jog 1 j4 -5
pose 0 j1=0 j2=20 j4=30
home 0

speed 0 all 5
accel 0 all 10
current 0 all 1.5
clear all
setzero 0 j2

vel 0 j2 5 2
holdstop all
disable 0
stop

demo1
demo3
quit
```

## 推荐测试顺序

1. 先用 `test_single_motor_channel_v3.py` 分别测 CAN1/CAN2。
2. 确认单电机读取和 C1 速度控制正常。
3. 配置右臂/左臂 5 个电机 ID。
4. 运行菜单，先执行 `bus 0`、`bus 1`。
5. 再执行小角度 `move` 或 `jog`。
