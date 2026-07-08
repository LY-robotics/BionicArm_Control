# BionicArm_Control

仿生机械臂 CAN 控制代码沉淀仓库。

## 环境：

1. **当前代码采用windows控制，可无障碍移植ubuntu，需要安装pyserial库**
2. **usb_can板子：SANPO 集成板标准板，双核双com，每com可控制两路can**

## 当前保留版本

### 1. legacy/advanced_single_can

早期 Advanced 模式单 CAN 版本。

用途：
- 单电机通信验证
- Advanced 模式协议回溯
- 旧版本菜单和测试代码留档

特点：
- 使用 Advanced 模式

- 不支持同一个 COM 口下选择 CAN1/CAN2 Channel

- 适合单路 CAN 测试

  ![image-20260708155416787](C:\Users\lzy\AppData\Roaming\Typora\typora-user-images\image-20260708155416787.png)

### 2. stable/dualcan_v3

当前主线版本。

用途：
- 同一个 COM 口下，通过 CAN1/CAN2 分别控制左右两条机械臂
- ARM0 右臂 -> CAN1
- ARM1 左臂 -> CAN2

特点：
- 使用 Normal/Passthrough 模式

- 支持 Channel 字段

- 支持可变长度 DLC

- 已验证单电机 A0/AE/C1 通信正常

  ![image-20260708155715424](C:\Users\lzy\AppData\Roaming\Typora\typora-user-images\image-20260708155715424.png)

## V3 单电机测试

```bash
cd stable/dualcan_v3
python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --no-move --debug
python test_single_motor_channel_v3.py --port COM17 --channel 1 --motor-id 34 --rpm 5 --seconds 2 --debug
```

## V3 双臂菜单

```bash
cd stable/dualcan_v3
python dualcan_dualarm_menu_v3.py
```

## 版本说明

- v0.1：导入 Advanced 单 CAN 版本和 Dual CAN V3 可变 DLC 验证版本。