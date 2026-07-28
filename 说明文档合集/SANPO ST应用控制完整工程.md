# SANPO ST应用控制完整工程

## 五关节协同控制版本

已增加五关节目标暂存、统一校验、按行程比例同步到达、状态位图和 PC SDK
`move_group()` 接口。请先阅读
`docs/五关节协同控制与完整调试说明.md`。

`Release/original_factory_firmware_v4.bin` 是旧原厂固件，不含新增功能；请用
STM32CubeIDE 重新构建当前源码后烧录。

本工程基于用户提供的原厂 `sanpo_spine` CubeIDE工程修改，保留原厂：

- STM32F407VET6时钟和引脚；
- USB CDC描述符、初始化及中断；
- CAN1/CAN2初始化、滤波器和FIFO；
- SPI、RS485、I2C、ADC和DMA；
- 原厂FreeRTOS任务；
- `libloop_function_v4.a`。

新增功能：

- 使用原厂ST帧格式承载板级应用命令；
- PC查询板卡身份；
- STM32执行关节角度、速度和减速比换算；
- 软限位检查；
- CAN位置控制、回零、清故障和全停；
- 周期读取电机A4状态；
- STM32缓存角度、速度、故障和在线状态；
- PC心跳超时后发送全停；
- 普通ST/ET帧继续交给原厂USB-CAN静态库。

## 关键稳定性修改

本版没有创建新的FreeRTOS任务，也没有使用 `osMessageQueueNew()`：

```text
USB中断 → 静态USB环形缓冲区
CAN1中断 → 静态CAN1环形缓冲区
CAN2中断 → 静态CAN2环形缓冲区
原厂defaultTask → SanpoApp_Process()
```

这避免了此前新增任务及动态队列导致FreeRTOS堆不足、USB停止运行的问题。

## 应用ST帧

应用命令仍使用原厂标准帧格式：

```text
53 54 FE 00 00 07 F0 DLC DATA 0D 0A
```

- Channel：`0xFE`，区别于原厂0～4通道；
- 请求ID：`0x7F0`；
- 响应ID：`0x7F1`；
- 无新增CRC；
- 普通电机ST帧仍由原厂 `cdc_recv_fs()` 处理。

使用前必须确认实际CAN总线没有设备使用标准ID `0x7F0/0x7F1`。

## 应用命令

| 命令 | 数值 | 数据 |
|---|---:|---|
| BOARD_INFO | `0x01` | 无 |
| MOVE_JOINT | `0x02` | 关节1B、角度int32×0.01°、速度uint16×0.01rpm |
| GET_STATE | `0x03` | 关节ID |
| STOP_ALL | `0x04` | 无 |
| HEARTBEAT | `0x05` | 无 |
| HOME | `0x06` | 关节ID |
| CLEAR_FAULT | `0x07` | 关节ID |

响应的第一个字节为 `命令|0x80`，第二个字节为状态码。

## 编译芯片1

默认：

```c
#define SANPO_BUILD_TARGET_MCU  1U
```

直接编译并烧录到STM32(1)，其本地CAN1/CAN2对应物理CAN-1/CAN-2。

## 编译芯片2

打开：

```text
Core/App/config/sanpo_board_config.h
```

改成：

```c
#define SANPO_BUILD_TARGET_MCU  2U
```

重新Clean、Build，烧录到STM32(2)。其本地CAN1/CAN2对应物理CAN-3/CAN-4。

## CubeIDE操作

重要：`StartDefaultTask()` 中原厂模板的 `remove_me();` 必须删除。应用处理
循环 `SanpoApp_Process()` 位于该位置之后；如果仍调用 `remove_me()`，
上位机命令可能被USB回调接收但不会得到处理，PC端会表现为
`BOARD_INFO timeout`。

1. `File → Import → Existing Projects into Workspace`；
2. 选择本工程目录；
3. `Project → Clean`；
4. `Project → Build Project`；
5. 确认 `Core/App` 下所有 `.c` 都出现在编译日志；
6. 烧录对应芯片。

如果构建日志没有编译 `Core/App`，刷新工程后执行：

```text
右键Core/App → Resource Configurations → Exclude from Build
```

确保Debug和Release均未勾选排除。

## PC SDK

```powershell
cd pc_sdk
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
pip install -e .
python examples\board_test.py COM30
```

第一次只运行只读测试，不发送运动命令。

## 参数位置

打开：

```text
Core/App/config/sanpo_joint_config.h
```

逐轴确认：

- `motor_id`；
- `can_channel`；
- `direction`；
- `gear_ratio`；
- `zero_deg`；
- `min_deg/max_deg`；
- `max_rpm`；
- `max_current_a`。

当前CAN通道、方向、零点及0.5A电流只是调试参考值，不是量产参数。

## 首次实验顺序

1. 不接电机动力，烧录后确认两个COM口稳定出现并可打开；
2. PC执行 `BOARD_INFO`；
3. 接一台电机，先确认CAN波特率和终端电阻；
4. 执行 `GET_STATE`，确认能收到A4；
5. 测试 `CLEAR_FAULT`；
6. 测试 `STOP_ALL`；
7. 电机脱离机械负载后，以极小角度、低速测试 `MOVE_JOINT`；
8. 验证方向、减速比和软限位；
9. 单芯片稳定后再测试第二颗芯片。

## 已知边界

- 原厂V4.1 ST协议没有序号字段，PC端应串行发送请求；
- 应用帧必须由一次USB写操作发送，不要逐字节发送；
- 原厂静态库内部不可见，应用模式下CAN反馈由新增代码接管；
- PC心跳超过1秒后，STM32发送全停并恢复原厂CAN回调；
- 未接电机时，BOARD_INFO仍应正常返回，GET_STATE返回离线状态；
- 此环境没有ARM GCC工具链，最终链接和硬件验证必须在STM32CubeIDE完成。
