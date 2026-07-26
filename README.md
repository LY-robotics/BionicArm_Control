# SANPO 双 F4 五轴机械臂运动控制库

这个目录是整理后的独立 SDK。运动学、轨迹规划、双臂协调、Gloria-M
双夹爪、实时反馈和 `can_motor_arm_lib.py` 通信协议已经分层，不再依赖
原工程的 `ControlCAN.dll` 调用链。

当前稳定版本：`v1.0.0`。

新电脑部署和新维护者阅读入口：

- [Windows/Linux 从零部署](docs/DEPLOYMENT.md)
- [项目结构、逐文件作用和调用链](docs/PROJECT_STRUCTURE.md)
- [版本说明与实机验证范围](CHANGELOG.md)

## 1. 从输入坐标到电机的调用链

```text
arm_dashboard.py / 你的 Python 程序
        |
        v
ArmController / DualArmController
        |
        +-- MoveJ: 关节目标 -> 五次轨迹
        |
        +-- MoveCart: TCP目标 -> 目标点IK -> 关节五次轨迹
        |
        +-- MoveLine: TCP直线采样 -> 每点连续IK -> 关节采样轨迹
        |
        v
ArmHardware 统一硬件接口
        |
        v
SanpoCanBackend
        |
        v
CanArm / CanMotor
        |
        v
SerialUsbCanTransport
        |
        +-- AT 高级帧，或
        +-- ST + Channel + CAN ID + Data + CRLF
        |
        v
左臂 F4 串口 / 右臂 F4 串口 -> CAN 电机
```

运控层只处理“关节角、速度、加速度、时间”，不拼 CAN 字节；协议层只处理
USB/CAN 报文，不做逆解。换板卡协议时通常只改 `protocol/` 和
`hardware/`，不用动运动学。

## 2. 目录和文件作用

```text
sanpo_arm_control/
├─ arm_dashboard.py                 图形双臂控制台（推荐入口）
├─ arm_menu.py                      旧式命令行菜单，保留用于协议调试
├─ CHANGELOG.md                     版本能力、限制和实机验证记录
├─ docs/
│  ├─ DEPLOYMENT.md                 Windows/Linux 从零部署手册
│  └─ PROJECT_STRUCTURE.md          逐文件职责、分层和维护入口
├─ examples/
│  ├─ dual_arm_gripper_chain.py     坐标运动后夹持的完整调用示例
│  ├─ probe_gloria_can.py           不运动夹爪的 Channel/反馈 ID 探测
│  └─ smoke_test.py                 安装后的仿真自检
├─ pyproject.toml                   安装信息、依赖和命令入口
├─ README.md                        当前说明
├─ scripts/
│  ├─ bootstrap_windows.ps1         Windows 一键安装和验证
│  └─ bootstrap_linux.sh            Linux 一键安装和验证
├─ sanpo_arm_sdk/
│  ├─ __init__.py                   汇总稳定的公开 API
│  ├─ config.py                     左右臂电机 ID、方向、减速比和硬限位
│  ├─ settings.py                   常调的轨迹、推荐搜索和监控默认值
│  ├─ errors.py                     统一错误码与文本
│  ├─ factory.py                    真机/仿真/双臂控制器创建函数
│  ├─ system.py                     双 F4 串口资源的组合与生命周期
│  ├─ protocol/
│  │  └─ can_motor_arm_lib.py       AT/ST、通道端点、电机命令和响应解析
│  ├─ end_effectors/
│  │  ├─ base.py                    可替换夹爪的统一接口
│  │  ├─ models.py                  夹爪配置、标定、状态和寄存器定义
│  │  ├─ gloria_protocol.py         Gloria-M 纯 CAN 载荷编码/解析
│  │  ├─ gloria.py                  单个 Gloria-M 独立控制对象
│  │  ├─ dual.py                    双夹爪并行控制对象
│  │  ├─ simulated.py               不接真机的仿真夹爪
│  │  └─ telemetry.py               夹爪反馈、峰值和 CSV 导出
│  ├─ hardware/
│  │  ├─ base.py                    运控层依赖的硬件接口约定
│  │  ├─ can_backend.py             ArmHardware 到 CanArm 的适配器
│  │  └─ simulated_backend.py       不接真机的内存仿真后端
│  ├─ kinematics/
│  │  ├─ kinematic_5dof.py          正解、逆解、无解姿态推荐
│  │  ├─ guiji_quintic.py           关节五次轨迹与 MoveCart 点到点规划
│  │  ├─ cartesian_line.py          TCP 直线采样和连续分支 IK
│  │  └─ ideal_arm_model.py         理想连杆、Base/J1~J5/TCP 坐标系
│  ├─ motion/
│  │  ├─ arm_controller.py          单臂公开控制 API 和轨迹执行器
│  │  └─ dual_arm_controller.py     双臂并行规划、同步开始和同步结束
│  └─ monitoring/
│     └─ telemetry.py               实时采样、峰值统计和 CSV 导出
└─ tests/
   ├─ test_protocol.py              AT/ST 报文、电机 ID 和快速位置命令
   ├─ test_motion_chain.py          单臂 MoveJ/MoveCart 完整链路
   ├─ test_dual_line_telemetry.py   双臂、直线插补、推荐解和反馈导出
   ├─ test_ideal_arm_model.py       连杆长度、坐标系和正解一致性
   └─ test_gripper_integration.py   夹爪协议、通道隔离和双侧整链路
```

### 通常改哪个文件

| 需求 | 修改位置 |
|---|---|
| 换电机 ID、方向、减速比、硬限位 | `sanpo_arm_sdk/config.py` |
| 改默认速度、采样周期、直线容差、推荐步长 | `sanpo_arm_sdk/settings.py` |
| 改连杆、TCP 偏置、运动学关节范围 | `kinematics/kinematic_5dof.py` |
| 改 TCP 直线插补策略 | `kinematics/cartesian_line.py` |
| 改双臂同步行为 | `motion/dual_arm_controller.py` |
| 改板卡 USB/CAN 报文 | `protocol/can_motor_arm_lib.py` |
| 换夹爪或增加 RS485 夹爪 | `end_effectors/` 新增适配器 |
| 改 Gloria-M ID、反馈 ID、开合标定 | GUI 连接页或 `GloriaGripperConfig` |
| 改界面布局或增加操作入口 | `arm_dashboard.py` |

## 3. 三种运动方式

### MoveJ

输入 5 个关节角。控制器读取当前角度，按速度和加速度上限生成五次多项式，
再逐点调用协议库发送 5 个关节位置。

适合回零、摆姿态、已知关节目标的动作。TCP 路径不保证是直线。

### MoveCart

输入 `[x, y, z, pitch, j5]`。只在目标点做一次逆解，得到目标关节角后执行
MoveJ 式五次轨迹。

适合快速到达坐标点。目标 TCP 正确，但中间 TCP 路径不保证是直线。

`preview_ik_recommendation()` 会先尝试原始 pitch/J5；无解时先固定 J5 搜索
最近 pitch，再搜索最近的 pitch/J5 组合。GUI 可以预览并把推荐值写回输入框。

### MoveLine

先从当前 TCP 到目标 TCP 生成五次时间律直线采样点，再对每个采样点连续求
逆解。求解器会参考上一点和预测点选择同一 IK 分支，并限制相邻关节跳变。

TCP 位置和 J5 是每点硬约束；pitch 是参考曲线。原因是五轴机构通常无法在
任意直线的每一点同时严格满足位置、pitch 和固定 J5。

规划器会根据采样关节速度/加速度自动延长过短轨迹，不能通过把时长填得很小
绕过限速。

## 4. 双臂同步是怎样工作的

板卡的两颗 F4 各自对应一个串口，所以左右臂各有一个 `ArmController`。
`DualArmController` 的顺序是：

1. 两个线程同时读取左右臂真实关节角并离线规划。
2. 规划失败时不向任意一侧发送位置命令。
3. 把较短轨迹按时间拉伸到较长轨迹的时长，实现同步结束。
4. 分别给两个 F4 配置速度和加速度。
5. 两个发送线程等待同一个 `threading.Event`。
6. 事件释放后，两个串口各自按轨迹时间戳下发。

这是软件同步，不是两颗 MCU 的硬件时钟同步。操作系统线程和两个 USB 串口仍
可能有毫秒级启动差；需要更严格同步时，应在板卡固件中增加“缓存轨迹 +
统一触发时间”协议。

## 5. 图形控制台

按部署文档安装后先运行仿真：

```powershell
sanpo-arm-dashboard --simulate
```

只控制机械臂时，原来的 `create_dual_can_controller()` 仍可使用已验证的
AT 模式。机械臂和夹爪分别占用同一 F4 的两个 CAN 口时，必须使用带
`Channel` 的 ST 标准帧。图形控制台默认按当前实机状态启用右夹爪、关闭左夹爪：

```powershell
python arm_dashboard.py `
  --left-port COM8 `
  --right-port COM9 `
  --usb-mode standard `
  --left-channel 1 `
  --left-gripper-channel 2 `
  --right-channel 3 `
  --right-gripper-channel 4 `
  --no-left-gripper-enabled `
  --right-gripper-enabled
```

以后安装左夹爪时，在连接页勾选“启用左夹爪”，或传入
`--left-gripper-enabled`。机械臂连接是整机可用的必要条件；夹爪按侧独立探测，
任意一侧未启用、未安装或暂时无反馈，都不会再关闭已经连通的双臂和另一侧夹爪。

界面没有序号菜单：

- **连接**：设置两个 F4 串口、机械臂通道，并分别启用和配置左右夹爪。
- **双臂运动**：选择双臂/左臂/右臂、运动方式，输入目标并预览或执行；可按关节设置机械零点。
- **双夹爪**：左右/同步使能、开合、清错、设零、失能、反馈曲线和 CSV/PNG 导出。
- **理论模型**：显示理想连杆、关节坐标系及目标/理论/反馈 TCP 误差。
- **实时反馈**：记录角度、速度、Q 轴电流，显示峰值和折线图。
- **参数**：集中调整轨迹、推荐搜索和监控参数。

反馈数据会导出原始 UTF-8 CSV 和配套的 `_peaks.csv` 峰值表；当前选择的臂和
关节可导出三联 PNG 曲线。

## 6. Python API 示例

### 单臂 TCP 直线

```python
from sanpo_arm_sdk import OK, create_can_controller

arm = create_can_controller(
    "COM9",
    profile="right",
    baudrate=1_000_000,
    usb_mode="advanced",
)

if arm.connect() != OK or arm.sync_state() != OK:
    raise RuntimeError("右臂连接或反馈同步失败")

try:
    result = arm.MoveLine(
        [520.0, 300.0, 150.0, 10.0, 0.0],
        speed=20.0,
        accel=40.0,
        sample_period_s=0.05,
        position_tolerance_mm=0.1,
    )
    print("MoveLine:", result)
finally:
    arm.disable_all()
    arm.close()
```

### 双臂同步坐标运动

```python
from sanpo_arm_sdk import create_dual_can_controller

dual = create_dual_can_controller(
    "COM8",
    "COM9",
    baudrate=1_000_000,
    usb_mode="advanced",
)

if not dual.connect().success or not dual.sync_state().success:
    raise RuntimeError("双臂连接或反馈同步失败")

try:
    result = dual.move_both(
        [500, 250, -100, 0, 0],
        [500, 250,  100, 0, 0],
        mode="cartesian",
        speed=20,
        accel=40,
        allow_recommendation=True,
        synchronize_finish=True,
    )
    print(result)
finally:
    dual.stop(disable=True)
    dual.close()
```

## 7. 实时反馈与峰值

`TelemetryRecorder` 每次记录以下字段：

```text
timestamp, elapsed_s, arm, joint,
angle_deg, speed_rpm, current_a, error_code
```

峰值按“臂 + 关节”统计：

- 角度最小值和最大值
- 绝对速度峰值
- 绝对 Q 轴电流峰值
- 有效样本数

监控读取和轨迹下发共用每个 F4 串口的可重入锁，不会把报文写乱，但反馈请求会
占用串口时间。实际轨迹周期为 `0.05 s` 时，建议先从 `0.10 s` 的监控周期开始；
真机测得下发延迟后再提高采样率。

## 8. 理想连杆模型与坐标系

理论模型与当前运控正解使用完全相同的参数和变换链：

- 肩部 Base 原点：`[0, 0, 0] mm`
- Base 方向：`+X` 向下，`+Y` 向前，`+Z` 向右
- 上臂长度：`350 mm`
- 前臂长度：`250 mm`
- 肩部基准偏置：`[0, 0, -18] mm`
- 腕部到 TCP 偏置：`[0, -50.9117, 84.9117] mm`
- TCP 固定绕局部 X 轴旋转：`45°`

`build_ideal_arm_model(q_deg)` 输出：

- Base、J1、J2、J3、J4、J5、TCP 的 `4x4` 齐次变换矩阵
- 每个坐标系在 Base 中的原点和 X/Y/Z 单位轴向
- 理想连杆折线点
- 与现有 `forward_kinematics()` 相同的 TCP 位姿

```python
from sanpo_arm_sdk.kinematics import (
    build_ideal_arm_model,
    compare_tcp_positions,
    export_ideal_model_csv,
)

theory = build_ideal_arm_model([-10, 15, 20, 30, 5])
print("理论 TCP:", theory.tcp_position_mm)

for frame in theory.frames:
    print(frame.name, frame.origin_mm, frame.rotation)

comparison = compare_tcp_positions(
    target_mm=[524.5, 347.7, 156.1],
    theoretical_model=theory,
)
print("目标到理论误差:", comparison.target_to_theoretical_mm)

export_ideal_model_csv(
    "ideal_model.csv",
    target_mm=[524.5, 347.7, 156.1],
    theoretical_model=theory,
)
```

控制器也可以直接用当前编码器反馈建立模型：

```python
error, feedback_model = arm.ideal_model(check_limits=False)
if feedback_model is not None:
    print("反馈正解 TCP:", feedback_model.tcp_position_mm)
```

图形控制台“理论模型”页的流程：

1. 选择左臂或右臂，输入 `X/Y/Z/Pitch/J5`。
2. 点击“计算理论模型”，得到 IK 关节角、理论 TCP 和全部坐标系。
3. 点击“读取反馈对比”，读取编码器关节角并通过正运动学计算反馈 TCP。
4. 查看目标到理论、目标到反馈、理论到反馈的误差和误差向量。
5. 可导出完整坐标系 CSV 和当前 3D PNG。

注意：反馈 TCP 仍然是“编码器角度 + 理想正运动学”的计算值，不是独立空间测量。
它能反映电机/关节是否跟随到理论角度，但不能单独识别连杆加工误差、装配偏差、
零点误差、间隙或负载变形。要标定绝对 TCP 精度，需要激光跟踪仪、三坐标、
视觉标定板或可靠的实体基准点提供外部测量值。

## 9. 板卡和电机映射

运控模型的 J1/J2 与原始 `can_motor_arm_lib.py` 示例编号相反，因此交换发生在
`config.py` 的硬件适配边界，运动学公式和轨迹数组顺序不变：

| 运控逻辑关节 | 原协议关节 | 右臂电机 ID | 左臂电机 ID |
|---|---|---:|---:|
| J1 | 原 J2 | 34 | 1 |
| J2 | 原 J1 | 35 | 55 |
| J3 | 原 J3 | 31 | 15 |
| J4 | 原 J4 | 32 | 18 |
| J5 | 原 J5 | 33 | 23 |

右臂方向为 `+1`，左臂方向为 `-1`。J1/J2 的名称和限位参数也随原关节一起
交换，保证命令下发、反馈显示和限位检查使用同一套运控编号。

串口波特率 `1_000_000` 在 `factory.create_can_controller()` 的 `baudrate`
参数和 GUI 连接页中设置。它是 PC 到 F4 的 USB CDC 串口波特率；CAN 总线位
时序由板卡固件配置，不应把两者当成同一个参数。

## 10. 安装和测试

完整的新电脑部署、串口权限和故障处理见
[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)。

```powershell
python -m pip install -e .
python -m unittest discover -s tests -v
sanpo-arm-smoke-test
```

测试全程使用仿真后端或内存报文，不连接真机。

## 11. 上真机前必须确认

1. 核对左右 F4 串口，不要接反。
2. 核对 10 个电机 ID、方向、减速比、机械零点和硬限位。
3. 首次运动卸载或架空机械臂，使用低速度和低加速度。
4. 准备独立硬件急停；软件“停止并失能”不能替代硬件急停。
5. 先在 GUI 做“规划预览”，再执行短距离单臂动作，最后测试双臂。
6. 仿真通过只证明软件链路正确，不代表真机标定和负载参数正确。

## 12. 双 F4、双臂和双夹爪结构

整板有两颗 F4，USB 后显示两个串口。每颗 F4 控制两个物理 CAN 接口：

```text
左 F4 串口 -> 左侧两个 CAN 口 -> 左机械臂 / 左 GloriaGripper
右 F4 串口 -> 右侧两个 CAN 口 -> 右机械臂 / 右 GloriaGripper
```

SANPO V4 官方 USB 协议把物理口全局编号为 CAN1～CAN4，但部分固件实测会在
每颗 F4 对应的 COM 口内使用局部 Channel 1/2。通道编号必须以 `AT+VER` 返回的
固件和只读探测结果为准，GUI 中可以修改。四个设备对象
相互独立。同侧机械臂和夹爪只共享 F4 的 USB 串口及事务锁，不共享 CAN 口，
夹爪代码也不会进入运动学或关节轨迹调用链。

夹爪接入失败时，先关闭控制台，再运行只读探测。它只请求状态，不使能也不
发送运动命令：

```powershell
python examples\probe_gloria_can.py COM25 --baudrate 1000000 --motor-id 1 --debug
```

输出会分别给出请求 Channel、实际返回 Channel 和检测到的 Master CAN ID。
若四个 Channel 都没有反馈，问题不在 GUI 通道编号，应检查夹爪供电、
CAN_H/CAN_L/GND、终端电阻、Motor ID，以及该物理 CAN 口的总线波特率。
这里的 `--baudrate` 是 USB CDC 串口波特率，不等于 CAN 总线波特率。

`DualF4System` 只负责串口生命周期。实际控制仍从两个独立属性进入：

```python
from sanpo_arm_sdk import create_dual_f4_system

system = create_dual_f4_system(
    "COM8",
    "COM9",
    left_arm_channel=1,
    left_gripper_channel=2,
    right_arm_channel=3,
    right_gripper_channel=4,
    left_gripper_enabled=False,
    right_gripper_enabled=True,
)

connection = system.connect()
if not connection.success:
    raise RuntimeError(connection)
print("右夹爪在线:", connection.grippers.right_success)

arms = system.arms
grippers = system.grippers

try:
    if not arms.sync_state().success:
        raise RuntimeError("机械臂反馈同步失败")

    # 运控链路：坐标 -> IK -> 轨迹 -> 左臂关节 CAN 命令。
    error = arms.left.MoveCart(
        [450.0, 100.0, 180.0, 30.0, 0.0],
        speed=10.0,
        accel=20.0,
    )
    if error != 0:
        raise RuntimeError(f"左臂运动失败: {error}")

    # 夹爪链路完全独立：50% 开度 -> Gloria PV 命令 -> 右夹爪 CAN 口。
    grippers.right.enable()
    state = grippers.right.move_normalized(0.5, velocity_rad_s=0.2)
    print(state)
finally:
    arms.stop(disable=True)
    system.close()
```

`connection.success` 表示双臂可用；需要确认所有已启用夹爪也在线时，检查
`connection.all_requested_devices_connected`。关节零点和夹爪零点都会改变后续
位置基准，只能在人工确认机械姿态并停止运动后执行。

以后更换夹爪时，实现 `end_effectors/base.py` 中的 `GripperHardware` 接口，
再在系统工厂中注入新对象即可。机械臂的 `ArmController`、逆解和轨迹代码
不需要修改。RS485 夹爪同样新增 RS485 传输端点和对应适配器，不改变上层
夹爪 API。

## 13. Gloria-M 参数含义

| 参数 | 含义 |
|---|---|
| `motor_id` | Gloria-M 电机地址，当前反馈格式支持 0～15 |
| `master_can_id` | 预期的夹爪反馈 CAN ID；运行时也会按反馈数据自动识别实际值 |
| `startup_control_mode` | 使能前选择的模式，控制台默认 `POSITION_VELOCITY`，不写 Flash |
| `closed_position_rad` | 全闭位置标定，默认 0.0 rad |
| `open_position_rad` | 全开位置标定，默认 2.7 rad |
| `opening_fraction` | 统一开度，0 表示全闭，1 表示全开 |
| `velocity_rad_s` | Gloria-M PV 模式速度，单位 rad/s |

默认开合位置来自原始夹爪 SDK 的安全范围，只能作为初始值。首次实机调试要
卸载工件、低速点动，并根据真实机械限位重新标定。当前没有夹爪连杆尺寸和
力传感器标定，因此不能把电机弧度或估算力矩直接宣称为毫米开度和牛顿夹持力。
