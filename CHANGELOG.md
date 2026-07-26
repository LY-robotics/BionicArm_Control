# Changelog

本项目使用语义化版本号。硬件标定变化、协议变化和安全相关变化必须记录在这里。

## v1.0.0 - 2026-07-26

首个完成双臂和右侧 Gloria-M 夹爪实机闭环验证的发布版本。

### 主要能力

- SANPO 双 F4 控制板 USB CDC `ST` 标准帧通信。
- 左右五轴机械臂独立控制和双臂同步执行。
- `MoveJ`、`MoveCart`、`MoveLine` 三种运动方式。
- 无解目标的 Pitch/J5 推荐搜索。
- 理想连杆模型、J1～J5/TCP 坐标系和理论/反馈 TCP 对比。
- 关节与夹爪实时反馈、峰值统计、CSV 和曲线导出。
- 独立、可替换的夹爪接口；允许未安装左夹爪。
- Gloria-M 状态、寄存器、使能、清错、设零和 PV 开合控制。
- 自动识别 Gloria-M 实际 Master CAN ID。
- 夹爪使能前自动选择 `POSITION_VELOCITY` 运行模式，不写入 Flash。
- 只读夹爪 Channel/Master CAN ID 探测工具。

### 硬件验证范围

- 左右机械臂：实机运动通过。
- 右侧 CAN Gloria-M 夹爪：实机通信与控制通过。
- 左侧夹爪：当前未安装，软件按可选设备处理。
- Windows：主开发和实机测试平台。
- Linux：安装、导入、仿真和测试链路由 CI 覆盖；真机串口名称和权限需按主机配置。

### 兼容性和限制

- 支持 Python 3.10～3.12，推荐 Python 3.12。
- 默认使用 USB CDC `ST` 模式，不使用旧 `ControlCAN.dll`。
- 五轴运动学只严格控制 XYZ、Pitch 和 J5，不是六自由度完整姿态控制。
- 理想 TCP 对比基于编码器角度和理想连杆参数，不代替外部绝对位置测量。
- CAN Channel 编号必须以板卡固件和只读探测结果为准。

### 从旧仓库迁移

- 旧 `legacy/` 和 `stable/dualcan_v3/` 代码仍可从 Git 历史和旧 Tag
  `v0.1-baseline-advanced-and-dualcan-v3` 获取。
- 当前 `main` 只保留可安装、可测试的统一 SDK。
- 原始 Gloria 实习生 SDK 已完成协议提取，发布版不再重复携带或运行时依赖它。
