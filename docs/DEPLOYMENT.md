# 从零部署指南

本文面向一台没有安装过本项目的新电脑。完成后应能：

1. 安装 `sanpo-arm-control` 及全部 Python 依赖。
2. 运行 32 项离线单元测试和仿真自检。
3. 启动图形控制台仿真模式。
4. 识别双 F4 串口，并在人工核对后连接真机。

## 1. 支持范围

| 项目 | 支持范围 |
|---|---|
| Python | 3.10、3.11、3.12，推荐 3.12 |
| Windows | Windows 10/11 64 位 |
| Linux | 带桌面的常见 x86_64/ARM64 发行版 |
| 板卡 | SANPO 双 F4 集成板，USB CDC 串口方式 |
| USB 协议 | `ST` 标准帧模式 |
| Python 依赖 | NumPy、Matplotlib、pySerial |

本项目不依赖 `ControlCAN.dll`，也不依赖原始 Gloria SDK。Linux 下当前同样
使用 `/dev/ttyACM*` 或 `/dev/ttyUSB*` 的 USB CDC 串口，不使用 SocketCAN
`can0` 接口。

## 2. 下载指定版本

安装 Git 后，任选一种方式克隆：

```bash
# 已配置 GitHub SSH 密钥
git clone git@github.com:LY-robotics/BionicArm_Control.git

# 或使用 HTTPS
git clone https://github.com/LY-robotics/BionicArm_Control.git
```

进入目录并固定到经过实机验证的发布版本：

```bash
cd BionicArm_Control
git checkout v1.0.0
```

日常开发可以继续使用 `main`，生产设备建议固定 Tag，避免拉取尚未重新做过
实机验证的提交。

## 3. Windows 10/11

### 3.1 安装基础软件

1. 安装 64 位 Git for Windows。
2. 安装 Python 3.12 64 位。
3. Python 安装界面勾选 `Add python.exe to PATH` 和 `py launcher`。
4. 用 USB 连接 SANPO 板，确认设备管理器中出现两个 COM 口。

在 PowerShell 检查：

```powershell
git --version
py -3.12 --version
```

### 3.2 一键安装

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\bootstrap_windows.ps1
```

脚本会创建本项目专用的 `.venv`、安装依赖、以 editable 模式安装 SDK、
运行单元测试并运行仿真自检。它不会连接或运动真机。

### 3.3 手动安装

需要理解每一步时，可执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip setuptools wheel
.\.venv\Scripts\python.exe -m pip install -e .
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\sanpo-arm-smoke-test.exe
```

PowerShell 激活虚拟环境不是必需的。需要激活时：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\.venv\Scripts\Activate.ps1
```

### 3.4 启动

先验证仿真界面：

```powershell
.\.venv\Scripts\sanpo-arm-dashboard.exe --simulate
```

再启动真机界面：

```powershell
.\.venv\Scripts\sanpo-arm-dashboard.exe
```

查看串口：

```powershell
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

不要沿用旧电脑的 COM25/COM26。Windows 在新电脑上分配的 COM 号可能不同，
必须结合板卡背面说明和低风险单臂反馈测试重新确认左右 F4。

## 4. Linux

### 4.1 安装系统包

Debian/Ubuntu：

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip python3-tk
```

Fedora：

```bash
sudo dnf install -y git python3 python3-pip python3-tkinter
```

Arch Linux：

```bash
sudo pacman -S --needed git python python-pip tk
```

检查系统 Python：

```bash
python3 --version
```

版本必须为 3.10～3.12。若发行版默认版本超出范围，请安装 Python 3.12，并在
后面的命令中设置，例如：

```bash
PYTHON_BIN=python3.12 ./scripts/bootstrap_linux.sh
```

### 4.2 串口权限

Debian/Ubuntu 通常需要将当前用户加入 `dialout`：

```bash
sudo usermod -aG dialout "$USER"
```

执行后必须注销桌面会话并重新登录。重新登录后检查：

```bash
groups
python3 -m serial.tools.list_ports -v
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

其他发行版的串口组可能是 `uucp` 或发行版自定义组，应以设备节点的 group
为准。不要为了绕过权限长期使用 `sudo` 启动整个控制台。

### 4.3 一键安装

```bash
chmod +x scripts/bootstrap_linux.sh
./scripts/bootstrap_linux.sh
```

脚本只使用仿真硬件执行验证，不会向串口发送命令。

### 4.4 手动安装

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e .
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/sanpo-arm-smoke-test
```

### 4.5 启动

图形控制台要求桌面环境和可用的 `DISPLAY`：

```bash
.venv/bin/sanpo-arm-dashboard --simulate
.venv/bin/sanpo-arm-dashboard
```

SSH 到无桌面的主机时不要直接启动 Tk 图形界面。可使用
`sanpo-arm-menu`、Python API，或配置 X11/远程桌面后再启动 GUI。

## 5. 安装验收

以下命令必须全部成功：

```bash
python -c "import sanpo_arm_sdk; print(sanpo_arm_sdk.__version__)"
python -m unittest discover -s tests -v
sanpo-arm-smoke-test
sanpo-arm-dashboard --simulate
```

预期版本为 `1.0.0`，测试结尾为 `OK`，仿真自检输出：

```text
SANPO Arm Control v1.0.0: smoke test passed
```

测试和自检不接触真机，适合在任何新电脑上先验证软件环境。

## 6. 首次连接真机

1. 断开机械负载或架空机械臂，准备硬件急停。
2. 确认左右 F4 分别出现一个 USB CDC 串口。
3. GUI 选择 `ST 标准模式`，串口波特率使用 `1000000`。
4. 左夹爪未安装时取消勾选“启用左夹爪”。
5. 先连接并同步，不立即运动。
6. 先读取单臂关节反馈，确认左右臂没有接反。
7. 先执行低速、短距离、单关节动作。
8. 再测试单臂坐标运动、双臂同步和右夹爪。

界面的 `1000000` 是 PC 到 F4 的 USB CDC 串口波特率，不等于夹爪或机械臂
物理 CAN 总线的位时序。

### 6.1 右夹爪只读探测

先关闭控制台，保证串口没有被其他进程占用：

```bash
sanpo-arm-gripper-probe COM25 --baudrate 1000000 --motor-id 1 --debug
```

Linux 示例：

```bash
sanpo-arm-gripper-probe /dev/ttyACM1 --baudrate 1000000 --motor-id 1 --debug
```

该命令只请求夹爪状态，不使能、不设零、不发送运动命令。输出中的
`TX channel`、`RX channel` 和 `Master CAN ID` 用于填写 GUI。

## 7. 常见故障

### `ModuleNotFoundError: sanpo_arm_sdk`

没有安装项目，或启动时使用了系统 Python。重新运行 bootstrap 脚本，或明确
使用 `.venv` 中的 Python。

### Windows 无法执行 `Activate.ps1`

不必激活环境，直接运行 `.venv\Scripts\python.exe`；或仅对当前 PowerShell
进程执行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

### Linux `Permission denied: /dev/ttyACM0`

检查设备节点所属组，将用户加入对应组后注销并重新登录。不要只开一个新终端，
因为旧桌面会话的组权限不会自动更新。

### 串口存在但连接超时

依次检查：

1. 左右 F4 串口是否接反。
2. 是否选择 `ST` 而不是旧 `AT` 模式。
3. 固件是否支持当前 ST 协议。
4. Channel 是全局 1～4 还是当前固件的局部 1/2 映射。
5. 电机 ID、夹爪 Motor ID 和 Master CAN ID。
6. CAN_H/CAN_L、共地、终端电阻、设备供电和物理 CAN 波特率。
7. 是否有另一个控制台、串口工具或系统服务正在占用串口。

### GUI 能打开但图表或 Tk 报错

Linux 安装 `python3-tk`/`python3-tkinter`，确认处于桌面会话；不要在没有
`DISPLAY` 的纯 SSH 终端启动 GUI。

### 测试成功但真机不运动

离线测试只验证软件算法、报文格式和调用关系。真机还依赖机械零点、方向、
限位、电机 ID、CAN 位时序、供电和急停链路。

## 8. 更新、回退和卸载

更新 main：

```bash
git switch main
git pull --ff-only
python -m pip install -e .
python -m unittest discover -s tests -v
```

回退到实机验证版本：

```bash
git checkout v1.0.0
python -m pip install -e .
```

卸载 Python 包：

```bash
python -m pip uninstall sanpo-arm-control
```

删除本地虚拟环境后可完全重建；不要提交 `.venv`、反馈 CSV、曲线图片或
`__pycache__`。

## 9. 权威参考

- Python 虚拟环境：
  <https://docs.python.org/3.12/tutorial/venv.html>
- pySerial 串口枚举：
  <https://pyserial.readthedocs.io/en/latest/tools.html>
- SANPO V4 文档：
  <https://docs.sanporobot.com/v4/zh/>
- SANPO USB 转 CAN/ST 协议：
  <https://docs.sanporobot.com/v4/zh/usb_can.html>
