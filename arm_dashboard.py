"""Graphical dual-arm control, trajectory preview and telemetry console."""

from __future__ import annotations

import argparse
import queue
import threading
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox
import tkinter as tk
from tkinter import ttk
from typing import Callable, Optional

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import rcParams

from sanpo_arm_sdk import (
    JOINT_KEYS,
    OK,
    ArmController,
    DualArmController,
    DualF4System,
    DualGripperController,
    GloriaGripperConfig,
    GripperCalibration,
    GripperState,
    GripperTelemetryRecorder,
    TelemetryRecorder,
    create_dual_f4_system,
    create_dual_simulated_system,
    list_serial_ports,
)
from sanpo_arm_sdk.errors import err_text
from sanpo_arm_sdk.kinematics import (
    IKRecommendConfig,
    IdealArmModel,
    build_ideal_arm_model,
    compare_tcp_positions,
    export_ideal_model_csv,
    inverse_kinematics,
    recommend_feasible_yaw,
)
from sanpo_arm_sdk.settings import (
    LINE_DEFAULTS,
    MOTION_DEFAULTS,
    RECOMMENDATION_DEFAULTS,
    TELEMETRY_DEFAULTS,
)


MODE_LABELS = {
    "关节空间 MoveJ": "joint",
    "坐标点到点 MoveCart": "cartesian",
    "TCP 直线插补 MoveLine": "line",
}
SCOPE_LABELS = {
    "双臂同步": "dual",
    "仅左臂": "left",
    "仅右臂": "right",
}
JOINT_COLORS = ("#0f766e", "#d1495b", "#e09f3e", "#2563eb", "#7c3aed")

# Matplotlib does not inherit Tk's font.  Set explicit CJK fallbacks so plot
# labels remain readable on Windows workstations and common Linux images.
rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "Noto Sans SC",
    "SimHei",
    "DejaVu Sans",
]
rcParams["axes.unicode_minus"] = False


class TargetPanel(ttk.LabelFrame):
    """Five-value editor reused for joint and Cartesian targets."""

    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(parent, text=title, padding=12)
        self.variables = [tk.StringVar(value="0") for _ in range(5)]
        self.labels: list[ttk.Label] = []
        self.entries: list[ttk.Entry] = []
        for index in range(5):
            label = ttk.Label(self, width=9, anchor="w")
            entry = ttk.Entry(self, textvariable=self.variables[index], width=13)
            label.grid(row=index, column=0, padx=(0, 8), pady=4, sticky="w")
            entry.grid(row=index, column=1, pady=4, sticky="ew")
            self.labels.append(label)
            self.entries.append(entry)
        self.columnconfigure(1, weight=1)
        self.set_mode("joint")

    def set_mode(self, mode: str) -> None:
        labels = (
            ("J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)", "J5 (deg)")
            if mode == "joint"
            else ("X (mm)", "Y (mm)", "Z (mm)", "Yaw (deg)", "J5 (deg)")
        )
        for label, text in zip(self.labels, labels):
            label.configure(text=text)

    def values(self) -> list[float]:
        return [float(variable.get().strip()) for variable in self.variables]

    def set_values(self, values: object) -> None:
        for variable, value in zip(self.variables, values):
            variable.set(f"{float(value):.4f}")

    def set_recommended_yaw_j5(self, yaw_deg: float, j5_deg: float) -> None:
        self.variables[3].set(f"{yaw_deg:.4f}")
        self.variables[4].set(f"{j5_deg:.4f}")


class ArmDashboard(tk.Tk):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self.title("SANPO 双臂运动控制台")
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        window_width = min(1420, max(1040, screen_width - 40))
        window_height = min(900, max(680, screen_height - 70))
        self.geometry(f"{window_width}x{window_height}+20+20")
        self.minsize(min(1040, window_width), min(680, window_height))
        self.configure(background="#eef1f4")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.system: DualF4System | None = None
        self.dual: DualArmController | None = None
        self.grippers: DualGripperController | None = None
        self.recorder: TelemetryRecorder | None = None
        self.gripper_recorder: GripperTelemetryRecorder | None = None
        self._events: queue.Queue[tuple[str, object, object]] = queue.Queue()
        self._busy = False
        self._configure_style()
        self._build_variables()
        self._build_ui()
        self.after(80, self._drain_events)
        self.after(300, self._monitor_tick)
        self.after(300, self._gripper_monitor_tick)

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", font=("Microsoft YaHei UI", 10))
        style.configure("TFrame", background="#eef1f4")
        style.configure("TLabel", background="#eef1f4", foreground="#20262e")
        style.configure("Header.TLabel", font=("Microsoft YaHei UI", 19, "bold"))
        style.configure("Sub.TLabel", foreground="#5f6b76")
        style.configure("Status.TLabel", foreground="#0f766e", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("TLabelframe", background="#f8fafb", borderwidth=1)
        style.configure("TLabelframe.Label", background="#eef1f4", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Accent.TButton", foreground="white", background="#0f766e", padding=(14, 8))
        style.map("Accent.TButton", background=[("active", "#115e59"), ("disabled", "#9aa5ad")])
        style.configure("Danger.TButton", foreground="white", background="#c2414f", padding=(14, 8))
        style.map("Danger.TButton", background=[("active", "#a83240")])
        style.configure("Treeview", rowheight=27, background="#ffffff", fieldbackground="#ffffff")
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 9, "bold"))
        style.configure("TNotebook", background="#eef1f4", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(16, 9))

    def _build_variables(self) -> None:
        ports = list_serial_ports()
        self.port_values = ports
        self.simulate_var = tk.BooleanVar(value=bool(self.args.simulate))
        self.left_port_var = tk.StringVar(value=self.args.left_port or (ports[0] if ports else "COM8"))
        self.right_port_var = tk.StringVar(
            value=self.args.right_port or (ports[1] if len(ports) > 1 else "COM9")
        )
        self.usb_mode_var = tk.StringVar(
            value="AT 高级模式" if self.args.usb_mode == "advanced" else "ST 标准模式"
        )
        self.left_channel_var = tk.IntVar(value=self.args.left_channel)
        self.right_channel_var = tk.IntVar(value=self.args.right_channel)
        self.left_gripper_channel_var = tk.IntVar(
            value=self.args.left_gripper_channel
        )
        self.right_gripper_channel_var = tk.IntVar(
            value=self.args.right_gripper_channel
        )
        self.left_gripper_enabled_var = tk.BooleanVar(
            value=bool(self.args.left_gripper_enabled)
        )
        self.right_gripper_enabled_var = tk.BooleanVar(
            value=bool(self.args.right_gripper_enabled)
        )
        self.left_gripper_id_var = tk.IntVar(value=self.args.left_gripper_id)
        self.right_gripper_id_var = tk.IntVar(value=self.args.right_gripper_id)
        self.left_gripper_master_id_var = tk.IntVar(
            value=self.args.left_gripper_master_id
        )
        self.right_gripper_master_id_var = tk.IntVar(
            value=self.args.right_gripper_master_id
        )
        self.gripper_closed_position_var = tk.StringVar(
            value=str(self.args.gripper_closed_position)
        )
        self.gripper_open_position_var = tk.StringVar(
            value=str(self.args.gripper_open_position)
        )
        self.baudrate_var = tk.IntVar(value=self.args.baudrate)
        self.connection_status_var = tk.StringVar(value="未连接")
        self.mode_var = tk.StringVar(value="关节空间 MoveJ")
        self.scope_var = tk.StringVar(value="双臂同步")
        self.speed_var = tk.StringVar(value=str(MOTION_DEFAULTS.speed_deg_s))
        self.accel_var = tk.StringVar(value=str(MOTION_DEFAULTS.acceleration_deg_s2))
        self.sample_period_var = tk.StringVar(value=str(MOTION_DEFAULTS.sample_period_s))
        self.duration_var = tk.StringVar(value="")
        self.position_tolerance_var = tk.StringVar(value=str(LINE_DEFAULTS.position_tolerance_mm))
        self.branch_jump_var = tk.StringVar(value=str(LINE_DEFAULTS.max_branch_jump_deg))
        self.allow_recommendation_var = tk.BooleanVar(value=True)
        self.synchronize_finish_var = tk.BooleanVar(value=True)
        self.yaw_min_var = tk.StringVar(value=str(RECOMMENDATION_DEFAULTS.yaw_min_deg))
        self.yaw_max_var = tk.StringVar(value=str(RECOMMENDATION_DEFAULTS.yaw_max_deg))
        self.yaw_step_var = tk.StringVar(value=str(RECOMMENDATION_DEFAULTS.yaw_step_deg))
        self.motion_status_var = tk.StringVar(value="等待目标")
        self.arm_zero_joint_var = tk.StringVar(value="j1")
        self.monitor_period_var = tk.StringVar(value=str(TELEMETRY_DEFAULTS.sample_period_s))
        self.monitor_arm_var = tk.StringVar(value="left")
        self.monitor_joint_var = tk.StringVar(value="j1")
        self.monitor_status_var = tk.StringVar(value="监控未启动")
        self.model_arm_var = tk.StringVar(value="right")
        self.model_use_recommendation_var = tk.BooleanVar(value=True)
        self.model_status_var = tk.StringVar(value="等待目标坐标")
        self.model_target_text_var = tk.StringVar(value="目标 TCP: -")
        self.model_theory_text_var = tk.StringVar(value="理论 TCP: -")
        self.model_feedback_text_var = tk.StringVar(value="反馈 TCP: -")
        self.model_error_text_var = tk.StringVar(value="误差: -")
        self.model_joint_text_var = tk.StringVar(value="理论关节角: -")
        self.model_theoretical: IdealArmModel | None = None
        self.model_feedback: IdealArmModel | None = None
        self.model_target_mm: np.ndarray | None = None
        self.model_requested_pose: np.ndarray | None = None
        self.model_recommendation = None
        self.gripper_scope_var = tk.StringVar(
            value=(
                "仅右夹爪"
                if not self.args.left_gripper_enabled
                and self.args.right_gripper_enabled
                else "双夹爪"
            )
        )
        self.left_gripper_opening_var = tk.DoubleVar(value=100.0)
        self.right_gripper_opening_var = tk.DoubleVar(value=100.0)
        self.gripper_velocity_var = tk.StringVar(value="0.2")
        self.gripper_sample_period_var = tk.StringVar(value="0.1")
        self.gripper_status_var = tk.StringVar(value="夹爪等待连接")
        self.left_gripper_state_var = tk.StringVar(value="左夹爪: -")
        self.right_gripper_state_var = tk.StringVar(value="右夹爪: -")

    def _build_ui(self) -> None:
        header = ttk.Frame(self, padding=(20, 14, 20, 10))
        header.pack(fill="x")
        ttk.Label(header, text="SANPO 双臂运动控制台", style="Header.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.connection_status_var, style="Status.TLabel").pack(
            side="right", padx=(12, 0)
        )

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=16, pady=(0, 16))
        self.connection_tab = ttk.Frame(self.notebook, padding=18)
        self.motion_tab = ttk.Frame(self.notebook, padding=14)
        self.gripper_tab = ttk.Frame(self.notebook, padding=14)
        self.model_tab = ttk.Frame(self.notebook, padding=14)
        self.monitor_tab = ttk.Frame(self.notebook, padding=14)
        self.parameters_tab = ttk.Frame(self.notebook, padding=18)
        self.notebook.add(self.connection_tab, text="连接")
        self.notebook.add(self.motion_tab, text="双臂运动")
        self.notebook.add(self.gripper_tab, text="双夹爪")
        self.notebook.add(self.model_tab, text="理论模型")
        self.notebook.add(self.monitor_tab, text="实时反馈")
        self.notebook.add(self.parameters_tab, text="参数")
        self._build_connection_tab()
        self._build_motion_tab()
        self._build_gripper_tab()
        self._build_model_tab()
        self._build_monitor_tab()
        self._build_parameters_tab()

    def _build_connection_tab(self) -> None:
        outer = ttk.Frame(self.connection_tab)
        outer.pack(fill="x", anchor="n")
        device = ttk.LabelFrame(outer, text="控制板连接", padding=16)
        device.pack(fill="x")
        ttk.Checkbutton(
            device,
            text="仿真模式",
            variable=self.simulate_var,
            command=self._toggle_connection_fields,
        ).grid(row=0, column=0, sticky="w", pady=(0, 12))
        ttk.Label(device, text="左臂 F4 串口").grid(row=1, column=0, sticky="w", pady=5)
        self.left_port_box = ttk.Combobox(
            device, textvariable=self.left_port_var, values=self.port_values, width=18
        )
        self.left_port_box.grid(row=1, column=1, sticky="ew", padx=(12, 28), pady=5)
        ttk.Label(device, text="右臂 F4 串口").grid(row=1, column=2, sticky="w", pady=5)
        self.right_port_box = ttk.Combobox(
            device, textvariable=self.right_port_var, values=self.port_values, width=18
        )
        self.right_port_box.grid(row=1, column=3, sticky="ew", padx=(12, 0), pady=5)
        ttk.Label(device, text="USB 协议").grid(row=2, column=0, sticky="w", pady=5)
        self.usb_mode_box = ttk.Combobox(
            device,
            textvariable=self.usb_mode_var,
            values=("AT 高级模式", "ST 标准模式"),
            state="readonly",
            width=18,
        )
        self.usb_mode_box.grid(row=2, column=1, sticky="ew", padx=(12, 28), pady=5)
        ttk.Label(device, text="串口波特率").grid(row=2, column=2, sticky="w", pady=5)
        self.baud_entry = ttk.Entry(device, textvariable=self.baudrate_var, width=20)
        self.baud_entry.grid(row=2, column=3, sticky="ew", padx=(12, 0), pady=5)
        ttk.Label(device, text="左臂 CAN 通道").grid(row=3, column=0, sticky="w", pady=5)
        self.left_channel_spin = ttk.Spinbox(
            device, from_=1, to=2, textvariable=self.left_channel_var, width=17
        )
        self.left_channel_spin.grid(row=3, column=1, sticky="ew", padx=(12, 28), pady=5)
        ttk.Label(device, text="右臂 CAN 通道").grid(row=3, column=2, sticky="w", pady=5)
        self.right_channel_spin = ttk.Spinbox(
            device, from_=1, to=2, textvariable=self.right_channel_var, width=17
        )
        self.right_channel_spin.grid(row=3, column=3, sticky="ew", padx=(12, 0), pady=5)
        self.left_gripper_enabled_check = ttk.Checkbutton(
            device,
            text="启用左夹爪 · CAN 通道",
            variable=self.left_gripper_enabled_var,
            command=self._toggle_connection_fields,
        )
        self.left_gripper_enabled_check.grid(
            row=4, column=0, sticky="w", pady=5
        )
        self.left_gripper_channel_spin = ttk.Spinbox(
            device,
            from_=1,
            to=2,
            textvariable=self.left_gripper_channel_var,
            width=17,
        )
        self.left_gripper_channel_spin.grid(
            row=4,
            column=1,
            sticky="ew",
            padx=(12, 28),
            pady=5,
        )
        self.right_gripper_enabled_check = ttk.Checkbutton(
            device,
            text="启用右夹爪 · CAN 通道",
            variable=self.right_gripper_enabled_var,
            command=self._toggle_connection_fields,
        )
        self.right_gripper_enabled_check.grid(
            row=4, column=2, sticky="w", pady=5
        )
        self.right_gripper_channel_spin = ttk.Spinbox(
            device,
            from_=1,
            to=2,
            textvariable=self.right_gripper_channel_var,
            width=17,
        )
        self.right_gripper_channel_spin.grid(
            row=4,
            column=3,
            sticky="ew",
            padx=(12, 0),
            pady=5,
        )
        ttk.Label(device, text="左夹爪 Motor ID").grid(
            row=5, column=0, sticky="w", pady=5
        )
        self.left_gripper_id_spin = ttk.Spinbox(
            device,
            from_=0,
            to=15,
            textvariable=self.left_gripper_id_var,
            width=17,
        )
        self.left_gripper_id_spin.grid(
            row=5,
            column=1,
            sticky="ew",
            padx=(12, 28),
            pady=5,
        )
        ttk.Label(device, text="右夹爪 Motor ID").grid(
            row=5, column=2, sticky="w", pady=5
        )
        self.right_gripper_id_spin = ttk.Spinbox(
            device,
            from_=0,
            to=15,
            textvariable=self.right_gripper_id_var,
            width=17,
        )
        self.right_gripper_id_spin.grid(
            row=5,
            column=3,
            sticky="ew",
            padx=(12, 0),
            pady=5,
        )
        ttk.Label(device, text="左夹爪 Master CAN ID").grid(
            row=6, column=0, sticky="w", pady=5
        )
        self.left_gripper_master_id_spin = ttk.Spinbox(
            device,
            from_=0,
            to=0x7FF,
            textvariable=self.left_gripper_master_id_var,
            width=17,
        )
        self.left_gripper_master_id_spin.grid(
            row=6,
            column=1,
            sticky="ew",
            padx=(12, 28),
            pady=5,
        )
        ttk.Label(device, text="右夹爪 Master CAN ID").grid(
            row=6, column=2, sticky="w", pady=5
        )
        self.right_gripper_master_id_spin = ttk.Spinbox(
            device,
            from_=0,
            to=0x7FF,
            textvariable=self.right_gripper_master_id_var,
            width=17,
        )
        self.right_gripper_master_id_spin.grid(
            row=6,
            column=3,
            sticky="ew",
            padx=(12, 0),
            pady=5,
        )
        ttk.Label(device, text="夹爪全闭位置 (rad)").grid(
            row=7, column=0, sticky="w", pady=5
        )
        self.gripper_closed_position_entry = ttk.Entry(
            device,
            textvariable=self.gripper_closed_position_var,
            width=20,
        )
        self.gripper_closed_position_entry.grid(
            row=7,
            column=1,
            sticky="ew",
            padx=(12, 28),
            pady=5,
        )
        ttk.Label(device, text="夹爪全开位置 (rad)").grid(
            row=7, column=2, sticky="w", pady=5
        )
        self.gripper_open_position_entry = ttk.Entry(
            device,
            textvariable=self.gripper_open_position_var,
            width=20,
        )
        self.gripper_open_position_entry.grid(
            row=7,
            column=3,
            sticky="ew",
            padx=(12, 0),
            pady=5,
        )
        for column in (1, 3):
            device.columnconfigure(column, weight=1)

        actions = ttk.Frame(outer, padding=(0, 16, 0, 0))
        actions.pack(fill="x")
        self.connect_button = ttk.Button(
            actions, text="连接并同步", style="Accent.TButton", command=self._connect
        )
        self.connect_button.pack(side="left")
        ttk.Button(actions, text="断开", command=self._disconnect).pack(side="left", padx=8)
        ttk.Button(actions, text="刷新串口", command=self._refresh_ports).pack(side="left")
        self._toggle_connection_fields()

    def _build_motion_tab(self) -> None:
        controls = ttk.Frame(self.motion_tab)
        controls.pack(fill="x")
        ttk.Label(controls, text="执行范围").pack(side="left")
        ttk.Combobox(
            controls,
            textvariable=self.scope_var,
            values=tuple(SCOPE_LABELS),
            state="readonly",
            width=12,
        ).pack(side="left", padx=(8, 20))
        ttk.Label(controls, text="运动方式").pack(side="left")
        mode_box = ttk.Combobox(
            controls,
            textvariable=self.mode_var,
            values=tuple(MODE_LABELS),
            state="readonly",
            width=24,
        )
        mode_box.pack(side="left", padx=8)
        mode_box.bind("<<ComboboxSelected>>", lambda _event: self._update_target_mode())
        ttk.Checkbutton(
            controls, text="无解时使用推荐姿态", variable=self.allow_recommendation_var
        ).pack(side="left", padx=14)
        ttk.Checkbutton(
            controls, text="同步结束", variable=self.synchronize_finish_var
        ).pack(side="left")

        content = ttk.Panedwindow(self.motion_tab, orient="vertical")
        content.pack(fill="both", expand=True, pady=(12, 0))
        target_area = ttk.Frame(content)
        plot_area = ttk.Frame(content)
        content.add(target_area, weight=2)
        content.add(plot_area, weight=3)

        panels = ttk.Frame(target_area)
        panels.pack(fill="both", expand=True)
        self.left_target = TargetPanel(panels, "左臂目标")
        self.right_target = TargetPanel(panels, "右臂目标")
        panels.columnconfigure(0, weight=1, uniform="arm-targets")
        panels.columnconfigure(1, weight=1, uniform="arm-targets")
        panels.rowconfigure(0, weight=1)
        self.left_target.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.right_target.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        command_bar = ttk.Frame(target_area, padding=(0, 10, 0, 0))
        command_bar.pack(fill="x")
        ttk.Button(command_bar, text="读取当前值", command=self._load_current_targets).pack(
            side="left"
        )
        ttk.Button(command_bar, text="推荐可行姿态", command=self._recommend_targets).pack(
            side="left", padx=8
        )
        ttk.Button(command_bar, text="规划预览", command=self._preview_motion).pack(
            side="left"
        )
        self.execute_button = ttk.Button(
            command_bar, text="执行运动", style="Accent.TButton", command=self._execute_motion
        )
        self.execute_button.pack(side="right")
        ttk.Button(
            command_bar, text="停止并失能", style="Danger.TButton", command=self._stop_motion
        ).pack(side="right", padx=8)
        ttk.Label(command_bar, textvariable=self.motion_status_var, style="Sub.TLabel").pack(
            side="right", padx=18
        )

        maintenance_bar = ttk.Frame(target_area, padding=(0, 6, 0, 0))
        maintenance_bar.pack(fill="x")
        ttk.Label(maintenance_bar, text="关节零点").pack(side="left")
        ttk.Combobox(
            maintenance_bar,
            textvariable=self.arm_zero_joint_var,
            values=JOINT_KEYS,
            state="readonly",
            width=6,
        ).pack(side="left", padx=(8, 6))
        self.arm_zero_button = ttk.Button(
            maintenance_bar,
            text="将当前位置设为零点",
            style="Danger.TButton",
            command=self._set_arm_zero,
        )
        self.arm_zero_button.pack(side="left")

        self.preview_figure = Figure(figsize=(9, 3.5), dpi=100, facecolor="#eef1f4")
        self.preview_axes = self.preview_figure.subplots(1, 2)
        self.preview_canvas = FigureCanvasTkAgg(self.preview_figure, master=plot_area)
        self.preview_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty_preview()

    def _build_gripper_tab(self) -> None:
        toolbar = ttk.Frame(self.gripper_tab)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="控制范围").pack(side="left")
        ttk.Combobox(
            toolbar,
            textvariable=self.gripper_scope_var,
            values=("双夹爪", "仅左夹爪", "仅右夹爪"),
            state="readonly",
            width=12,
        ).pack(side="left", padx=(8, 20))
        ttk.Label(toolbar, text="速度 (rad/s)").pack(side="left")
        ttk.Entry(
            toolbar,
            textvariable=self.gripper_velocity_var,
            width=10,
        ).pack(side="left", padx=(8, 18))

        self.gripper_buttons: list[ttk.Button] = []
        actions = (
            ("使能", "enable", "Accent.TButton"),
            ("移动", "move", "Accent.TButton"),
            ("全开", "open", None),
            ("闭合", "close", None),
            ("刷新", "refresh", None),
            ("清错", "clear", None),
            ("设零", "zero", "Danger.TButton"),
            ("失能", "disable", "Danger.TButton"),
        )
        for label, action, style in actions:
            button = ttk.Button(
                toolbar,
                text=label,
                command=lambda selected=action: self._run_gripper_action(selected),
            )
            if style is not None:
                button.configure(style=style)
            button.pack(side="left", padx=3)
            self.gripper_buttons.append(button)

        body = ttk.Panedwindow(self.gripper_tab, orient="vertical")
        body.pack(fill="both", expand=True, pady=(12, 0))
        control_area = ttk.Frame(body)
        plot_area = ttk.Frame(body)
        body.add(control_area, weight=2)
        body.add(plot_area, weight=3)

        left_panel = ttk.LabelFrame(control_area, text="左夹爪", padding=14)
        right_panel = ttk.LabelFrame(control_area, text="右夹爪", padding=14)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 6))
        right_panel.pack(side="left", fill="both", expand=True, padx=(6, 0))
        self._build_gripper_side_panel(
            left_panel,
            self.left_gripper_opening_var,
            self.left_gripper_state_var,
        )
        self._build_gripper_side_panel(
            right_panel,
            self.right_gripper_opening_var,
            self.right_gripper_state_var,
        )

        monitor_toolbar = ttk.Frame(plot_area)
        monitor_toolbar.pack(fill="x", pady=(0, 6))
        ttk.Label(monitor_toolbar, text="采样周期 (s)").pack(side="left")
        ttk.Entry(
            monitor_toolbar,
            textvariable=self.gripper_sample_period_var,
            width=9,
        ).pack(side="left", padx=(8, 14))
        ttk.Button(
            monitor_toolbar,
            text="开始记录",
            command=self._start_gripper_monitor,
        ).pack(side="left", padx=3)
        ttk.Button(
            monitor_toolbar,
            text="停止",
            command=self._stop_gripper_monitor,
        ).pack(side="left", padx=3)
        ttk.Button(
            monitor_toolbar,
            text="清空",
            command=self._clear_gripper_monitor,
        ).pack(side="left", padx=3)
        ttk.Button(
            monitor_toolbar,
            text="导出 CSV",
            command=self._export_gripper_csv,
        ).pack(side="left", padx=(16, 3))
        ttk.Button(
            monitor_toolbar,
            text="导出曲线 PNG",
            command=self._export_gripper_plot,
        ).pack(side="left", padx=3)
        ttk.Label(
            monitor_toolbar,
            textvariable=self.gripper_status_var,
            style="Status.TLabel",
        ).pack(side="right")

        self.gripper_figure = Figure(figsize=(10, 4), dpi=100)
        self.gripper_opening_axes = self.gripper_figure.add_subplot(211)
        self.gripper_effort_axes = self.gripper_figure.add_subplot(212)
        self.gripper_figure.subplots_adjust(
            left=0.08,
            right=0.98,
            top=0.94,
            bottom=0.12,
            hspace=0.38,
        )
        self.gripper_canvas = FigureCanvasTkAgg(
            self.gripper_figure,
            master=plot_area,
        )
        self.gripper_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty_gripper_plot()

    @staticmethod
    def _build_gripper_side_panel(
        parent: ttk.LabelFrame,
        opening_variable: tk.DoubleVar,
        state_variable: tk.StringVar,
    ) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Label(row, text="目标开度 (%)").pack(side="left")
        ttk.Entry(
            row,
            textvariable=opening_variable,
            width=9,
        ).pack(side="right")
        ttk.Scale(
            parent,
            from_=0.0,
            to=100.0,
            orient="horizontal",
            variable=opening_variable,
        ).pack(fill="x", pady=(8, 12))
        ttk.Label(
            parent,
            textvariable=state_variable,
            justify="left",
        ).pack(fill="x", anchor="w")

    def _draw_empty_gripper_plot(self) -> None:
        for axes in (self.gripper_opening_axes, self.gripper_effort_axes):
            axes.clear()
            axes.grid(True, color="#d8dee4", linewidth=0.7)
        self.gripper_opening_axes.set_ylabel("开度 (%)")
        self.gripper_opening_axes.set_ylim(-5, 105)
        self.gripper_effort_axes.set_ylabel("力矩 (Nm)")
        self.gripper_effort_axes.set_xlabel("时间 (s)")
        self.gripper_canvas.draw_idle()

    def _build_model_tab(self) -> None:
        self.model_tab.columnconfigure(0, weight=0, minsize=360)
        self.model_tab.columnconfigure(1, weight=1)
        self.model_tab.rowconfigure(0, weight=1)

        controls = ttk.Frame(self.model_tab)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        visualization = ttk.Frame(self.model_tab)
        visualization.grid(row=0, column=1, sticky="nsew")
        visualization.columnconfigure(0, weight=1)
        visualization.rowconfigure(0, weight=3)
        visualization.rowconfigure(1, weight=2)

        selector = ttk.Frame(controls)
        selector.pack(fill="x", pady=(0, 8))
        ttk.Label(selector, text="机械臂").pack(side="left")
        ttk.Combobox(
            selector,
            textvariable=self.model_arm_var,
            values=("left", "right"),
            state="readonly",
            width=9,
        ).pack(side="left", padx=8)
        ttk.Checkbutton(
            selector,
            text="无解时推荐 Yaw（J5 保持不变）",
            variable=self.model_use_recommendation_var,
        ).pack(side="left", padx=(8, 0))

        self.model_target_panel = TargetPanel(controls, "目标 TCP（肩部基坐标系）")
        self.model_target_panel.set_mode("cartesian")
        default_model = build_ideal_arm_model(self._default_model_seed())
        self.model_target_panel.set_values(
            [
                *default_model.tcp_position_mm,
                default_model.pose.yaw_deg,
                default_model.q_deg[4],
            ]
        )
        self.model_target_panel.pack(fill="x")

        target_actions = ttk.Frame(controls, padding=(0, 9, 0, 0))
        target_actions.pack(fill="x")
        ttk.Button(
            target_actions,
            text="从运动页载入",
            command=self._load_model_target_from_motion,
        ).pack(side="left")
        ttk.Button(
            target_actions,
            text="计算理论模型",
            style="Accent.TButton",
            command=self._calculate_theoretical_model,
        ).pack(side="left", padx=6)
        ttk.Button(
            target_actions,
            text="读取反馈对比",
            command=self._read_model_feedback,
        ).pack(side="left")

        comparison = ttk.LabelFrame(controls, text="位置与误差", padding=12)
        comparison.pack(fill="x", pady=(12, 0))
        for variable in (
            self.model_target_text_var,
            self.model_theory_text_var,
            self.model_feedback_text_var,
            self.model_error_text_var,
            self.model_joint_text_var,
        ):
            ttk.Label(
                comparison,
                textvariable=variable,
                justify="left",
                wraplength=330,
            ).pack(fill="x", anchor="w", pady=3)

        export_actions = ttk.Frame(controls, padding=(0, 12, 0, 0))
        export_actions.pack(fill="x")
        ttk.Button(
            export_actions,
            text="导出模型 CSV",
            command=self._export_model_csv,
        ).pack(side="left")
        ttk.Button(
            export_actions,
            text="导出 3D 图",
            command=self._export_model_plot,
        ).pack(side="left", padx=6)
        ttk.Label(
            controls,
            textvariable=self.model_status_var,
            style="Status.TLabel",
            wraplength=340,
        ).pack(fill="x", anchor="w", pady=(12, 0))

        plot_frame = ttk.Frame(visualization)
        plot_frame.grid(row=0, column=0, sticky="nsew")
        self.model_figure = Figure(figsize=(8, 5.2), dpi=100, facecolor="#eef1f4")
        self.model_axis = self.model_figure.add_subplot(111, projection="3d")
        self.model_canvas = FigureCanvasTkAgg(self.model_figure, master=plot_frame)
        NavigationToolbar2Tk(self.model_canvas, plot_frame, pack_toolbar=False).pack(fill="x")
        self.model_canvas.get_tk_widget().pack(fill="both", expand=True)

        table_frame = ttk.LabelFrame(
            visualization,
            text="理论坐标系（原点和轴向均在 Base 中表达）",
            padding=6,
        )
        table_frame.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        columns = ("frame", "x", "y", "z", "x_axis", "y_axis", "z_axis")
        self.model_frame_table = ttk.Treeview(
            table_frame,
            columns=columns,
            show="headings",
            height=7,
        )
        headings = {
            "frame": "坐标系",
            "x": "原点 X",
            "y": "原点 Y",
            "z": "原点 Z",
            "x_axis": "X 轴单位向量",
            "y_axis": "Y 轴单位向量",
            "z_axis": "Z 轴单位向量",
        }
        widths = {
            "frame": 65,
            "x": 85,
            "y": 85,
            "z": 85,
            "x_axis": 145,
            "y_axis": 145,
            "z_axis": 145,
        }
        for column in columns:
            self.model_frame_table.heading(column, text=headings[column])
            self.model_frame_table.column(
                column,
                width=widths[column],
                anchor="center",
            )
        table_scroll = ttk.Scrollbar(
            table_frame,
            orient="vertical",
            command=self.model_frame_table.yview,
        )
        self.model_frame_table.configure(yscrollcommand=table_scroll.set)
        self.model_frame_table.pack(side="left", fill="both", expand=True)
        table_scroll.pack(side="right", fill="y")
        self._draw_empty_model()

    def _build_monitor_tab(self) -> None:
        toolbar = ttk.Frame(self.monitor_tab)
        toolbar.pack(fill="x", pady=(0, 10))
        ttk.Button(toolbar, text="开始记录", style="Accent.TButton", command=self._start_monitor).pack(
            side="left"
        )
        ttk.Button(toolbar, text="停止记录", command=self._stop_monitor).pack(side="left", padx=6)
        ttk.Button(toolbar, text="清空", command=self._clear_monitor).pack(side="left")
        ttk.Button(toolbar, text="导出 CSV", command=self._export_csv).pack(side="left", padx=(18, 6))
        ttk.Button(toolbar, text="导出曲线 PNG", command=self._export_plot).pack(side="left")
        ttk.Label(toolbar, text="显示").pack(side="left", padx=(24, 6))
        ttk.Combobox(
            toolbar,
            textvariable=self.monitor_arm_var,
            values=("left", "right"),
            state="readonly",
            width=8,
        ).pack(side="left")
        ttk.Combobox(
            toolbar,
            textvariable=self.monitor_joint_var,
            values=JOINT_KEYS,
            state="readonly",
            width=7,
        ).pack(side="left", padx=6)
        ttk.Label(toolbar, textvariable=self.monitor_status_var, style="Status.TLabel").pack(
            side="right"
        )

        split = ttk.Panedwindow(self.monitor_tab, orient="horizontal")
        split.pack(fill="both", expand=True)
        table_frame = ttk.Frame(split)
        plot_frame = ttk.Frame(split)
        split.add(table_frame, weight=2)
        split.add(plot_frame, weight=5)
        columns = (
            "arm",
            "joint",
            "angle",
            "speed",
            "current",
            "angle_range",
            "speed_peak",
            "current_peak",
            "count",
        )
        self.feedback_table = ttk.Treeview(
            table_frame, columns=columns, show="headings", height=12
        )
        headings = {
            "arm": "臂",
            "joint": "关节",
            "angle": "角度 deg",
            "speed": "速度 rpm",
            "current": "电流 A",
            "angle_range": "角度范围",
            "speed_peak": "|速度|峰值",
            "current_peak": "|电流|峰值",
            "count": "样本",
        }
        widths = {
            "arm": 55,
            "joint": 50,
            "angle": 80,
            "speed": 80,
            "current": 75,
            "angle_range": 135,
            "speed_peak": 90,
            "current_peak": 90,
            "count": 65,
        }
        for column in columns:
            self.feedback_table.heading(column, text=headings[column])
            self.feedback_table.column(column, width=widths[column], anchor="center")
        for arm in ("left", "right"):
            for joint in JOINT_KEYS:
                self.feedback_table.insert(
                    "", "end", iid=f"{arm}:{joint}", values=(arm, joint, "-", "-", "-", "-", "-", "-", 0)
                )
        scrollbar = ttk.Scrollbar(
            table_frame, orient="vertical", command=self.feedback_table.yview
        )
        self.feedback_table.configure(yscrollcommand=scrollbar.set)
        self.feedback_table.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.monitor_figure = Figure(figsize=(7.5, 6), dpi=100, facecolor="#eef1f4")
        self.monitor_axes = self.monitor_figure.subplots(3, 1, sharex=True)
        self.monitor_canvas = FigureCanvasTkAgg(self.monitor_figure, master=plot_frame)
        NavigationToolbar2Tk(self.monitor_canvas, plot_frame, pack_toolbar=False).pack(fill="x")
        self.monitor_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty_monitor()

    def _build_parameters_tab(self) -> None:
        motion = ttk.LabelFrame(self.parameters_tab, text="轨迹参数", padding=16)
        recommendation = ttk.LabelFrame(self.parameters_tab, text="无解推荐搜索", padding=16)
        feedback = ttk.LabelFrame(self.parameters_tab, text="实时反馈", padding=16)
        motion.pack(fill="x", pady=(0, 12))
        recommendation.pack(fill="x", pady=(0, 12))
        feedback.pack(fill="x")
        fields = (
            ("速度上限 (deg/s)", self.speed_var),
            ("加速度上限 (deg/s²)", self.accel_var),
            ("轨迹采样周期 (s)", self.sample_period_var),
            ("指定时长 (s，留空自动)", self.duration_var),
            ("直线位置容差 (mm)", self.position_tolerance_var),
            ("相邻点最大跳变 (deg)", self.branch_jump_var),
        )
        self._parameter_grid(motion, fields)
        self._parameter_grid(
            recommendation,
            (
                ("Yaw 最小值 (deg)", self.yaw_min_var),
                ("Yaw 最大值 (deg)", self.yaw_max_var),
                ("Yaw 搜索步长 (deg)", self.yaw_step_var),
            ),
        )
        self._parameter_grid(
            feedback,
            (("采样周期 (s)", self.monitor_period_var),),
        )

    @staticmethod
    def _parameter_grid(
        parent: ttk.LabelFrame,
        fields: tuple[tuple[str, tk.Variable], ...],
    ) -> None:
        for index, (label, variable) in enumerate(fields):
            row, pair = divmod(index, 3)
            column = pair * 2
            ttk.Label(parent, text=label).grid(row=row, column=column, sticky="w", pady=5)
            ttk.Entry(parent, textvariable=variable, width=18).grid(
                row=row, column=column + 1, sticky="ew", padx=(8, 24), pady=5
            )
            parent.columnconfigure(column + 1, weight=1)

    def _toggle_connection_fields(self) -> None:
        state = "disabled" if self.simulate_var.get() else "normal"
        for widget in (
            self.left_port_box,
            self.right_port_box,
            self.usb_mode_box,
            self.baud_entry,
            self.left_channel_spin,
            self.right_channel_spin,
            self.gripper_closed_position_entry,
            self.gripper_open_position_entry,
        ):
            widget.configure(state=state)

        left_state = (
            state if self.left_gripper_enabled_var.get() else "disabled"
        )
        right_state = (
            state if self.right_gripper_enabled_var.get() else "disabled"
        )
        for widget in (
            self.left_gripper_channel_spin,
            self.left_gripper_id_spin,
            self.left_gripper_master_id_spin,
        ):
            widget.configure(state=left_state)
        for widget in (
            self.right_gripper_channel_spin,
            self.right_gripper_id_spin,
            self.right_gripper_master_id_spin,
        ):
            widget.configure(state=right_state)
        if not self.simulate_var.get():
            self.usb_mode_box.configure(state="readonly")

    def _refresh_ports(self) -> None:
        self.port_values = list_serial_ports()
        self.left_port_box.configure(values=self.port_values)
        self.right_port_box.configure(values=self.port_values)

    def _set_busy(self, busy: bool, text: str | None = None) -> None:
        self._busy = busy
        self.connect_button.configure(state="disabled" if busy else "normal")
        self.execute_button.configure(state="disabled" if busy else "normal")
        self.arm_zero_button.configure(state="disabled" if busy else "normal")
        for button in self.gripper_buttons:
            button.configure(state="disabled" if busy else "normal")
        if text:
            self.motion_status_var.set(text)

    def _run_async(
        self,
        label: str,
        function: Callable[[], object],
        callback: Optional[Callable[[object], None]] = None,
    ) -> None:
        if self._busy:
            messagebox.showinfo("任务进行中", "请等待当前任务完成。")
            return
        self._set_busy(True, label)

        def worker() -> None:
            try:
                result = function()
                self._events.put(("success", result, callback))
            except Exception as exc:
                self._events.put(("error", exc, label))

        threading.Thread(target=worker, daemon=True, name=f"ui-{label}").start()

    def _drain_events(self) -> None:
        try:
            while True:
                kind, value, extra = self._events.get_nowait()
                self._set_busy(False)
                if kind == "error":
                    self.motion_status_var.set(f"{extra}失败")
                    messagebox.showerror("操作失败", f"{extra}\n\n{value}")
                elif callable(extra):
                    extra(value)
        except queue.Empty:
            pass
        self.after(80, self._drain_events)

    def _connect(self) -> None:
        if self.system is not None:
            messagebox.showinfo("已连接", "请先断开当前控制器。")
            return
        try:
            simulate = self.simulate_var.get()
            left_port = self.left_port_var.get().strip()
            right_port = self.right_port_var.get().strip()
            usb_mode = (
                "advanced"
                if self.usb_mode_var.get().startswith("AT")
                else "standard"
            )
            baudrate = int(self.baudrate_var.get())
            left_channel = int(self.left_channel_var.get())
            right_channel = int(self.right_channel_var.get())
            left_gripper_channel = int(self.left_gripper_channel_var.get())
            right_gripper_channel = int(self.right_gripper_channel_var.get())
            left_gripper_enabled = bool(self.left_gripper_enabled_var.get())
            right_gripper_enabled = bool(self.right_gripper_enabled_var.get())
            calibration = GripperCalibration(
                closed_position_rad=float(
                    self.gripper_closed_position_var.get()
                ),
                open_position_rad=float(self.gripper_open_position_var.get()),
            )
            left_gripper_config = GloriaGripperConfig(
                motor_id=int(self.left_gripper_id_var.get()),
                master_can_id=int(self.left_gripper_master_id_var.get()),
                calibration=calibration,
            )
            right_gripper_config = GloriaGripperConfig(
                motor_id=int(self.right_gripper_id_var.get()),
                master_can_id=int(self.right_gripper_master_id_var.get()),
                calibration=calibration,
            )
        except Exception as exc:
            messagebox.showerror("连接参数错误", str(exc))
            return
        if not simulate and usb_mode != "standard":
            messagebox.showerror(
                "协议模式错误",
                "机械臂和夹爪使用同一 F4 的两个独立 CAN 口时必须选择 ST 标准模式。",
            )
            return

        def task() -> tuple[DualF4System, object]:
            system = (
                create_dual_simulated_system(
                    left_gripper_enabled=left_gripper_enabled,
                    right_gripper_enabled=right_gripper_enabled,
                    left_gripper_config=left_gripper_config,
                    right_gripper_config=right_gripper_config,
                )
                if simulate
                else create_dual_f4_system(
                    left_port,
                    right_port,
                    baudrate=baudrate,
                    left_arm_channel=left_channel,
                    left_gripper_channel=left_gripper_channel,
                    right_arm_channel=right_channel,
                    right_gripper_channel=right_gripper_channel,
                    left_gripper_enabled=left_gripper_enabled,
                    right_gripper_enabled=right_gripper_enabled,
                    left_gripper_config=left_gripper_config,
                    right_gripper_config=right_gripper_config,
                )
            )
            result = system.connect()
            if not result.success:
                system.close()
                raise RuntimeError(
                    "连接失败\n"
                    f"左臂: {err_text(result.arms.left_error)}; "
                    f"右臂: {err_text(result.arms.right_error)}\n"
                    f"左夹爪: {result.grippers.left_error or 'OK'}; "
                    f"右夹爪: {result.grippers.right_error or 'OK'}"
                )
            sync = system.arms.sync_state()
            if not sync.success:
                system.close()
                raise RuntimeError(
                    f"关节反馈同步失败，左臂: {err_text(sync.left_error)}; "
                    f"右臂: {err_text(sync.right_error)}"
                )
            return system, result

        def done(value: object) -> None:
            system, connection = value  # type: ignore[misc]
            self.system = system  # type: ignore[assignment]
            self.dual = system.arms  # type: ignore[union-attr]
            self.grippers = system.grippers  # type: ignore[union-attr]
            mode = "仿真" if simulate else f"{usb_mode.upper()} / {baudrate}"
            self.connection_status_var.set(f"已连接 · {mode}")
            self.motion_status_var.set("双臂状态已同步")
            gripper_result = connection.grippers
            left_status = self._gripper_connection_text(
                "左夹爪",
                gripper_result.left_available,
                gripper_result.left_success,
                gripper_result.left_error,
            )
            right_status = self._gripper_connection_text(
                "右夹爪",
                gripper_result.right_available,
                gripper_result.right_success,
                gripper_result.right_error,
            )
            self.gripper_status_var.set(f"{left_status}；{right_status}")
            if not gripper_result.left_available:
                self.left_gripper_state_var.set("左夹爪: 未启用")
            elif not gripper_result.left_success:
                self.left_gripper_state_var.set(
                    f"左夹爪: 离线\n{gripper_result.left_error}"
                )
            if not gripper_result.right_available:
                self.right_gripper_state_var.set("右夹爪: 未启用")
            elif not gripper_result.right_success:
                self.right_gripper_state_var.set(
                    f"右夹爪: 离线\n{gripper_result.right_error}"
                )
            self.notebook.select(self.motion_tab)
            self._load_current_targets()
            self._refresh_gripper_display()

        self._run_async("连接并同步", task, done)

    def _disconnect(self) -> None:
        system = self.system
        if system is None:
            return
        self._stop_monitor()
        self._stop_gripper_monitor()

        def task() -> object:
            system.arms.stop(disable=True)
            system.close()
            return None

        def done(_result: object) -> None:
            self.system = None
            self.dual = None
            self.grippers = None
            self.connection_status_var.set("未连接")
            self.motion_status_var.set("等待连接")
            self.gripper_status_var.set("夹爪等待连接")

        self._run_async("断开控制器", task, done)

    def _require_dual(self) -> DualArmController:
        if self.dual is None:
            raise RuntimeError("控制器尚未连接")
        return self.dual

    def _require_grippers(self) -> DualGripperController:
        if self.grippers is None:
            raise RuntimeError("夹爪控制器尚未连接")
        return self.grippers

    @staticmethod
    def _gripper_connection_text(
        label: str,
        available: bool,
        success: bool,
        error: str,
    ) -> str:
        if not available:
            return f"{label}: 未启用"
        if success:
            return f"{label}: 在线"
        return f"{label}: 离线 ({error or '无反馈'})"

    @staticmethod
    def _check_gripper_result(result: object) -> None:
        if hasattr(result, "success") and not result.success:
            raise RuntimeError(
                f"左夹爪: {result.left_error or 'OK'}; "
                f"右夹爪: {result.right_error or 'OK'}"
            )

    def _selected_grippers(self) -> tuple[str, ...]:
        return {
            "双夹爪": ("left", "right"),
            "仅左夹爪": ("left",),
            "仅右夹爪": ("right",),
        }[self.gripper_scope_var.get()]

    @staticmethod
    def _state_text(label: str, state: GripperState) -> str:
        opening = (
            "-"
            if state.opening_fraction is None
            else f"{state.opening_fraction * 100.0:.1f}%"
        )
        return (
            f"{label}: {state.status}  开度 {opening}\n"
            f"位置 {state.position_rad:+.4f} rad   "
            f"速度 {state.velocity_rad_s:+.4f} rad/s\n"
            f"力矩 {state.torque_nm:+.4f} Nm   "
            f"MOS/转子 {state.mos_temperature_c}/"
            f"{state.rotor_temperature_c} °C"
        )

    def _update_gripper_states(
        self,
        states: dict[str, GripperState | None],
    ) -> None:
        left = states.get("left")
        right = states.get("right")
        if isinstance(left, GripperState):
            self.left_gripper_state_var.set(self._state_text("左夹爪", left))
        if isinstance(right, GripperState):
            self.right_gripper_state_var.set(self._state_text("右夹爪", right))

    def _refresh_gripper_display(self) -> None:
        if self.grippers is None:
            return
        self._update_gripper_states(
            {
                "left": getattr(self.grippers.left, "state", None),
                "right": getattr(self.grippers.right, "state", None),
            }
        )

    def _run_gripper_action(self, action: str) -> None:
        try:
            grippers = self._require_grippers()
            sides = self._selected_grippers()
            velocity = float(self.gripper_velocity_var.get())
            if velocity <= 0.0:
                raise ValueError("夹爪速度必须大于 0")
            openings = {
                "left": float(self.left_gripper_opening_var.get()) / 100.0,
                "right": float(self.right_gripper_opening_var.get()) / 100.0,
            }
            if any(not 0.0 <= openings[side] <= 1.0 for side in sides):
                raise ValueError("目标开度必须在 0..100% 范围内")
        except Exception as exc:
            messagebox.showerror("夹爪参数错误", str(exc))
            return

        if action == "close" and not messagebox.askyesno(
            "夹爪闭合确认",
            "确认夹持区域无人体，并已设置低速和合适的机械限位？",
        ):
            return
        if action == "zero" and not messagebox.askyesno(
            "夹爪零点确认",
            "确认所选夹爪已经人工放到机械零位？\n\n"
            "该命令会把当前位置写为设备零点，错误设置会导致后续位置命令偏移。",
        ):
            return
        if action == "open":
            for side in sides:
                openings[side] = 1.0
                (
                    self.left_gripper_opening_var
                    if side == "left"
                    else self.right_gripper_opening_var
                ).set(100.0)
        elif action == "close":
            for side in sides:
                openings[side] = 0.0
                (
                    self.left_gripper_opening_var
                    if side == "left"
                    else self.right_gripper_opening_var
                ).set(0.0)

        def task() -> dict[str, GripperState | None]:
            states: dict[str, GripperState | None] = {}
            if len(sides) == 2:
                if action == "enable":
                    self._check_gripper_result(grippers.enable_both())
                elif action == "disable":
                    self._check_gripper_result(grippers.disable_both())
                elif action == "clear":
                    self._check_gripper_result(grippers.clear_faults_both())
                elif action == "zero":
                    self._check_gripper_result(
                        grippers.set_zero_both(confirm=True)
                    )
                elif action in {"move", "open", "close"}:
                    result, states = grippers.move_both(
                        openings["left"],
                        openings["right"],
                        velocity,
                    )
                    self._check_gripper_result(result)
                elif action == "refresh":
                    result, states = grippers.refresh_both()
                    self._check_gripper_result(result)
                if not states:
                    result, states = grippers.refresh_both()
                    self._check_gripper_result(result)
                return states

            side = sides[0]
            gripper = grippers.left if side == "left" else grippers.right
            if action == "enable":
                gripper.enable()
            elif action == "disable":
                gripper.disable()
            elif action == "clear":
                gripper.clear_faults()
            elif action == "zero":
                gripper.set_zero(confirm=True)
            elif action in {"move", "open", "close"}:
                states[side] = gripper.move_normalized(
                    openings[side],
                    velocity,
                )
            elif action == "refresh":
                states[side] = gripper.refresh_state()
            if side not in states:
                states[side] = gripper.refresh_state()
            return states

        def done(states: object) -> None:
            self._update_gripper_states(states)  # type: ignore[arg-type]
            labels = {
                "enable": "使能完成",
                "disable": "失能完成",
                "clear": "清错完成",
                "zero": "夹爪零点设置完成",
                "move": "移动完成",
                "open": "全开命令完成",
                "close": "闭合命令完成",
                "refresh": "状态已刷新",
            }
            self.gripper_status_var.set(labels[action])

        self._run_async(f"夹爪{action}", task, done)

    def _set_arm_zero(self) -> None:
        try:
            dual = self._require_dual()
            joint = self.arm_zero_joint_var.get().strip().lower()
            if joint not in JOINT_KEYS:
                raise ValueError("请选择 J1 到 J5 中的一个关节")
            scope = self._current_scope()
            selected = {
                "dual": (("left", dual.left), ("right", dual.right)),
                "left": (("left", dual.left),),
                "right": (("right", dual.right),),
            }[scope]
            if any(arm.is_moving for _side, arm in selected):
                raise RuntimeError("机械臂运动中，不能修改关节零点")
        except Exception as exc:
            messagebox.showerror("关节零点设置失败", str(exc))
            return

        side_text = {
            "dual": "左右双臂",
            "left": "左臂",
            "right": "右臂",
        }[scope]
        if not messagebox.askyesno(
            "关节零点确认",
            f"确认将{side_text}的 {joint.upper()} 当前位置设为 0°？\n\n"
            "该操作会写入电机机械零点。请先固定机械臂、核对关节姿态并确保当前没有运动。",
        ):
            return

        def task() -> dict[str, int]:
            results: dict[str, int] = {}
            for side, arm in selected:
                result = arm.set_zero(joint)
                results[side] = result
                if result != OK:
                    raise RuntimeError(
                        f"{side} {joint.upper()} 设置失败: {err_text(result)}"
                    )
            return results

        def done(_results: object) -> None:
            self.motion_status_var.set(
                f"{side_text} {joint.upper()} 零点设置完成"
            )
            self._load_current_targets()

        self._run_async("设置关节零点", task, done)

    def _current_mode(self) -> str:
        return MODE_LABELS[self.mode_var.get()]

    def _current_scope(self) -> str:
        return SCOPE_LABELS[self.scope_var.get()]

    def _update_target_mode(self) -> None:
        mode = self._current_mode()
        self.left_target.set_mode(mode)
        self.right_target.set_mode(mode)

    def _load_current_targets(self) -> None:
        if self.dual is None:
            return
        mode = self._current_mode()
        for arm, panel in (
            (self.dual.left, self.left_target),
            (self.dual.right, self.right_target),
        ):
            if mode == "joint":
                values = [arm.joints[key]["current"] for key in JOINT_KEYS]
                panel.set_values(values)
            else:
                error, pose = arm.forward_pose()
                if error == OK and pose is not None:
                    j5 = arm.joints["j5"]["current"]
                    panel.set_values([pose.x, pose.y, pose.z, pose.yaw_deg, j5])

    def _recommend_config(self) -> IKRecommendConfig:
        return IKRecommendConfig(
            yaw_min_deg=float(self.yaw_min_var.get()),
            yaw_max_deg=float(self.yaw_max_var.get()),
            yaw_step_deg=float(self.yaw_step_var.get()),
        )

    def _motion_parameters(self) -> dict[str, object]:
        duration_text = self.duration_var.get().strip()
        return {
            "mode": self._current_mode(),
            "speed": float(self.speed_var.get()),
            "accel": float(self.accel_var.get()),
            "sample_period_s": float(self.sample_period_var.get()),
            "total_time_s": None if not duration_text else float(duration_text),
            "position_tolerance_mm": float(self.position_tolerance_var.get()),
            "max_branch_jump_deg": float(self.branch_jump_var.get()),
            "allow_recommendation": bool(self.allow_recommendation_var.get()),
            "recommendation_config": self._recommend_config(),
        }

    def _prepare_single(
        self,
        arm: ArmController,
        target: list[float],
        parameters: dict[str, object],
    ) -> tuple[int, object]:
        mode = parameters["mode"]
        if mode == "joint":
            return arm.prepare_move_j(
                target,
                speed=parameters["speed"],
                accel=parameters["accel"],
                sample_period_s=float(parameters["sample_period_s"]),
            )
        if mode == "cartesian":
            return arm.prepare_move_cart(
                target,
                speed=parameters["speed"],
                accel=parameters["accel"],
                sample_period_s=float(parameters["sample_period_s"]),
                allow_recommendation=bool(parameters["allow_recommendation"]),
                recommendation_config=parameters["recommendation_config"],
            )
        return arm.prepare_move_line(
            target,
            speed=parameters["speed"],
            accel=parameters["accel"],
            total_time_s=parameters["total_time_s"],
            sample_period_s=float(parameters["sample_period_s"]),
            position_tolerance_mm=float(parameters["position_tolerance_mm"]),
            max_branch_jump_deg=float(parameters["max_branch_jump_deg"]),
        )

    def _preview_motion(self) -> None:
        try:
            dual = self._require_dual()
            left_target = self.left_target.values()
            right_target = self.right_target.values()
            parameters = self._motion_parameters()
            scope = self._current_scope()
            synchronize_finish = self.synchronize_finish_var.get()
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def task() -> dict[str, object]:
            if scope == "dual":
                result, prepared = dual.prepare_both(
                    left_target,
                    right_target,
                    synchronize_finish=synchronize_finish,
                    **parameters,
                )
                if not result.success or prepared is None:
                    raise RuntimeError(
                        f"左臂: {err_text(result.left_error)}; 右臂: {err_text(result.right_error)}"
                    )
                return prepared
            arm = dual.left if scope == "left" else dual.right
            target = left_target if scope == "left" else right_target
            error, motion = self._prepare_single(arm, target, parameters)
            if error != OK or motion is None:
                raise RuntimeError(err_text(error))
            return {scope: motion}

        self._run_async("规划轨迹", task, self._show_prepared)

    def _show_prepared(self, prepared_value: object) -> None:
        prepared = prepared_value  # type: ignore[assignment]
        summaries: list[str] = []
        for side, motion in prepared.items():
            duration = float(motion.trajectory.time_s[-1])
            summary = f"{side}: {motion.trajectory.point_count} 点 / {duration:.3f} s"
            if motion.line_plan is not None:
                summary += f" / 直线偏差峰值 {motion.line_plan.max_line_deviation_mm:.3f} mm"
            if motion.recommendation is not None and (
                motion.recommendation.changed_yaw
            ):
                summary += (
                    f" / 推荐 Yaw={motion.recommendation.recommended_yaw_deg:.2f}, "
                    f"J5={motion.recommendation.recommended_j5_deg:.2f}"
                )
            summaries.append(summary)
        self.motion_status_var.set("；".join(summaries))
        self._plot_prepared(prepared)

    def _plot_prepared(self, prepared: dict[str, object]) -> None:
        for axis in self.preview_axes:
            axis.clear()
            axis.set_facecolor("#ffffff")
            axis.grid(True, color="#d8dee5", linewidth=0.7)
        for axis, side in zip(self.preview_axes, ("left", "right")):
            motion = prepared.get(side)
            if motion is None:
                axis.set_title(f"{side} 未参与")
                continue
            trajectory = motion.trajectory
            for index, color in enumerate(JOINT_COLORS):
                axis.plot(
                    trajectory.time_s,
                    trajectory.q_deg[:, index],
                    color=color,
                    linewidth=1.7,
                    label=f"J{index + 1}",
                )
            axis.set_title(f"{side} · {motion.kind}")
            axis.set_xlabel("时间 (s)")
            axis.set_ylabel("关节角 (deg)")
            axis.legend(ncol=5, fontsize=8, loc="best")
        self.preview_figure.tight_layout()
        self.preview_canvas.draw_idle()

    def _draw_empty_preview(self) -> None:
        for side, axis in zip(("left", "right"), self.preview_axes):
            axis.set_facecolor("#ffffff")
            axis.set_title(f"{side} 轨迹预览")
            axis.set_xlabel("时间 (s)")
            axis.set_ylabel("关节角 (deg)")
            axis.grid(True, color="#d8dee5", linewidth=0.7)
        self.preview_figure.tight_layout()
        self.preview_canvas.draw_idle()

    def _load_model_target_from_motion(self) -> None:
        panel = (
            self.left_target
            if self.model_arm_var.get() == "left"
            else self.right_target
        )
        try:
            values = panel.values()
            if self._current_mode() == "joint":
                model = build_ideal_arm_model(values, check_limits=False)
                values = [
                    *model.tcp_position_mm,
                    model.pose.yaw_deg,
                    values[4],
                ]
            self.model_target_panel.set_values(values)
            self.notebook.select(self.model_tab)
            self.model_status_var.set("已从运动页载入目标")
        except Exception as exc:
            messagebox.showerror("无法载入目标", str(exc))

    @staticmethod
    def _default_model_seed() -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 30.0, 0.0], dtype=float)

    def _calculate_theoretical_model(self) -> None:
        try:
            requested_pose = np.asarray(
                self.model_target_panel.values(),
                dtype=float,
            )
            arm_name = self.model_arm_var.get()
            use_recommendation = self.model_use_recommendation_var.get()
            recommendation_config = self._recommend_config()
            dual = self.dual
        except Exception as exc:
            messagebox.showerror("模型参数错误", str(exc))
            return

        def task() -> dict[str, object]:
            q_seed = self._default_model_seed()
            if dual is not None:
                arm = dual.left if arm_name == "left" else dual.right
                if arm.connected and arm.sync_state() == OK:
                    q_seed = np.array(
                        [arm.joints[key]["current"] for key in JOINT_KEYS],
                        dtype=float,
                    )

            recommendation = None
            if use_recommendation:
                recommendation = recommend_feasible_yaw(
                    requested_pose[:3],
                    float(requested_pose[3]),
                    float(requested_pose[4]),
                    q_seed,
                    q_reference=q_seed,
                    config=recommendation_config,
                )
                if not recommendation.success:
                    raise RuntimeError(recommendation.message)
                ik_result = recommendation.ik_result
            else:
                ik_result = inverse_kinematics(
                    requested_pose[:3],
                    q_seed=q_seed,
                    target_yaw_deg=float(requested_pose[3]),
                    target_j5_deg=float(requested_pose[4]),
                    q_reference=q_seed,
                )
                if not ik_result.success:
                    raise RuntimeError(ik_result.message)

            model = build_ideal_arm_model(ik_result.q_deg)
            return {
                "requested_pose": requested_pose,
                "model": model,
                "recommendation": recommendation,
                "ik_result": ik_result,
            }

        def done(value: object) -> None:
            result = value  # type: ignore[assignment]
            self.model_requested_pose = np.asarray(
                result["requested_pose"],
                dtype=float,
            )
            self.model_target_mm = self.model_requested_pose[:3].copy()
            self.model_theoretical = result["model"]
            self.model_feedback = None
            self.model_recommendation = result["recommendation"]
            recommendation = result["recommendation"]
            if recommendation is not None and (
                recommendation.changed_yaw
            ):
                self.model_target_panel.set_recommended_yaw_j5(
                    recommendation.recommended_yaw_deg,
                    recommendation.recommended_j5_deg,
                )
                self.model_status_var.set(
                    "原姿态无解，模型使用推荐值 "
                    f"Yaw={recommendation.recommended_yaw_deg:.3f}°（右正左负）, "
                    f"J5={recommendation.recommended_j5_deg:.3f}°"
                )
            else:
                self.model_status_var.set("理论模型计算完成")
            self._update_model_view()

        self._run_async("计算理论模型", task, done)

    def _read_model_feedback(self) -> None:
        try:
            dual = self._require_dual()
            if self.model_theoretical is None or self.model_target_mm is None:
                raise RuntimeError("请先计算理论模型")
            arm_name = self.model_arm_var.get()
        except Exception as exc:
            messagebox.showerror("无法读取反馈", str(exc))
            return

        def task() -> IdealArmModel:
            arm = dual.left if arm_name == "left" else dual.right
            error = arm.sync_state()
            if error != OK:
                raise RuntimeError(err_text(error))
            q_feedback = [arm.joints[key]["current"] for key in JOINT_KEYS]
            return build_ideal_arm_model(q_feedback, check_limits=False)

        def done(value: object) -> None:
            self.model_feedback = value  # type: ignore[assignment]
            self.model_status_var.set("已用实机关节反馈计算 TCP 对比")
            self._update_model_view()

        self._run_async("读取反馈模型", task, done)

    @staticmethod
    def _format_xyz(values: np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):.3f}" for value in values) + "] mm"

    @staticmethod
    def _format_axis(values: np.ndarray) -> str:
        return "(" + ", ".join(f"{float(value):.3f}" for value in values) + ")"

    def _update_model_view(self) -> None:
        theoretical = self.model_theoretical
        target = self.model_target_mm
        if theoretical is None or target is None:
            self._draw_empty_model()
            return
        comparison = compare_tcp_positions(
            target,
            theoretical,
            self.model_feedback,
        )
        self.model_target_text_var.set(
            f"目标 TCP: {self._format_xyz(comparison.target_mm)}"
        )
        self.model_theory_text_var.set(
            f"理论 TCP: {self._format_xyz(comparison.theoretical_mm)}"
        )
        theory_delta = comparison.theoretical_mm - comparison.target_mm
        error_text = (
            f"目标→理论 |Δ|={comparison.target_to_theoretical_mm:.4f} mm, "
            f"Δ={self._format_xyz(theory_delta)}"
        )
        if comparison.feedback_mm is None:
            self.model_feedback_text_var.set("反馈 TCP: -")
        else:
            self.model_feedback_text_var.set(
                f"反馈 TCP: {self._format_xyz(comparison.feedback_mm)}"
            )
            feedback_delta = comparison.feedback_mm - comparison.target_mm
            error_text += (
                f"\n目标→反馈 |Δ|={comparison.target_to_feedback_mm:.4f} mm, "
                f"Δ={self._format_xyz(feedback_delta)}"
                f"\n理论→反馈={comparison.theoretical_to_feedback_mm:.4f} mm"
            )
        self.model_error_text_var.set(error_text)
        self.model_joint_text_var.set(
            "理论关节角: ["
            + ", ".join(f"{value:.3f}" for value in theoretical.q_deg)
            + "] deg"
        )
        self._update_model_frame_table(theoretical)
        self._draw_model_scene(theoretical, self.model_feedback, target)

    def _update_model_frame_table(self, model: IdealArmModel) -> None:
        for item in self.model_frame_table.get_children():
            self.model_frame_table.delete(item)
        for frame in model.frames:
            origin = frame.origin_mm
            self.model_frame_table.insert(
                "",
                "end",
                values=(
                    frame.name,
                    f"{origin[0]:.3f}",
                    f"{origin[1]:.3f}",
                    f"{origin[2]:.3f}",
                    self._format_axis(frame.x_axis),
                    self._format_axis(frame.y_axis),
                    self._format_axis(frame.z_axis),
                ),
            )

    def _draw_one_arm_model(
        self,
        model: IdealArmModel,
        *,
        color: str,
        label: str,
        linestyle: str,
        show_frames: bool,
    ) -> None:
        points = model.link_points_mm
        self.model_axis.plot(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            color=color,
            linewidth=3.0,
            linestyle=linestyle,
            marker="o",
            markersize=5,
            label=label,
        )
        if not show_frames:
            return
        axis_length = 42.0
        axis_colors = ("#d1495b", "#2a9d5b", "#2563eb")
        for index, frame in enumerate(model.frames):
            origin = frame.origin_mm
            for direction, axis_color in zip(frame.rotation.T, axis_colors):
                self.model_axis.quiver(
                    origin[0],
                    origin[1],
                    origin[2],
                    direction[0],
                    direction[1],
                    direction[2],
                    length=axis_length,
                    normalize=True,
                    color=axis_color,
                    linewidth=1.1,
                    alpha=0.72,
                )
            label_offset = np.array([5.0, 5.0, 5.0 + 6.0 * (index % 2)])
            label_position = origin + label_offset
            self.model_axis.text(
                label_position[0],
                label_position[1],
                label_position[2],
                frame.name,
                fontsize=8,
                color="#20262e",
            )

    def _draw_model_scene(
        self,
        theoretical: IdealArmModel,
        feedback: IdealArmModel | None,
        target_mm: np.ndarray,
    ) -> None:
        axis = self.model_axis
        axis.clear()
        axis.set_facecolor("#ffffff")
        self._draw_one_arm_model(
            theoretical,
            color="#0f766e",
            label="理论模型",
            linestyle="-",
            show_frames=True,
        )
        all_points = [theoretical.link_points_mm, target_mm.reshape(1, 3)]
        if feedback is not None:
            self._draw_one_arm_model(
                feedback,
                color="#d1495b",
                label="反馈模型",
                linestyle="--",
                show_frames=False,
            )
            all_points.append(feedback.link_points_mm)
        axis.scatter(
            [target_mm[0]],
            [target_mm[1]],
            [target_mm[2]],
            marker="x",
            s=90,
            linewidths=2.5,
            color="#7c3aed",
            label="目标 TCP",
        )
        points = np.vstack(all_points)
        center = (np.min(points, axis=0) + np.max(points, axis=0)) / 2.0
        radius = max(120.0, float(np.max(np.ptp(points, axis=0))) * 0.58)
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)
        axis.set_box_aspect((1.0, 1.0, 1.0))
        axis.set_xlabel("Base X / 向下 (mm)")
        axis.set_ylabel("Base Y / 向前 (mm)")
        axis.set_zlabel("Base Z / 向右 (mm)")
        axis.set_title(
            f"{self.model_arm_var.get()} arm · 肩部 Base 理想连杆模型"
        )
        axis.grid(True, color="#d8dee5", linewidth=0.6)
        axis.view_init(elev=23, azim=-55)
        axis.legend(loc="upper left")
        self.model_figure.tight_layout()
        self.model_canvas.draw_idle()

    def _draw_empty_model(self) -> None:
        axis = self.model_axis
        axis.clear()
        axis.set_facecolor("#ffffff")
        axis.scatter([0.0], [0.0], [0.0], color="#20262e", s=32)
        axis.text(0.0, 0.0, 0.0, " Base", fontsize=9)
        axis.set_xlim(-350.0, 350.0)
        axis.set_ylim(-350.0, 350.0)
        axis.set_zlim(-350.0, 350.0)
        axis.set_box_aspect((1.0, 1.0, 1.0))
        axis.set_xlabel("Base X / 向下 (mm)")
        axis.set_ylabel("Base Y / 向前 (mm)")
        axis.set_zlabel("Base Z / 向右 (mm)")
        axis.set_title("肩部 Base 理想连杆模型")
        axis.grid(True, color="#d8dee5", linewidth=0.6)
        axis.view_init(elev=23, azim=-55)
        self.model_figure.tight_layout()
        self.model_canvas.draw_idle()

    def _export_model_csv(self) -> None:
        theoretical = self.model_theoretical
        target = self.model_target_mm
        if theoretical is None or target is None:
            messagebox.showinfo("没有模型", "请先计算理论模型。")
            return
        path = filedialog.asksaveasfilename(
            title="导出理想模型与坐标系",
            defaultextension=".csv",
            initialfile=f"sanpo_ideal_model_{self.model_arm_var.get()}.csv",
            filetypes=(("CSV 表格", "*.csv"),),
        )
        if not path:
            return
        destination = export_ideal_model_csv(
            path,
            target_mm=target,
            theoretical_model=theoretical,
            feedback_model=self.model_feedback,
        )
        self.model_status_var.set(f"已导出 {destination.name}")

    def _export_model_plot(self) -> None:
        if self.model_theoretical is None:
            messagebox.showinfo("没有模型", "请先计算理论模型。")
            return
        path = filedialog.asksaveasfilename(
            title="导出理想模型三维图",
            defaultextension=".png",
            initialfile=f"sanpo_ideal_model_{self.model_arm_var.get()}.png",
            filetypes=(("PNG 图片", "*.png"),),
        )
        if path:
            self.model_figure.savefig(Path(path), dpi=200, bbox_inches="tight")
            self.model_status_var.set(f"已导出 {Path(path).name}")

    def _recommend_targets(self) -> None:
        if self._current_mode() == "joint":
            messagebox.showinfo("姿态推荐", "姿态推荐用于 TCP 目标，请先选择 MoveCart 或 MoveLine。")
            return
        try:
            dual = self._require_dual()
            scope = self._current_scope()
            targets = {
                "left": self.left_target.values(),
                "right": self.right_target.values(),
            }
            config = self._recommend_config()
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        names = ("left", "right") if scope == "dual" else (scope,)

        def task() -> dict[str, object]:
            results: dict[str, object] = {}
            with_threads: list[threading.Thread] = []
            lock = threading.Lock()

            def solve(name: str) -> None:
                arm = dual.left if name == "left" else dual.right
                error, result = arm.preview_ik_recommendation(targets[name], config=config)
                with lock:
                    results[name] = (error, result)

            for name in names:
                thread = threading.Thread(target=solve, args=(name,))
                thread.start()
                with_threads.append(thread)
            for thread in with_threads:
                thread.join()
            return results

        def done(result_value: object) -> None:
            results = result_value  # type: ignore[assignment]
            messages: list[str] = []
            changed = False
            for name, (error, result) in results.items():
                if error != OK or result is None:
                    messages.append(f"{name}: {err_text(error)}")
                    continue
                messages.append(
                    f"{name}: Yaw {result.requested_yaw_deg:.2f} -> "
                    f"{result.recommended_yaw_deg:.2f}, J5 {result.requested_j5_deg:.2f} -> "
                    f"{result.recommended_j5_deg:.2f}"
                )
                changed = changed or result.changed_yaw
            if not changed:
                messagebox.showinfo("姿态推荐", "\n".join(messages) + "\n\n当前目标已有可行解。")
                return
            if messagebox.askyesno(
                "采用推荐姿态",
                "\n".join(messages) + "\n\n是否写回目标输入框？",
            ):
                for name, (_error, result) in results.items():
                    if result is None:
                        continue
                    panel = self.left_target if name == "left" else self.right_target
                    panel.set_recommended_yaw_j5(
                        result.recommended_yaw_deg,
                        result.recommended_j5_deg,
                    )

        self._run_async("搜索可行姿态", task, done)

    def _execute_motion(self) -> None:
        try:
            dual = self._require_dual()
            scope = self._current_scope()
            left_target = self.left_target.values()
            right_target = self.right_target.values()
            parameters = self._motion_parameters()
            synchronize_finish = self.synchronize_finish_var.get()
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        if not messagebox.askyesno(
            "运动确认",
            "确认机械臂周围无人员和障碍物，并已准备硬件急停？",
        ):
            return

        def task() -> object:
            if scope == "dual":
                return dual.move_both(
                    left_target,
                    right_target,
                    synchronize_finish=synchronize_finish,
                    blocking=True,
                    **parameters,
                )
            arm = dual.left if scope == "left" else dual.right
            target = left_target if scope == "left" else right_target
            mode = parameters["mode"]
            common = dict(
                speed=parameters["speed"],
                accel=parameters["accel"],
                blocking=True,
                sample_period_s=parameters["sample_period_s"],
            )
            if mode == "joint":
                return arm.MoveJ(target, **common)
            if mode == "cartesian":
                if parameters["allow_recommendation"]:
                    return arm.MoveCartRecommended(
                        target,
                        recommendation_config=parameters["recommendation_config"],
                        **common,
                    )
                return arm.MoveCart(target, **common)
            return arm.MoveLine(
                target,
                total_time_s=parameters["total_time_s"],
                position_tolerance_mm=parameters["position_tolerance_mm"],
                max_branch_jump_deg=parameters["max_branch_jump_deg"],
                **common,
            )

        def done(result: object) -> None:
            if hasattr(result, "success"):
                success = result.success
                text = (
                    "双臂运动完成"
                    if success
                    else f"左臂: {err_text(result.left_error)}; 右臂: {err_text(result.right_error)}"
                )
            else:
                success = int(result) == OK
                text = "运动完成" if success else err_text(int(result))
            self.motion_status_var.set(text)
            if not success:
                messagebox.showerror("运动失败", text)
            self._load_current_targets()

        self._run_async("执行运动", task, done)

    def _stop_motion(self) -> None:
        if self.dual is None:
            return
        self.dual.stop(disable=True)
        self.motion_status_var.set("已停止并发送失能命令")

    def _start_monitor(self) -> None:
        try:
            dual = self._require_dual()
            period = float(self.monitor_period_var.get())
            if period <= 0.0:
                raise ValueError("采样周期必须大于 0")
        except Exception as exc:
            messagebox.showerror("监控参数错误", str(exc))
            return
        if self.recorder is not None:
            self.recorder.stop()
        self.recorder = TelemetryRecorder(
            {"left": dual.left, "right": dual.right},
            sample_period_s=period,
            max_samples_per_joint=TELEMETRY_DEFAULTS.max_samples_per_joint,
        )
        self.recorder.start()
        self.monitor_status_var.set(f"记录中 · 目标周期 {period:.3f} s")

    def _stop_monitor(self) -> None:
        if self.recorder is not None:
            self.recorder.stop()
        self.monitor_status_var.set("监控已停止")

    def _start_gripper_monitor(self) -> None:
        try:
            grippers = self._require_grippers()
            period = float(self.gripper_sample_period_var.get())
            if period <= 0.0:
                raise ValueError("采样周期必须大于 0")
        except Exception as exc:
            messagebox.showerror("夹爪监控参数错误", str(exc))
            return
        if self.gripper_recorder is not None:
            self.gripper_recorder.stop()
        self.gripper_recorder = GripperTelemetryRecorder(
            grippers,
            sample_period_s=period,
        )
        self.gripper_recorder.start()
        self.gripper_status_var.set(f"夹爪记录中 · {period:.3f} s")

    def _stop_gripper_monitor(self) -> None:
        if self.gripper_recorder is not None:
            self.gripper_recorder.stop()
        if self.grippers is not None:
            self.gripper_status_var.set("夹爪监控已停止")

    def _clear_gripper_monitor(self) -> None:
        if self.gripper_recorder is not None:
            self.gripper_recorder.clear()
        self._draw_empty_gripper_plot()
        self.gripper_status_var.set("夹爪记录已清空")

    def _plot_gripper_series(self) -> None:
        recorder = self.gripper_recorder
        if recorder is None:
            return
        colors = {"left": "#0f766e", "right": "#d1495b"}
        self.gripper_opening_axes.clear()
        self.gripper_effort_axes.clear()
        for side, label in (("left", "左夹爪"), ("right", "右夹爪")):
            samples = recorder.snapshot(side)[-1000:]
            if not samples:
                continue
            elapsed = [sample.elapsed_s for sample in samples]
            opening = [
                float("nan")
                if sample.opening_fraction is None
                else sample.opening_fraction * 100.0
                for sample in samples
            ]
            torque = [sample.torque_nm for sample in samples]
            self.gripper_opening_axes.plot(
                elapsed,
                opening,
                color=colors[side],
                label=label,
                linewidth=1.4,
            )
            self.gripper_effort_axes.plot(
                elapsed,
                torque,
                color=colors[side],
                label=label,
                linewidth=1.4,
            )
        for axes in (self.gripper_opening_axes, self.gripper_effort_axes):
            axes.grid(True, color="#d8dee4", linewidth=0.7)
            if axes.lines:
                axes.legend(loc="upper right")
        self.gripper_opening_axes.set_ylabel("开度 (%)")
        self.gripper_opening_axes.set_ylim(-5, 105)
        self.gripper_effort_axes.set_ylabel("力矩 (Nm)")
        self.gripper_effort_axes.set_xlabel("时间 (s)")
        self.gripper_canvas.draw_idle()

    def _gripper_monitor_tick(self) -> None:
        recorder = self.gripper_recorder
        if recorder is not None:
            latest = recorder.latest()
            states: dict[str, GripperState | None] = {}
            for side, sample in latest.items():
                states[side] = GripperState(
                    status_code=sample.status_code,
                    position_rad=sample.position_rad,
                    velocity_rad_s=sample.velocity_rad_s,
                    torque_nm=sample.torque_nm,
                    mos_temperature_c=sample.mos_temperature_c,
                    rotor_temperature_c=sample.rotor_temperature_c,
                    opening_fraction=sample.opening_fraction,
                )
            self._update_gripper_states(states)
            self._plot_gripper_series()
            if recorder.last_error:
                self.gripper_status_var.set(
                    f"夹爪监控通信错误: {recorder.last_error}"
                )
        self.after(300, self._gripper_monitor_tick)

    def _clear_monitor(self) -> None:
        if self.recorder is not None:
            self.recorder.clear()
        for arm in ("left", "right"):
            for joint in JOINT_KEYS:
                self.feedback_table.item(
                    f"{arm}:{joint}",
                    values=(arm, joint, "-", "-", "-", "-", "-", "-", 0),
                )
        self._draw_empty_monitor()

    @staticmethod
    def _format_optional(value: object, digits: int = 3) -> str:
        return "-" if value is None else f"{float(value):.{digits}f}"

    def _monitor_tick(self) -> None:
        recorder = self.recorder
        if recorder is not None:
            latest = recorder.latest()
            peaks = {(item.arm, item.joint): item for item in recorder.peak_summaries()}
            for key, sample in latest.items():
                peak = peaks.get(key)
                angle_range = "-"
                if peak and peak.angle_min_deg is not None and peak.angle_max_deg is not None:
                    angle_range = f"{peak.angle_min_deg:.2f} .. {peak.angle_max_deg:.2f}"
                self.feedback_table.item(
                    f"{key[0]}:{key[1]}",
                    values=(
                        key[0],
                        key[1],
                        self._format_optional(sample.angle_deg),
                        self._format_optional(sample.speed_rpm),
                        self._format_optional(sample.current_a),
                        angle_range,
                        self._format_optional(None if peak is None else peak.max_abs_speed_rpm),
                        self._format_optional(None if peak is None else peak.max_abs_current_a),
                        0 if peak is None else peak.sample_count,
                    ),
                )
            self._plot_monitor_series()
        self.after(300, self._monitor_tick)

    def _plot_monitor_series(self) -> None:
        if self.recorder is None:
            return
        samples = self.recorder.snapshot(
            self.monitor_arm_var.get(),
            self.monitor_joint_var.get(),
        )[-800:]
        if not samples:
            return
        times = [sample.elapsed_s for sample in samples]
        series = (
            ([sample.angle_deg for sample in samples], "角度 (deg)", "#0f766e"),
            ([sample.speed_rpm for sample in samples], "速度 (rpm)", "#d1495b"),
            ([sample.current_a for sample in samples], "Q 轴电流 (A)", "#2563eb"),
        )
        for axis, (values, ylabel, color) in zip(self.monitor_axes, series):
            axis.clear()
            axis.set_facecolor("#ffffff")
            valid = [
                (time_value, value)
                for time_value, value in zip(times, values)
                if value is not None
            ]
            if valid:
                axis.plot(
                    [item[0] for item in valid],
                    [item[1] for item in valid],
                    color=color,
                    linewidth=1.5,
                )
            axis.set_ylabel(ylabel)
            axis.grid(True, color="#d8dee5", linewidth=0.7)
        self.monitor_axes[-1].set_xlabel("记录时间 (s)")
        self.monitor_figure.suptitle(
            f"{self.monitor_arm_var.get()} / {self.monitor_joint_var.get().upper()}"
        )
        self.monitor_figure.tight_layout()
        self.monitor_canvas.draw_idle()

    def _draw_empty_monitor(self) -> None:
        for axis, ylabel in zip(
            self.monitor_axes,
            ("角度 (deg)", "速度 (rpm)", "Q 轴电流 (A)"),
        ):
            axis.clear()
            axis.set_facecolor("#ffffff")
            axis.set_ylabel(ylabel)
            axis.grid(True, color="#d8dee5", linewidth=0.7)
        self.monitor_axes[-1].set_xlabel("记录时间 (s)")
        self.monitor_figure.tight_layout()
        self.monitor_canvas.draw_idle()

    def _default_export_name(self, suffix: str) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"sanpo_feedback_{stamp}.{suffix}"

    def _export_csv(self) -> None:
        if self.recorder is None or not self.recorder.snapshot():
            messagebox.showinfo("没有数据", "请先开始记录反馈。")
            return
        path = filedialog.asksaveasfilename(
            title="导出反馈数据",
            defaultextension=".csv",
            initialfile=self._default_export_name("csv"),
            filetypes=(("CSV 表格", "*.csv"),),
        )
        if path:
            destination = self.recorder.export_csv(path)
            peak_destination = destination.with_name(
                f"{destination.stem}_peaks{destination.suffix}"
            )
            self.recorder.export_peak_csv(peak_destination)
            self.monitor_status_var.set(
                f"已导出 {destination.name} 和 {peak_destination.name}"
            )

    def _export_plot(self) -> None:
        path = filedialog.asksaveasfilename(
            title="导出反馈曲线",
            defaultextension=".png",
            initialfile=self._default_export_name("png"),
            filetypes=(("PNG 图片", "*.png"),),
        )
        if path:
            self.monitor_figure.savefig(Path(path), dpi=180, bbox_inches="tight")
            self.monitor_status_var.set(f"已导出 {Path(path).name}")

    def _export_gripper_csv(self) -> None:
        recorder = self.gripper_recorder
        if recorder is None or not recorder.snapshot():
            messagebox.showinfo("没有数据", "请先开始记录夹爪反馈。")
            return
        path = filedialog.asksaveasfilename(
            title="导出夹爪反馈数据",
            defaultextension=".csv",
            initialfile=self._default_export_name("csv").replace(
                "feedback",
                "gripper_feedback",
            ),
            filetypes=(("CSV 表格", "*.csv"),),
        )
        if path:
            destination = recorder.export_csv(path)
            peak_destination = destination.with_name(
                f"{destination.stem}_peaks{destination.suffix}"
            )
            recorder.export_peak_csv(peak_destination)
            self.gripper_status_var.set(
                f"已导出 {destination.name} 和 {peak_destination.name}"
            )

    def _export_gripper_plot(self) -> None:
        recorder = self.gripper_recorder
        if recorder is None or not recorder.snapshot():
            messagebox.showinfo("没有数据", "请先开始记录夹爪反馈。")
            return
        path = filedialog.asksaveasfilename(
            title="导出夹爪反馈曲线",
            defaultextension=".png",
            initialfile=self._default_export_name("png").replace(
                "feedback",
                "gripper_feedback",
            ),
            filetypes=(("PNG 图片", "*.png"),),
        )
        if path:
            self.gripper_figure.savefig(Path(path), dpi=180, bbox_inches="tight")
            self.gripper_status_var.set(f"已导出 {Path(path).name}")

    def _on_close(self) -> None:
        if self.recorder is not None:
            self.recorder.stop()
        if self.gripper_recorder is not None:
            self.gripper_recorder.stop()
        if self.system is not None:
            self.system.arms.stop(disable=True)
            self.system.close()
        self.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SANPO dual-arm graphical console")
    parser.add_argument("--simulate", action="store_true", help="use in-memory arm backends")
    parser.add_argument("--left-port")
    parser.add_argument("--right-port")
    parser.add_argument(
        "--usb-mode",
        choices=("advanced", "standard"),
        default="standard",
    )
    parser.add_argument("--left-channel", type=int, default=1)
    parser.add_argument("--left-gripper-channel", type=int, default=2)
    parser.add_argument("--right-channel", type=int, default=1)
    parser.add_argument("--right-gripper-channel", type=int, default=2)
    parser.add_argument(
        "--left-gripper-enabled",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--right-gripper-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--left-gripper-id", type=int, default=1)
    parser.add_argument("--right-gripper-id", type=int, default=1)
    parser.add_argument("--left-gripper-master-id", type=int, default=0)
    parser.add_argument("--right-gripper-master-id", type=int, default=0)
    parser.add_argument("--gripper-closed-position", type=float, default=0.0)
    parser.add_argument("--gripper-open-position", type=float, default=2.7)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    app = ArmDashboard(args)
    app.mainloop()


if __name__ == "__main__":
    main()
