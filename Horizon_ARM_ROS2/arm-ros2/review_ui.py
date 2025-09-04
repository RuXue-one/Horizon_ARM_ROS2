#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简易审核与真机控制 UI（Ubuntu）：
- 默认本地直控。
- 实时显示：原始弧度、原始角度(度)、目标角(度)、实测角度(度)，并做越界高亮。
- 主要操作：下发到位、停止/断电、回零。

运行示例：
  本地直控：python review_ui.py --mapping mapping.yaml --com-port /dev/ttyACM0
"""

import math
import threading
from typing import List, Tuple
import argparse
import yaml
import signal
import atexit

import tkinter as tk
from tkinter import ttk, messagebox

from pathlib import Path
import os
import sys
import platform
import time
import math
import queue
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# 默认使用已安装的 wheel 包；仅当设置 USE_LOCAL_SDK=1 时才优先加载本地拷贝
if os.environ.get('USE_LOCAL_SDK', '0') == '1':
    # 1) 显式加入 ~/Horizon_ARM_ROS2/Control_SDK-ros
    SDK_HOME_DIR = Path.home() / 'Horizon_ARM_ROS2' / 'Control_SDK-ros'
    if SDK_HOME_DIR.exists() and str(SDK_HOME_DIR) not in sys.path:
        sys.path.insert(0, str(SDK_HOME_DIR))
    # 2) 其次加入 <workspace_root>/Control_SDK-ros
    SDK_ROS_DIR = PROJECT_ROOT / 'Control_SDK-ros'
    if SDK_ROS_DIR.exists() and str(SDK_ROS_DIR) not in sys.path:
        sys.path.insert(0, str(SDK_ROS_DIR))

# 现在再导入封装（保证路径已加入）
SimpleArmSDK = None
try:
    from simple_sdk import SimpleArmSDK as _SimpleArmSDK  # type: ignore
    SimpleArmSDK = _SimpleArmSDK
except Exception:
    SimpleArmSDK = None

ZDTMotorControllerModular = None
_SDK_IMPORT_ERR = None
def _try_import_sdk():
    global ZDTMotorControllerModular, _SDK_IMPORT_ERR
    try:
        from Control_Core.motor_controller_modular import ZDTMotorControllerModular as Mod  # type: ignore
        # 不强制能力检查，若缺少方法在后续运行时兜底注入
        ZDTMotorControllerModular = Mod
        _SDK_IMPORT_ERR = None
        return True
    except Exception as e:
        _SDK_IMPORT_ERR = e
        ZDTMotorControllerModular = None
        return False

# 仅使用 ~/Horizon_ARM_ROS2/Control_SDK-ros 与项目内 Control_SDK-ros
_try_import_sdk()

# 兜底：运行时对 SLCANInterface 注入兼容方法，防止旧版SDK缺失
try:
    if ZDTMotorControllerModular is not None:
        from Control_Core import can_interface as _cif  # type: ignore
        if hasattr(_cif, 'SLCANInterface'):
            # 注入多包发送实现（若缺失）
            if not hasattr(_cif.SLCANInterface, '_send_multi_packet_command'):
                def _slcan_send_multi(self, motor_id: int, command_data: list, timeout: float):
                    base_frame_id = motor_id << 8
                    if not command_data:
                        return self.receive_message(base_frame_id, timeout)
                    function_code = command_data[0]
                    packets = []
                    packets.append(command_data[:8])
                    remaining = command_data[8:]
                    while remaining:
                        packets.append([function_code] + remaining[:7])
                        remaining = remaining[7:]
                    for i, packet in enumerate(packets):
                        frame_id = base_frame_id + i
                        self.send_message(frame_id, packet)
                        time.sleep(0.05)
                    return self.receive_message(base_frame_id, timeout)
                setattr(_cif.SLCANInterface, '_send_multi_packet_command', _slcan_send_multi)

            # 注入 send_command_and_receive_response（若缺失）
            if not hasattr(_cif.SLCANInterface, 'send_command_and_receive_response'):
                def _slcan_send_and_recv(self, motor_id: int, command_data: list, timeout: float = 1.0):
                    base_frame_id = motor_id << 8
                    if len(command_data) <= 8:
                        self.send_message(base_frame_id, command_data)
                        return self.receive_message(base_frame_id, timeout)
                    else:
                        return self._send_multi_packet_command(motor_id, command_data, timeout)
                setattr(_cif.SLCANInterface, 'send_command_and_receive_response', _slcan_send_and_recv)
except Exception:
    pass


def rad_to_deg(vals: List[float]) -> List[float]:
    return [v * 180.0 / math.pi for v in vals]


class Mapper:
    def __init__(self, cfg: dict):
        self.sign = cfg['sign']
        self.gear = cfg.get('gear_ratio', [1,1,1,1,1,1])
        self.zero = cfg['zero_offset_deg']
        self.lim  = cfg['pos_limit_deg']
        self.names = cfg.get('joints', [f'J{i+1}' for i in range(6)])
        self.send_space = cfg.get('send_space', 'motor')  # 'motor' or 'output'
        self.apply_sign_in_ui = bool(cfg.get('apply_sign_in_ui', True))
        self.show_output_measured = bool(cfg.get('show_output_measured', False))

    def compute_targets(self, deg: List[float]) -> Tuple[List[float], List[float]]:
        """返回 (output_target_deg, motor_target_deg)。两者都已按关节限位裁剪并应用零位。"""
        out: List[float] = []
        mot: List[float] = []
        for i, v in enumerate(deg):
            mn, mx = self.lim[i]
            v_clamped = min(max(v, mn), mx)
            # 输出端角不叠加零位（零位仅用于电机映射）
            output_deg = v_clamped
            sign = (self.sign[i] if self.apply_sign_in_ui else 1)
            gear = (self.gear[i] if self.gear[i] else 1)
            motor_deg = sign * output_deg * gear + self.zero[i]
            out.append(output_deg)
            mot.append(motor_deg)
        return out, mot

    def map_deg(self, deg: List[float]) -> List[float]:
        # 保持兼容：根据 send_space 返回要下发的目标序列
        out, mot = self.compute_targets(deg)
        return out if self.send_space == 'output' else mot

    def clamp_only(self, deg: List[float]) -> List[float]:
        """仅返回按关节限位裁剪后的关节角(°)，用于 UI 警示显示。"""
        out = []
        for i, v in enumerate(deg):
            mn, mx = self.lim[i]
            out.append(min(max(v, mn), mx))
        return out


class ReviewUI:
    def __init__(self, args):
        with open(args.mapping, 'r', encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        self.mapper = Mapper(self.cfg)
        # 仅本地直控
        self.com_port = args.com_port or self.cfg.get('com_port', '/dev/ttyACM0')
        self.baudrate = self.cfg.get('baudrate', 500000)
        self.max_speed = self.cfg.get('max_speed_rpm', 100)
        self.acc = self.cfg.get('acceleration_rpm_s', 150)
        self.dec = self.cfg.get('deceleration_rpm_s', 150)
        self.stream_min_delta_deg = float(self.cfg.get('stream_min_delta_deg', 0.3))
        self.stream_use_direct = bool(self.cfg.get('stream_use_direct', True))
        self.stream_speed_rpm = int(self.cfg.get('stream_speed_rpm', 150))
        self.servo_rate_hz = float(self.cfg.get('servo_rate_hz', 50.0))
        # 抖动抑制/收敛与终结（防止回读滞后导致来回调）
        self.stream_settle_tol_deg = float(self.cfg.get('stream_settle_tol_deg', 0.5))
        self.stream_settle_hold_iters = int(self.cfg.get('stream_settle_hold_iters', 5))
        self.stream_finalize_with_abs = bool(self.cfg.get('stream_finalize_with_abs', True))
        # 读取/下发节流与重试参数（避免串口帧黏连与瞬时超时）
        self.sdk_timeout_s = float(self.cfg.get('sdk_timeout_s', 1.0))
        self.read_retries = int(self.cfg.get('read_retries', 0))
        self.read_gap_ms = int(self.cfg.get('read_gap_ms', 20))
        self.connect_gap_ms = int(self.cfg.get('connect_gap_ms', 50))
        self.send_retries = int(self.cfg.get('send_retries', 0))
        self.send_gap_ms = int(self.cfg.get('send_gap_ms', 20))
        # 每关节角度套圈范围（度），与主程序一致默认全为180，可在mapping.yaml配置wrap_deg与wrap_epsilon_deg
        self.wrap_deg = self.cfg.get('wrap_deg', [180, 180, 180, 180, 180, 180])
        self.wrap_epsilon = float(self.cfg.get('wrap_epsilon_deg', 0.5))
        self.stop_flag = False
        self.busy = False
        self.in_stream_tick = False  # 实时下发中的软避让标志
        self._settle_hits = 0
        self._finalized_target = None  # 记录已终结过的目标，避免重复绝对下发
        # 控制态（用于实时计算最小角差的基线，与UI显示解耦）
        self.ctrl_state_out = [0.0]*6
        # 实时跟随默认关闭
        self.stream_enabled = False
        self.last_deg = [0.0]*6
        self.last_mapped = [0.0]*6
        self.last_motor_pos = [0.0]*6            # 最近一次读取的电机角(°)
        self.last_output_measured = [0.0]*6      # 最近一次读取并换算的输出端角(°)
        self.ctrls = []
        self.bc = None  # 广播控制器（motor_id=0）
        self._sdk = None  # SimpleArmSDK 实例
        self.io_lock = threading.Lock()
        # 回零
        self.home_mode = self.cfg.get('home_mode', 'zeros')
        self.home_output = [0.0] * 6  # 输出端角回零目标(°), len=6 - 固定为各轴0度

        self.root = tk.Tk()
        self.root.title("ROS 轨迹测试")
        
        # 设置安全退出处理（防止异常退出导致机械臂失能砸桌）
        self._setup_safe_exit()
        # —— 基础样式与主题（尽量贴近主程序风格）——
        try:
            style = ttk.Style(self.root)
            # Windows 优先使用更现代的 vista 主题，否则退回到 clam
            theme = 'vista' if 'vista' in style.theme_names() else ('xpnative' if 'xpnative' in style.theme_names() else 'clam')
            style.theme_use(theme)
            # 字体设置：Windows 使用微软雅黑，字号整体放大；并设置 tk scaling
            default_font_family = 'Microsoft YaHei UI' if platform.system().lower() == 'windows' else 'Helvetica'
            self.root.option_add('*Font', (default_font_family, 12))
            try:
                # 轻度放大整体缩放，提高可读性且不发糊
                self.root.call('tk', 'scaling', 1.15)
            except Exception:
                pass
            # 背景统一为白色
            self.root.configure(bg='white')
            style.configure('.', background='white')
            # 统一控件留白
            style.configure('TFrame', padding=6)
            style.configure('TFrame', background='white')
            style.configure('TLabel', padding=(2, 1), background='white', font=(default_font_family, 12))
            style.configure('Header.TLabel', font=(default_font_family, 12, 'bold'))
            style.configure('Title.TLabel', font=(default_font_family, 13, 'bold'))
            style.configure('TButton', padding=(12, 6), background='white', font=(default_font_family, 12))
            # 轻量强调而非大色块，贴近主程序简洁风
            style.configure('Primary.TButton')
            style.configure('Danger.TButton')
            style.configure('Info.TLabel', foreground='#2563eb', background='white')
        except Exception:
            pass
        self._build_ui()

        self.th_status = threading.Thread(target=self._status_loop, daemon=True)
        self.th_status.start()
        # 独立实时伺服循环线程
        try:
            self.th_stream = threading.Thread(target=self._stream_loop, daemon=True)
            self.th_stream.start()
        except Exception:
            pass
        # 恢复 UI 的“实时跟随”按钮逻辑：通过 ROS 话题发布到驱动
        # 新增：订阅 ROS2 的 DisplayTrajectory，更新 UI 目标但不自动下发
        try:
            import rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from moveit_msgs.msg import DisplayTrajectory  # type: ignore
            from trajectory_msgs.msg import JointTrajectory  # type: ignore
            from std_msgs.msg import Float64MultiArray  # type: ignore
            from sensor_msgs.msg import JointState  # type: ignore
            rclpy.init(args=None)
            self._ros_node = rclpy.create_node('review_ui_listener')
            # 后备实时话题发布者
            try:
                self._servo_pub = self._ros_node.create_publisher(Float64MultiArray, '/horizon_arm/servo_target', 10)
            except Exception:
                self._servo_pub = None
            self._jt_queue = queue.Queue(maxsize=2)
            self._js_sub = None
            # 订阅 /horizon_arm/servo_target：统一经由UI转发到SDK，避免外部进程直连串口
            def _on_servo(msg: Float64MultiArray):
                try:
                    vals = list(msg.data)
                    if len(vals) != 6:
                        return
                    
                    # 更新UI显示
                    self._update_ui_from_servo_data(vals)
                    
                    # 仅在"UI已连接硬件"时处理硬件控制，绝不在此自动连接串口（单主控纪律）
                    if not (self._sdk or self.ctrls):
                        return
                    if self.stream_enabled:
                        # 实时模式：走现有步进逻辑（包含最小角差/限位/收敛）
                        self._send_stream_step(vals)
                    else:
                        # 非实时：一次性绝对到位（输出端角）
                        with self.io_lock:
                            if self._sdk:
                                self._sdk.goto_output_abs(vals, max_speed_rpm=self.max_speed, acc=self.acc, dec=self.dec)
                            elif self.ctrls:
                                # 旧路径：从当前电机角+delta(由输出端差换算)得到绝对目标
                                targets_abs_motor = []
                                for i in range(6):
                                    direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                                    ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                                    delta_out = float(vals[i]) - float(self.last_output_measured[i])
                                    targets_abs_motor.append(float(self.last_motor_pos[i]) + direction * delta_out * ratio)
                                for c, target in zip(self.ctrls, targets_abs_motor):
                                    try:
                                        c.control_actions.move_to_position_trapezoid(
                                            position=float(target),
                                            max_speed=int(self.max_speed),
                                            acceleration=int(self.acc),
                                            deceleration=int(self.dec),
                                            is_absolute=True,
                                            multi_sync=False,
                                        )
                                    except Exception:
                                        pass
                except Exception:
                    pass
            def cb(msg):
                try:
                    if msg.trajectory:
                        self._jt_queue.put_nowait(msg.trajectory[0].joint_trajectory)
                except Exception:
                    pass
            self._ros_sub1 = self._ros_node.create_subscription(DisplayTrajectory, '/display_planned_path', cb, 10)
            self._ros_sub2 = self._ros_node.create_subscription(DisplayTrajectory, '/move_group/display_planned_path', cb, 10)
            # 订阅统一实时入口（外部脚本/RViz均可通过该话题与UI交互）
            try:
                self._servo_sub = self._ros_node.create_subscription(Float64MultiArray, '/horizon_arm/servo_target', _on_servo, 10)
            except Exception:
                self._servo_sub = None
            def _spin_bg():
                try:
                    while not self.stop_flag:
                        rclpy.spin_once(self._ros_node, timeout_sec=0.1)
                        try:
                            jt = self._jt_queue.get_nowait()
                        except Exception:
                            jt = None
                        if jt is None:
                            continue
                        try:
                            names = list(jt.joint_names)
                            if not jt.points:
                                continue
                            mapping = {n:i for i,n in enumerate(names)}
                            # 缓存整条轨迹（按点顺序），若实时跟随打开则逐点下发；否则仅更新末帧到UI
                            seq = []
                            for pt in jt.points:
                                raw_rad = []
                                for n in self.mapper.names:
                                    idx = mapping.get(n, -1)
                                    if idx >= 0 and idx < len(pt.positions):
                                        raw_rad.append(float(pt.positions[idx]))
                                    else:
                                        raw_rad.append(0.0)
                                raw_deg = rad_to_deg(raw_rad)
                                seq.append((raw_rad, raw_deg))
                            # 更新 UI 以末帧为准
                            raw_rad, raw_deg = seq[-1]
                            output_target, motor_target = self.mapper.compute_targets(raw_deg)
                            self.last_deg = raw_deg
                            self.last_mapped = (output_target if self.mapper.send_space=='output' else motor_target)
                            self.last_target_output = output_target
                            self.last_target_motor = motor_target
                            self._update_table(raw_rad, raw_deg, output_target)
                            # 实时下发由独立伺服线程处理；此处仅缓存目标并更新状态
                            if not self.stream_enabled:
                                self.status.set("已接收规划(缓存)，请点击‘下发到位’执行 或 打开实时跟随")
                        except Exception:
                            pass
                except Exception:
                    pass
            self._ros_spin_thread = threading.Thread(target=_spin_bg, daemon=True)
            self._ros_spin_thread.start()

            # 实时模式下仍通过硬件串口回读更新UI状态
            self._js_sub = None
        except Exception:
            self._ros_node = None

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=8)
        frm.grid(row=0, column=0, sticky='nsew')
        # 拉伸设置，保持横向长条
        try:
            self.root.grid_columnconfigure(0, weight=1)
            self.root.grid_rowconfigure(0, weight=1)
            # 让整个表格框架可以扩展
            for c in range(9):  # 更新为9列
                frm.grid_columnconfigure(c, weight=1)
        except Exception:
            pass

        # 连接区域
        top = ttk.Frame(frm)
        top.grid(row=0, column=0, columnspan=9, sticky='ew')
        lbl = "本地直控"
        ttk.Label(top, text=lbl).grid(row=0, column=0, padx=4)
        ttk.Label(top, text="串口/通道:").grid(row=0, column=1)
        self.com_var = tk.StringVar(value=self.com_port)
        ttk.Entry(top, textvariable=self.com_var, width=10).grid(row=0, column=2)
        ttk.Button(top, text="连接", command=self._connect_ctrls).grid(row=0, column=3, padx=4)
        ttk.Button(top, text="断开", command=lambda: self._disconnect_ctrls(disable=False)).grid(row=0, column=4, padx=4)
        ttk.Separator(frm).grid(row=1, column=0, columnspan=9, sticky='ew', pady=4)

        # 创建表头样式
        header_style = ttk.Style()
        header_style.configure('Table.Header.TLabel', 
                             background='#f0f0f0', 
                             font=('Microsoft YaHei UI', 12, 'bold'),
                             anchor='center')
        
        headers = ["关节", "原始弧度", "原始角度(度)", "目标角(度)", "实测角度(度)", "速度(rpm)", "状态", "下限", "上限"]
        for j,h in enumerate(headers):
            header_lbl = ttk.Label(frm, text=h, style='Table.Header.TLabel')
            header_lbl.grid(row=2, column=j, padx=0, pady=0, sticky='nsew', ipadx=8, ipady=6)
            # 配置列的权重，让数据列可以扩展
            frm.grid_columnconfigure(j, weight=1)

        # 表头底部分隔线 (插入在表头和数据行之间)
        header_separator = ttk.Separator(frm, orient='horizontal')
        header_separator.grid(row=3, column=0, columnspan=9, sticky='ew', pady=(2,2))

        # 创建数据行样式
        data_style = ttk.Style()
        data_style.configure('Table.Data.TLabel', 
                           background='white', 
                           font=('Microsoft YaHei UI', 11),
                           anchor='center')
        data_style.configure('Table.Name.TLabel', 
                           background='#f8f8f8', 
                           font=('Microsoft YaHei UI', 11, 'bold'),
                           anchor='center')

        self.rows = []
        for i,name in enumerate(self.mapper.names):
            r = 4+i  # 从第4行开始，第3行是分隔线
            # Excel风格的单元格，无边框，用背景色区分
            bg_color = '#ffffff' if i % 2 == 0 else '#f9f9f9'  # 隔行变色
            
            name_lbl = ttk.Label(frm, text=name, style='Table.Name.TLabel')
            raw_rad = ttk.Label(frm, text="0.000000", style='Table.Data.TLabel')
            raw_deg = ttk.Label(frm, text="0.000", style='Table.Data.TLabel')
            map_deg = ttk.Label(frm, text="0.000", style='Table.Data.TLabel')
            act_deg = ttk.Label(frm, text="-", style='Table.Data.TLabel')
            speed_rpm = ttk.Label(frm, text="-", style='Table.Data.TLabel')
            status  = ttk.Label(frm, text="-", style='Table.Data.TLabel')
            mn, mx = self.mapper.lim[i]
            lim_min = ttk.Label(frm, text=f"{mn:.1f}", style='Table.Data.TLabel')
            lim_max = ttk.Label(frm, text=f"{mx:.1f}", style='Table.Data.TLabel')
            
            # 隔行变色设置
            if i % 2 == 1:
                for lbl in [raw_rad, raw_deg, map_deg, act_deg, speed_rpm, status, lim_min, lim_max]:
                    lbl.configure(background='#f9f9f9')
            
            # 网格布局，紧密排列
            name_lbl.grid(row=r, column=0, sticky='nsew', ipadx=6, ipady=4)
            raw_rad.grid(row=r, column=1, sticky='nsew', ipadx=6, ipady=4)
            raw_deg.grid(row=r, column=2, sticky='nsew', ipadx=6, ipady=4)
            map_deg.grid(row=r, column=3, sticky='nsew', ipadx=6, ipady=4)
            act_deg.grid(row=r, column=4, sticky='nsew', ipadx=6, ipady=4)
            speed_rpm.grid(row=r, column=5, sticky='nsew', ipadx=6, ipady=4)
            status.grid(row=r, column=6, sticky='nsew', ipadx=6, ipady=4)
            lim_min.grid(row=r, column=7, sticky='nsew', ipadx=6, ipady=4)
            lim_max.grid(row=r, column=8, sticky='nsew', ipadx=6, ipady=4)
            
            self.rows.append((raw_rad, raw_deg, map_deg, act_deg, speed_rpm, status))

        # 表格底部分隔线
        table_separator = ttk.Separator(frm, orient='horizontal')
        table_separator.grid(row=10, column=0, columnspan=9, sticky='ew', pady=(2,8))
        
        ttk.Separator(frm).grid(row=11, column=0, columnspan=9, sticky='ew', pady=8)

        # —— 速度调节区（移动到按钮区上方）——
        speed_row = ttk.Frame(frm)
        speed_row.grid(row=12, column=0, columnspan=9, sticky='ew')
        ttk.Label(speed_row, text="速度与加减速度", style='Title.TLabel').grid(row=0, column=0, padx=(0,12), sticky='w')
        ttk.Label(speed_row, text="速度RPM").grid(row=0, column=1, sticky='e')
        self.var_speed = tk.IntVar(value=self.max_speed)
        spd = tk.Spinbox(speed_row, from_=10, to=3000, increment=10, width=6, textvariable=self.var_speed, command=self._apply_speed_params)
        spd.grid(row=0, column=2, padx=(4,12))
        spd.configure(background='white', readonlybackground='white')
        ttk.Label(speed_row, text="加速度RPM/s").grid(row=0, column=3, sticky='e')
        self.var_acc = tk.IntVar(value=self.acc)
        acc = tk.Spinbox(speed_row, from_=10, to=10000, increment=10, width=6, textvariable=self.var_acc, command=self._apply_speed_params)
        acc.grid(row=0, column=4, padx=(4,12))
        acc.configure(background='white', readonlybackground='white')
        ttk.Label(speed_row, text="减速度RPM/s").grid(row=0, column=5, sticky='e')
        self.var_dec = tk.IntVar(value=self.dec)
        dec = tk.Spinbox(speed_row, from_=10, to=10000, increment=10, width=6, textvariable=self.var_dec, command=self._apply_speed_params)
        dec.grid(row=0, column=6, padx=(4,12))
        dec.configure(background='white', readonlybackground='white')

        # —— 手动关节控制区 ——
        manual = ttk.Frame(frm)
        manual.grid(row=13, column=0, columnspan=9, sticky='ew', pady=(6,0))
        ttk.Label(manual, text="关节角度控制", style='Title.TLabel').grid(row=0, column=0, sticky='w', padx=(0,12))
        # 每轴微调控件
        self.var_step = tk.DoubleVar(value=1.0)
        ttk.Label(manual, text="步长(°)").grid(row=0, column=2, sticky='e')
        tk.Spinbox(manual, from_=0.1, to=30.0, increment=0.1, width=6, textvariable=self.var_step).grid(row=0, column=3, sticky='w', padx=(4,12))
        self.joint_vars = []
        for i, name in enumerate(self.mapper.names):
            r = 1 + i//3
            c = (i%3)*3
            box = ttk.Frame(manual)
            box.grid(row=r, column=c, columnspan=3, sticky='w', padx=(0,12), pady=2)
            ttk.Label(box, text=name).grid(row=0, column=0, padx=(0,6))
            var = tk.DoubleVar(value=0.0)
            self.joint_vars.append(var)
            ent = tk.Spinbox(box, from_=-360.0, to=360.0, increment=0.1, width=7, textvariable=var)
            ent.grid(row=0, column=1)
            ttk.Button(box, text="-", width=2, command=lambda idx=i: self._nudge_joint(idx, -self.var_step.get())).grid(row=0, column=2, padx=2)
            ttk.Button(box, text="+", width=2, command=lambda idx=i: self._nudge_joint(idx, self.var_step.get())).grid(row=0, column=3, padx=2)

        # —— 按钮区（重构布局以解决扩展问题）——
        btn_row = ttk.Frame(frm)
        btn_row.grid(row=14, column=0, columnspan=9, sticky='ew', pady=(6,0))
        
        # 配置btn_row的内部grid，让状态标签列(0)可以扩展，按钮列(1)不扩展
        btn_row.grid_columnconfigure(0, weight=1)
        btn_row.grid_columnconfigure(1, weight=0)

        # 1. 状态标签（放在第0列，左对齐）
        self.status = tk.StringVar(value="等待数据...")
        # wraplength 确保文本过长时会自动换行，而不是无限拉伸UI
        status_label = ttk.Label(btn_row, textvariable=self.status, style='Info.TLabel', wraplength=400) 
        status_label.grid(row=0, column=0, sticky='w')

        # 2. 创建一个单独的Frame来容纳所有按钮（放在第1列，右对齐）
        buttons_frame = ttk.Frame(btn_row)
        buttons_frame.grid(row=0, column=1, sticky='e')

        # 将所有按钮添加到新的 buttons_frame 中，使用 pack 或 grid 均可
        ttk.Button(buttons_frame, text="下发到位 (goto)", command=self._send_goto).pack(side='left', padx=4)
        ttk.Button(buttons_frame, text="停止/断电(全部)", command=self._stop_all).pack(side='left', padx=4)
        ttk.Button(buttons_frame, text="重置回零", command=self._set_home_now).pack(side='left', padx=4)
        ttk.Button(buttons_frame, text="回零", command=self._go_home).pack(side='left', padx=4)
        self.btn_stream = ttk.Button(buttons_frame, text="实时跟随(关)", command=self._toggle_stream)
        self.btn_stream.pack(side='left', padx=6)


    def _update_table(self, rad, deg, mapped):
        def set_lbl(lbl, val, warn=False):
            lbl.config(text=f"{val:.6f}" if lbl in (self.rows[0][0],) else f"{val:.3f}")
            lbl.config(foreground=("red" if warn else "black"))
        clamped = self.mapper.clamp_only(deg)
        for i,(raw_rad, raw_deg, map_deg, act_deg, speed_rpm, status) in enumerate(self.rows):  # 添加speed_rpm
            mn, mx = self.mapper.lim[i]
            warn = (deg[i] <= mn+1e-6) or (deg[i] >= mx-1e-6)
            raw_rad.config(text=f"{rad[i]:.6f}")
            raw_deg.config(text=f"{deg[i]:.3f}")
            map_deg.config(text=f"{mapped[i]:.3f}", foreground=("red" if warn else "black"))
        self.status.set("已接收最新帧，确认无误后可点击'下发到位'")

    def _connect_ctrls(self):
        self.com_port = self.com_var.get().strip() or self.com_port
        if self.ctrls or self._sdk: return
        try:
            with self.io_lock:
                if SimpleArmSDK is not None:
                    self._sdk = SimpleArmSDK(
                        motor_ids=self.cfg.get('motor_ids', [1,2,3,4,5,6]),
                        port_or_channel=self.com_port,
                        bitrate=self.baudrate,
                        sign=self.mapper.sign,
                        gear_ratio=self.mapper.gear,
                        zero_offset_deg=self.mapper.zero,
                        # SDK 侧统一在电机空间做映射，避免UI+SDK重复映射
                        send_space='motor',
                    )
                    self._sdk.connect(enable=True)
                else:
                    # 旧路径保留
                    if ZDTMotorControllerModular is None:
                        raise RuntimeError("未找到底层SDK")
                    self.ctrls = []
                    port = self.com_port
                    is_socketcan = port.lower().startswith('can')
                    if is_socketcan:
                        self.bc = ZDTMotorControllerModular(motor_id=0, interface_type='socketcan', channel=port, bitrate=self.baudrate)
                    else:
                        self.bc = ZDTMotorControllerModular(motor_id=0, interface_type='slcan', port=port, baudrate=self.baudrate)
                    self.bc.connect()
                    import time
                    for mid in self.cfg['motor_ids']:
                        if is_socketcan:
                            c = ZDTMotorControllerModular(motor_id=mid, interface_type='socketcan', channel=port, bitrate=self.baudrate)
                        else:
                            c = ZDTMotorControllerModular(motor_id=mid, interface_type='slcan', port=port, baudrate=self.baudrate)
                        c.connect(); c.control_actions.enable()
                        self.ctrls.append(c)
                        time.sleep(self.connect_gap_ms/1000.0)
            # 连接后立即刷新一次状态，符合你主程序“连接即校验”的体验
            self._refresh_status_once()
            # 回零策略：固定为各轴0度
            print("[HOME] 回零点已设置为各轴0度")
            messagebox.showinfo("连接", (f"已连接 6 个电机 @ {self.com_port}" if self._sdk else f"已连接 {len(self.ctrls)} 个电机 @ {self.com_port}"))
        except Exception as e:
            messagebox.showerror("错误", f"连接电机失败: {e}")
            self.ctrls = []
            self._sdk = None

    def _disconnect_ctrls(self, disable: bool = True):
        try:
            with self.io_lock:
                if self._sdk:
                    try:
                        self._sdk.close(disable=disable)
                    except Exception:
                        pass
                    self._sdk = None
                else:
                    for c in self.ctrls:
                        try:
                            if disable:
                                c.control_actions.disable()
                            c.disconnect()
                        except Exception:
                            pass
                    if self.bc:
                        try:
                            if disable:
                                self.bc.control_actions.disable()
                            self.bc.disconnect()
                        except Exception:
                            pass
        finally:
            self.ctrls.clear(); self.bc = None
            messagebox.showinfo("断开", ("已断开并失能" if disable else "已断开(未失能)"))

    def _send_goto(self):
        self._connect_ctrls()
        if not self.ctrls and not self._sdk:
            return
        try:
            import time
            self.busy = True
            with self.io_lock:
                # 生成最终下发目标（一次性绝对目标，参考主程序）
                targets_abs_out = list(self.last_target_output)
                if self._sdk:
                    self._sdk.goto_output_abs(
                        targets_abs_out,
                        max_speed_rpm=self.max_speed,
                        acc=self.acc,
                        dec=self.dec,
                    )
                else:
                    targets_abs_motor = []
                    for i in range(6):
                        direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                        ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                        targets_abs_motor.append(direction * targets_abs_out[i] * ratio)

                # 控制台打印：目标-输出端角 与 最终下发（相对/绝对）
                try:
                    print("========== 下发准备 ==========")
                    print(f"模式(send_space) = {self.mapper.send_space}")
                    print("目标-输出端角(°):  " + ", ".join(f"{v:.3f}" for v in targets_abs_out))
                    print("当前实测-输出端(°):" + ", ".join(f"{v:.3f}" for v in self.last_output_measured))
                    print("逐轴详情(输出端°): 关节名 | 电机ID | 目标(°)")
                    for i, tgt in enumerate(targets_abs_out):
                        name = self.mapper.names[i] if i < len(self.mapper.names) else f"J{i+1}"
                        mid = self.cfg.get('motor_ids', [1,2,3,4,5,6])[i]
                        print(f"  {name:>6} | {mid:>6} | {tgt:>9.3f}")
                except Exception:
                    pass

                # 逐轴下发（带multi_sync）——一次性绝对目标
                # 注意：同步算法已在trajectory_stream_sdk_driver层统一处理
                if (not self._sdk) and self.mapper.send_space == 'motor':
                    for c, motor_abs in zip(self.ctrls, targets_abs_motor):
                        c.control_actions.move_to_position_trapezoid(
                            position=motor_abs,
                            max_speed=self.max_speed,
                            acceleration=self.acc,
                            deceleration=self.dec,
                            is_absolute=True,
                            multi_sync=True,
                        )
                elif (not self._sdk):
                    for c, target_out in zip(self.ctrls, targets_abs_out):
                        c.control_actions.move_to_position_trapezoid(
                            position=target_out,
                            max_speed=self.max_speed,
                            acceleration=self.acc,
                            deceleration=self.dec,
                            is_absolute=True,
                            multi_sync=True,
                        )
                # 同步触发（旧路径）
                if (not self._sdk) and self.bc:
                    try:
                        self.bc.control_actions.sync_motion()
                    except Exception:
                        pass
            messagebox.showinfo("下发", "已下发到位命令。")
        except Exception as e:
            messagebox.showerror("错误", f"下发失败: {e}")
        finally:
            self.busy = False

    def _setup_safe_exit(self):
        """设置安全退出处理，防止异常退出导致机械臂失能砸桌"""
        def safe_exit_handler(*args):
            """安全退出处理器 - 保持机械臂使能状态"""
            print("\n🛡️ 检测到程序退出，执行安全断开（保持使能）...")
            try:
                self.stop_flag = True
                # 安全断开：不失能机械臂
                self._disconnect_ctrls(disable=False)
            except Exception as e:
                print(f"安全退出处理异常: {e}")
            finally:
                # 确保程序能够退出
                try:
                    self.root.quit()
                    self.root.destroy()
                except Exception:
                    pass
                import os
                os._exit(0)
        
        # 注册信号处理器（处理Ctrl+C等）
        signal.signal(signal.SIGINT, safe_exit_handler)
        signal.signal(signal.SIGTERM, safe_exit_handler)
        
        # 注册atexit处理器（处理正常退出）
        atexit.register(lambda: safe_exit_handler())
        
        print("🛡️ 安全退出机制已启用 - 程序异常退出时机械臂将保持使能")

    def _update_ui_from_servo_data(self, vals: list):
        """从servo数据更新UI显示"""
        try:
            if len(vals) != 6:
                return
            
            # 更新目标角度和最后目标输出
            self.last_target_output = vals[:]
            
            # 转换为弧度用于UI更新
            rad = [v * math.pi / 180.0 for v in vals]
            deg = vals[:]
            mapped = self.mapper.map_deg(deg)
            
            # 更新表格显示
            self._update_table(rad, deg, mapped)
            
            # 更新滑条位置
            for i in range(min(len(self.joint_vars), len(vals))):
                self.joint_vars[i].set(round(vals[i], 3))
                
        except Exception as e:
            print(f"UI数据更新失败: {e}")

    def _commit_pending_target(self):
        """将缓存的最后一帧目标一次性Goto（FD为主，参考主程序）。"""
        if not self._pending_target_output or not self.ctrls:
            return
        with self.io_lock:
            if self.mapper.send_space == 'motor':
                # 计算相对位移（避免大角）
                def norm180(x):
                    while x > 180.0: x -= 360.0
                    while x < -180.0: x += 360.0
                    return x
                targets_rel = []
                for i in range(6):
                    target_out = self._pending_target_output[i]
                    meas_out = self.last_output_measured[i]
                    delta_out = norm180(target_out - meas_out)
                    direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                    ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                    delta_motor = direction * delta_out * ratio
                    targets_rel.append(delta_motor)
                # 逐轴FD + multi_sync，再sync_motion
                for c, delta in zip(self.ctrls, targets_rel):
                    c.control_actions.move_to_position_trapezoid(
                        position=delta,
                        max_speed=self.max_speed,
                        acceleration=self.acc,
                        deceleration=self.dec,
                        is_absolute=False,
                        multi_sync=True,
                    )
                if self.bc:
                    try:
                        self.bc.control_actions.sync_motion()
                    except Exception:
                        pass
        # 提交后清空
        self._pending_target_output = None

    def _apply_speed_params(self):
        try:
            self.max_speed = int(self.var_speed.get())
            self.acc = int(self.var_acc.get())
            self.dec = int(self.var_dec.get())
        except Exception:
            pass

    def _stop_all(self):
        try:
            with self.io_lock:
                if self._sdk:
                    try:
                        self._sdk.stop_all()
                        self._sdk.disable_all()
                    except Exception:
                        pass
                else:
                    if self.bc:
                        try:
                            self.bc.control_actions.stop(); self.bc.control_actions.disable()
                        except Exception:
                            pass
                    for c in self.ctrls:
                        try:
                            c.control_actions.stop(); c.control_actions.disable()
                        except Exception:
                            pass
            messagebox.showinfo("停止", "已发送停止/断电")
        except Exception:
            pass

    # ---------- 单轴微调/执行 ----------
    def _nudge_joint(self, idx: int, delta_out_deg: float):
        if idx < 0 or idx >= 6:
            return
        try:
            with self.io_lock:
                if self._sdk:
                    self._sdk.nudge_single_output(idx, delta_out_deg, max_speed_rpm=self.max_speed, acc=self.acc, dec=self.dec)
                else:
                    if not self.ctrls or idx >= len(self.ctrls):
                        return
                    direction = (self.mapper.sign[idx] if self.mapper.apply_sign_in_ui else 1)
                    ratio = self.mapper.gear[idx] if self.mapper.gear[idx] else 1
                    delta_motor = direction * float(delta_out_deg) * ratio
                    self.ctrls[idx].control_actions.move_to_position_trapezoid(
                        position=delta_motor,
                        max_speed=self.max_speed,
                        acceleration=self.acc,
                        deceleration=self.dec,
                        is_absolute=False,
                        multi_sync=False,
                    )
                # 本地预测与UI同步（立即更新显示与输入框）
                direction = (self.mapper.sign[idx] if self.mapper.apply_sign_in_ui else 1)
                reducer = self.mapper.gear[idx] if self.mapper.gear[idx] else 1
                self.last_motor_pos[idx] = float(self.last_motor_pos[idx]) + float(direction * float(delta_out_deg) * reducer)
                pos_out = (self.last_motor_pos[idx] * direction) / reducer
                wrap = float(self.wrap_deg[idx] if idx < len(self.wrap_deg) else 180.0)
                pos_out = self._wrap_to_range(pos_out, wrap)
                self.last_output_measured[idx] = pos_out
                try:
                    self.joint_vars[idx].set(round(pos_out, 3))
                except Exception:
                    pass
        except Exception:
            pass


    # ---------- 实时跟随 ----------
    def _toggle_stream(self):
        self.stream_enabled = not self.stream_enabled
        self.btn_stream.config(text=("实时跟随(开)" if self.stream_enabled else "实时跟随(关)"))
        try:
            if self.stream_enabled:
                # UI 单主控：实时模式仍由本地直控并持续串口回读
                self.status.set("实时跟随：UI本地直控并持续串口回读")
                try:
                    # 以当前实测角初始化控制态，避免首次大跳
                    if self.last_output_measured and len(self.last_output_measured)==6:
                        self.ctrl_state_out = list(self.last_output_measured)
                except Exception:
                    pass
            else:
                self.status.set("实时已关闭：UI本地直控")
        except Exception:
            pass


    def _normalize_angle(self, angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle

    def _wrap_to_range(self, angle: float, wrap_deg: float) -> float:
        # 将角度包裹到(-wrap/2, wrap/2]
        half = wrap_deg / 2.0
        while angle > half:
            angle -= wrap_deg
        while angle <= -half:
            angle += wrap_deg
        # 接近±wrap视为0，避免套圈残留
        if abs(abs(angle) - wrap_deg) <= self.wrap_epsilon or abs(angle) >= (wrap_deg - self.wrap_epsilon):
            return 0.0
        return angle

    def _nearest_delta(self, target_out: float, meas_out: float, wrap_deg: float) -> float:
        # 目标与测量均按关节wrap包裹，再取最小角差
        t = self._wrap_to_range(target_out, wrap_deg)
        m = self._wrap_to_range(meas_out, wrap_deg)
        delta = t - m
        # 再做一次规范到(-wrap/2, wrap/2]
        return self._wrap_to_range(delta, wrap_deg)

    def _send_stream_step(self, output_target):
        """实时跟随：
        - 若已连接真机（_sdk 或 ctrls）：优先用本地直控（单主控）。
        - 否则（未连接），退回发布到驱动话题 /horizon_arm/servo_target（绝对输出端角，单位°）。
        """
        try:
            self.in_stream_tick = True
            # 1) 判稳：实测角在目标±tol 内连续 hold_iters 次，视为到点；
            #    若允许，则做一次绝对到位以钉死最终状态，随后停止继续相对步进。
            try:
                tol = float(self.stream_settle_tol_deg)
                within_all = True
                for i in range(6):
                    if abs(float(output_target[i]) - float(self.last_output_measured[i])) > tol:
                        within_all = False
                        break
                if within_all:
                    self._settle_hits += 1
                else:
                    # 目标改变或未达到阈值，重置计数；如目标变化，清空已终结标记
                    if isinstance(self._finalized_target, list):
                        for i in range(6):
                            if abs(float(output_target[i]) - float(self._finalized_target[i])) > tol:
                                self._finalized_target = None
                                break
                    self._settle_hits = 0

                if self._settle_hits >= int(self.stream_settle_hold_iters):
                    same_as_final = False
                    if isinstance(self._finalized_target, list):
                        same_as_final = all(abs(float(output_target[i]) - float(self._finalized_target[i])) <= tol for i in range(6))
                    if not same_as_final and self.stream_finalize_with_abs:
                        with self.io_lock:
                            if self._sdk:
                                self._sdk.goto_output_abs(list(output_target), max_speed_rpm=self.max_speed, acc=self.acc, dec=self.dec)
                            elif self.ctrls:
                                # 旧路径：将“当前电机角+增量”作为绝对目标
                                targets_abs_motor = []
                                for i in range(6):
                                    direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                                    ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                                    delta_out = float(output_target[i]) - float(self.last_output_measured[i])
                                    targets_abs_motor.append(float(self.last_motor_pos[i]) + direction * delta_out * ratio)
                                for c, target in zip(self.ctrls, targets_abs_motor):
                                    c.control_actions.move_to_position_trapezoid(
                                        position=float(target),
                                        max_speed=int(self.max_speed),
                                        acceleration=int(self.acc),
                                        deceleration=int(self.dec),
                                        is_absolute=True,
                                        multi_sync=False,
                                    )
                        self._finalized_target = list(output_target)
                    # 已收敛，停止进一步相对步进
                    return
            except Exception:
                pass
            # 1) 本地直控：SimpleArmSDK 优先
            if self._sdk:
                deltas_out = []
                changed = False
                for i in range(6):
                    wrap = float(self.wrap_deg[i] if i < len(self.wrap_deg) else 180.0)
                    # 控制基线采用 ctrl_state_out，避免使用滞后回读导致反复修正
                    base_out = float(self.ctrl_state_out[i] if (self.stream_enabled and self.ctrl_state_out) else self.last_output_measured[i])
                    delta_out = self._nearest_delta(float(output_target[i]), base_out, wrap)
                    if abs(delta_out) < self.stream_min_delta_deg:
                        deltas_out.append(0.0)
                        continue
                    # 目标限位到输出端上下限
                    lo, hi = self.mapper.lim[i]
                    cur = base_out
                    tgt = min(max(cur + delta_out, lo), hi)
                    deltas_out.append(tgt - cur)
                    if abs(tgt - cur) >= self.stream_min_delta_deg:
                        changed = True
                if not changed:
                    return
                # 串口IO串行化
                with self.io_lock:
                    if self.stream_use_direct:
                        speed = float(self.stream_speed_rpm if self.stream_speed_rpm>0 else self.max_speed)
                        self._sdk.servo_step_output_direct(deltas_out, speed_rpm=speed, acc=self.acc, dec=self.dec)
                    else:
                        self._sdk.servo_step_output(deltas_out, max_speed_rpm=float(self.max_speed), acc=self.acc, dec=self.dec)
                # 更新控制态（仅内部基线，不改UI显示）
                try:
                    for i in range(6):
                        self.ctrl_state_out[i] = float(self.ctrl_state_out[i]) + float(deltas_out[i])
                except Exception:
                    pass
                try:
                    import time
                    time.sleep(self.send_gap_ms/1000.0)
                except Exception:
                    pass
                return

            # 旧路径：直接用底层 ctrls
            if self.ctrls:
                # 本地SDK直接下发（最小角差 → 电机相对位移）
                with self.io_lock:
                    changed = False
                    deltas_motor = []
                    for i in range(6):
                        wrap = float(self.wrap_deg[i] if i < len(self.wrap_deg) else 180.0)
                        base_out = float(self.ctrl_state_out[i] if (self.stream_enabled and self.ctrl_state_out) else self.last_output_measured[i])
                        delta_out = self._nearest_delta(float(output_target[i]), base_out, wrap)
                        if abs(delta_out) < self.stream_min_delta_deg:
                            deltas_motor.append(0.0)
                            continue
                        changed = True
                        direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                        ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                        delta_motor = direction * delta_out * ratio
                        deltas_motor.append(delta_motor)
                    if not changed:
                        return
                # 逐轴发送（不长时间持锁）
                for i, (c, delta) in enumerate(zip(self.ctrls, deltas_motor)):
                    if abs(delta) < 1e-6:
                        continue
                    if self.stream_use_direct:
                        # 限幅速度
                        try:
                            vel_lim = list(self.cfg.get('vel_limit_deg_s', [15,15,15,20,20,30]))
                        except Exception:
                            vel_lim = [15,15,15,20,20,30]
                        gear = float(self.mapper.gear[i] if i < len(self.mapper.gear) and self.mapper.gear[i] else 1.0)
                        deg_s = float(vel_lim[i] if i < len(vel_lim) else 20)
                        max_rpm = deg_s * gear / 6.0
                        base_speed = float(self.stream_speed_rpm if self.stream_speed_rpm>0 else self.max_speed)
                        speed = min(base_speed, max_rpm)
                        try:
                            with self.io_lock:
                                c.control_actions.move_to_position(
                                    position=float(delta),
                                    speed=float(speed),
                                    is_absolute=False,
                                    multi_sync=True,
                                )
                        except Exception:
                            # 退回到梯形（同样限幅）
                            try:
                                with self.io_lock:
                                    c.control_actions.move_to_position_trapezoid(
                                        position=float(delta),
                                        max_speed=float(speed),
                                        acceleration=int(self.acc),
                                        deceleration=int(self.dec),
                                        is_absolute=False,
                                        multi_sync=True,
                                    )
                            except Exception:
                                pass
                    else:
                        try:
                            with self.io_lock:
                                c.control_actions.move_to_position_trapezoid(
                                    position=float(delta),
                                    max_speed=int(self.max_speed),
                                    acceleration=int(self.acc),
                                    deceleration=int(self.dec),
                                    is_absolute=False,
                                    multi_sync=True,
                                )
                        except Exception:
                            pass
                    # 更新控制态（从输出端角增量推算）
                    try:
                        direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                        ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                        delta_out = float(delta) / (direction * (ratio if ratio else 1.0))
                        self.ctrl_state_out[i] = float(self.ctrl_state_out[i]) + float(delta_out)
                    except Exception:
                        pass
                    try:
                        import time
                        time.sleep(self.send_gap_ms/1000.0)
                    except Exception:
                        pass
                if self.bc:
                    try:
                        with self.io_lock:
                            self.bc.control_actions.sync_motion()
                    except Exception:
                        pass
                # 不做预测更新：以硬件回读为准，避免重复套用映射导致数值漂移
                return

            # 2) 未连接情况下，发布到驱动话题作为后备
            if (not self._sdk) and (not self.ctrls) and getattr(self, '_servo_pub', None):
                from std_msgs.msg import Float64MultiArray  # type: ignore
                msg = Float64MultiArray()
                msg.data = [float(v) for v in output_target]
                self._servo_pub.publish(msg)
        except Exception:
            pass
        finally:
            self.in_stream_tick = False

    def _stream_loop(self):
        # 周期性将 UI 当前目标（last_target_output）发布出去（当实时跟随开启时）
        import time
        period = 1.0 / max(1.0, float(self.servo_rate_hz))
        while not self.stop_flag:
            try:
                if self.stream_enabled and (not self.busy) and getattr(self, 'last_target_output', None):
                    self._send_stream_step(self.last_target_output)
            except Exception:
                pass
            time.sleep(period)

    def _status_loop(self):
        import time
        while not self.stop_flag:
            if (not self.ctrls and not self._sdk) or self.busy:
                time.sleep(1.0); continue
            # 软避让：若实时下发正在进行，稍候再读，避免串口抢占
            if getattr(self, 'in_stream_tick', False):
                time.sleep(0.01)
                continue
            self._refresh_status_once()
            time.sleep(1.0)

    def _refresh_status_once(self):
        """立即读取一次实测角度与状态，更新到表格。"""
        try:
            positions = []
            statuses  = []
            import time
            with self.io_lock:
                if self._sdk:
                    try:
                        out_list = self._sdk.read_positions_output()
                    except Exception:
                        out_list = [float('nan')]*6
                    for i in range(min(6, len(out_list))):
                        try:
                            self.last_output_measured[i] = float(out_list[i])
                        except Exception:
                            self.last_output_measured[i] = float('nan')
                    positions = list(out_list)
                    statuses = ["已连接"]*len(positions)
                else:
                    for idx, c in enumerate(self.ctrls):
                        try:
                            pos = c.read_parameters.get_position()
                            st = c.read_parameters.get_motor_status()
                            self.last_motor_pos[idx] = pos
                            direction = (self.mapper.sign[idx] if self.mapper.apply_sign_in_ui else 1)
                            reducer = self.mapper.gear[idx] if self.mapper.gear[idx] else 1
                            pos_out = (pos * direction) / reducer
                            wrap = float(self.wrap_deg[idx] if idx < len(self.wrap_deg) else 180.0)
                            pos_out = self._wrap_to_range(pos_out, wrap)
                            self.last_output_measured[idx] = pos_out
                            positions.append(pos_out if (self.mapper.show_output_measured and self.mapper.gear) else pos)
                            statuses.append("使能" if st.enabled else "失能")
                        except Exception:
                            positions.append(float('nan'))
                            statuses.append("?")
                        time.sleep(self.read_gap_ms/1000.0)
            for i,(_,_,_, act_deg, speed_lbl, stat_lbl) in enumerate(self.rows):
                try:
                    act_deg.config(text=("nan" if math.isnan(positions[i]) else f"{positions[i]:.3f}"))
                    # 简单的速度估算（基于角度差）
                    if hasattr(self, 'last_target_output') and i < len(self.last_target_output):
                        angle_diff = abs(self.last_target_output[i] - positions[i]) if not math.isnan(positions[i]) else 0
                        if angle_diff > 0.5:  # 只有显著运动时才显示速度
                            gear_ratio = self.mapper.gear[i] if i < len(self.mapper.gear) else 50.0
                            estimated_rpm = min(angle_diff * gear_ratio / 6.0, self.max_speed)
                            speed_lbl.config(text=f"{estimated_rpm:.0f}")
                        else:
                            speed_lbl.config(text="-")
                    else:
                        speed_lbl.config(text="-")
                    stat_lbl.config(text=statuses[i])
                except Exception:
                    pass
            try:
                for i in range(min(len(self.joint_vars), len(self.last_output_measured))):
                    self.joint_vars[i].set(round(float(self.last_output_measured[i]), 3))
            except Exception:
                pass
            if getattr(self, 'no_udp', False):
                raw_deg = list(self.last_output_measured)
                raw_rad = [v*math.pi/180.0 for v in raw_deg]
                mapped = list(self.last_output_measured)
                try:
                    self._update_table(raw_rad, raw_deg, mapped)
                    self.status.set("本地直控 - 显示为实测角")
                except Exception:
                    pass
        except Exception:
            pass



    # ---------- 回零功能 ----------
    def _set_home_now(self):
        """重置回零点为各轴0度。"""
        self.home_output = [0.0] * 6
        print("[HOME] 回零点已重置为各轴0度")
        messagebox.showinfo("回零", "回零点已重置为各轴0度")

    def _go_home(self):
        """回到回零点（各轴0度）。"""
        # 回零目标固定为各轴0度
        self.home_output = [0.0] * 6
        # 生成一次“目标”为回零点的下发（复用最小角差逻辑）
        try:
            import time
            self.busy = True
            with self.io_lock:
                def norm180(x):
                    while x > 180.0: x -= 360.0
                    while x < -180.0: x += 360.0
                    return x
                if self._sdk:
                    # 直接绝对到位到 home_output（输出端角）
                    self._sdk.goto_output_abs(self.home_output, max_speed_rpm=self.max_speed, acc=self.acc, dec=self.dec)
                else:
                    targets = []
                    for i in range(6):
                        target_out = self.home_output[i]
                        meas_out = self.last_output_measured[i]
                        delta_out = norm180(target_out - meas_out)
                        direction = (self.mapper.sign[i] if self.mapper.apply_sign_in_ui else 1)
                        ratio = self.mapper.gear[i] if self.mapper.gear[i] else 1
                        delta_motor = direction * delta_out * ratio
                        target_motor_abs = self.last_motor_pos[i] + delta_motor
                        targets.append(target_motor_abs)
                    print("[HOME] 回零下发(电机角°): ", ", ".join(f"{v:.3f}" for v in targets))
                    for c, target in zip(self.ctrls, targets):
                        c.control_actions.move_to_position_trapezoid(
                            position=target,
                            max_speed=self.max_speed,
                            acceleration=self.acc,
                            deceleration=self.dec,
                            is_absolute=True,
                            multi_sync=False,
                        )
                        time.sleep(self.send_gap_ms/1000.0)
            messagebox.showinfo("回零", "已下发回零命令")
        except Exception as e:
            messagebox.showerror("回零", f"失败: {e}")
        finally:
            self.busy = False

    #（移除“仅记录”功能）

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        """窗口关闭处理 - 保持使能状态"""
        self.stop_flag = True
        # 仅断开串口，不自动失能，避免机械臂下垂
        try:
            self._disconnect_ctrls(disable=False)
        except Exception:
            pass
        try:
            node = getattr(self, '_ros_node', None)
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
                try:
                    import rclpy  # type: ignore
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
        except Exception:
            pass
        self.root.destroy()

    def _scan_ids(self):
        """按 mapping.yaml 的 motor_ids 扫描（不全范围扫），找出在线ID并提示。"""
        if ZDTMotorControllerModular is None:
            messagebox.showerror("错误", "未找到 Control_SDK，无法扫描")
            return
        com = self.com_var.get().strip() or self.com_port
        ids = list(self.cfg.get('motor_ids', [1,2,3,4,5,6]))
        online = []
        try:
            # 逐个尝试连接并读状态，失败忽略
            for mid in ids:
                try:
                    c = ZDTMotorControllerModular(motor_id=mid, port=com, baudrate=self.baudrate)
                    c.connect()
                    try:
                        # 多尝试两次，兼容临时超时
                        ok = False
                        for _ in range(2):
                            try:
                                _ = c.read_parameters.get_motor_status()
                                ok = True; break
                            except Exception:
                                pass
                        if ok:
                            online.append(mid)
                    finally:
                        try: c.disconnect()
                        except Exception: pass
                except Exception:
                    pass
        except Exception as e:
            messagebox.showerror("错误", f"扫描失败: {e}")
            return
        if online:
            messagebox.showinfo("扫描结果", f"在线ID: {online}\n请将 mapping.yaml 的 motor_ids 改为上述列表，或直接在UI连接使用。")
        else:
            messagebox.showwarning("扫描结果", "未发现在线电机，请检查：\n- 串口是否被占用\n- 波特率是否与驱动板一致(默认500K)\n- CANH/CANL接线与终端电阻\n- 电机ID是否与 mapping.yaml 的 motor_ids 一致")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mapping', default='mapping.yaml')
    ap.add_argument('--com-port', default=None)
    args = ap.parse_args()
    ui = ReviewUI(args)
    ui.run()


if __name__ == '__main__':
    main()


