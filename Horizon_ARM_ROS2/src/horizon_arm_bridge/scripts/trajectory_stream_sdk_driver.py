#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import math
import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

import yaml
import sys
import os
sys.path.append(os.path.dirname(__file__))
from sync_movement_algorithm import create_sync_algorithm

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _resolve_project_paths() -> dict:
    here = Path(__file__).resolve()
    # scripts -> horizon_arm_bridge -> src -> Horizon_ARM_ROS2 -> repo_root
    horizon_arm_root = here.parents[3]
    repo_root = here.parents[4]
    sdk_root = horizon_arm_root / 'Control_SDK-ros'
    return {
        'repo_root': repo_root,
        'horizon_arm_root': horizon_arm_root,
        'sdk_root': sdk_root,
    }


def _safe_add_sys_path(p: Path) -> None:
    import sys
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)


class TrajectoryStreamSdkDriver(Node):
    def __init__(self):
        super().__init__('trajectory_stream_sdk_driver')

        paths = _resolve_project_paths()
        _safe_add_sys_path(paths['sdk_root'])
        
        # 算法相关参数已移至独立模块 sync_movement_algorithm.py
        # 初始化独立同步算法模块
        reducer_ratios = [62.0, 51.0, 51.0, 62.0, 12.0, 10.0]  # 从mapping读取减速比
        self.sync_algorithm = create_sync_algorithm(reducer_ratios)
        self.get_logger().info('✅ 独立同步算法模块已加载')

        # 兼容旧版 SLCAN：注入缺失的方法，避免实时/到位在某些固件上报错
        try:
            from Control_Core import can_interface as _cif  # type: ignore
            if hasattr(_cif, 'SLCANInterface'):
                # 注入多包发送
                if not hasattr(_cif.SLCANInterface, '_send_multi_packet_command'):
                    def _slcan_send_multi(self, motor_id: int, command_data: list, timeout: float):
                        base_frame_id = motor_id << 8
                        if not command_data:
                            return self.receive_message(base_frame_id, timeout)
                        function_code = command_data[0]
                        packets = [command_data[:8]]
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
                # 注入 send_command_and_receive_response
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

        # 优先导入封装后的SDK
        self.SimpleArmSDK = None
        self.ControllerClass = None
        try:
            from simple_sdk import SimpleArmSDK  # type: ignore
            self.SimpleArmSDK = SimpleArmSDK
        except ImportError:
            # 回退到原始SDK
            from Control_Core.motor_controller_modular import ZDTMotorControllerModular  # type: ignore
            self.ControllerClass = ZDTMotorControllerModular

        # 参数
        self.declare_parameter('mapping_path', 'arm-ros2/mapping.yaml')
        self.declare_parameter('resample_rate_hz', 50.0)
        self.declare_parameter('skip_first_point', True)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('hold_final_seconds', 2.0)
        # 硬件模式：auto|none|slcan|socketcan；none 表示仅仿真，不连接硬件
        self.declare_parameter('hardware_mode', 'auto')
        # 是否在启动时初始化并连接硬件（默认否，避免UI未连接前的无效使能日志）
        self.declare_parameter('enable_hw_on_start', False)
        # 空闲态 JointState 发布频率（Hz），用于让 MoveIt/RViz 在无轨迹时也有“当前关节”
        self.declare_parameter('js_idle_rate_hz', 30.0)
        # 是否启用实时跟随话题。话题名：/horizon_arm/servo_target（unit=deg）
        self.declare_parameter('enable_servo_topic', True)

        self.mapping_path_param = self.get_parameter('mapping_path').get_parameter_value().string_value
        self.resample_rate_hz = float(self.get_parameter('resample_rate_hz').get_parameter_value().double_value)
        self.skip_first_point = bool(self.get_parameter('skip_first_point').get_parameter_value().bool_value)
        self.publish_joint_states = bool(self.get_parameter('publish_joint_states').get_parameter_value().bool_value)
        self.hold_final_seconds = float(self.get_parameter('hold_final_seconds').get_parameter_value().double_value)
        self.hardware_mode = str(self.get_parameter('hardware_mode').get_parameter_value().string_value).strip().lower()
        self.js_idle_rate_hz = float(self.get_parameter('js_idle_rate_hz').get_parameter_value().double_value)
        self.enable_servo_topic = bool(self.get_parameter('enable_servo_topic').get_parameter_value().bool_value)
        self.enable_hw_on_start = bool(self.get_parameter('enable_hw_on_start').get_parameter_value().bool_value)

        self.mapping = self._load_mapping(paths)
        self.joint_names: List[str] = list(self.mapping['joints'])
        self.motor_ids: List[int] = list(self.mapping['motor_ids'])
        self.sign: List[int] = list(self.mapping['sign'])
        self.gear_ratio: List[float] = list(self.mapping['gear_ratio'])
        self.zero_offset_deg: List[float] = list(self.mapping['zero_offset_deg'])
        self.pos_limit_deg: List[List[float]] = [list(p) for p in self.mapping['pos_limit_deg']]
        self.vel_limit_deg_s: List[float] = list(self.mapping.get('vel_limit_deg_s', [15, 15, 15, 20, 20, 30]))
        self.send_space: str = str(self.mapping.get('send_space', 'motor')).strip().lower()
        self.stream_use_direct: bool = bool(self.mapping.get('stream_use_direct', False))
        self.stream_speed_rpm: float = float(self.mapping.get('stream_speed_rpm', 0))
        self.accel_rpm_s: float = float(self.mapping.get('acceleration_rpm_s', 400))
        self.decel_rpm_s: float = float(self.mapping.get('deceleration_rpm_s', 400))

        # SDK 接口设置
        self.com_port: str = str(self.mapping.get('com_port', '/dev/ttyACM0'))
        self.baudrate: int = int(self.mapping.get('baudrate', 500000))

        # 硬件控制器初始化（失败时降级为仅仿真）
        self.controllers: List[object] = []
        self._sdk = None  # SimpleArmSDK实例
        self.hw_ready: bool = False
        try:
            if self.hardware_mode != 'none' and self.enable_hw_on_start:
                if self.SimpleArmSDK:
                    self._init_sdk()
                else:
                    self.controllers = self._init_controllers()
                    self.hw_ready = (len(self.controllers) == 6)
                if not self.hw_ready and not self._sdk:
                    self.get_logger().warning('硬件未完全就绪，将仅发布 /joint_states 供 RViz 显示。')
            else:
                # 启动阶段不连接硬件；由 UI 负责连接与下发
                self.get_logger().info('启动时不连接硬件：仅发布 /joint_states 以支持 RViz。')
        except Exception as e:
            self.get_logger().warning(f'硬件初始化失败，降级为仅仿真：{e}')
            self.controllers = []
            self._sdk = None
            self.hw_ready = False

        self.js_pub = None
        # 当前输出端角（度），用于空闲态持续发布 joint_states
        self._current_out_deg: List[float] = [0.0 for _ in range(6)]
        self._js_timer = None
        self._read_timer = None
        if self.publish_joint_states:
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
            # 空闲态持续发布，保证 MoveIt 能获取到"当前关节态"
            if self.js_idle_rate_hz > 0:
                period = max(0.005, 1.0 / self.js_idle_rate_hz)
                self._js_timer = self.create_timer(period, self._publish_idle_js)
            # 定时读取真机状态
            if self.hw_ready or self._sdk:
                read_period = 0.1  # 10Hz读取频率
                self._read_timer = self.create_timer(read_period, self._read_hw_state)

        # 订阅 MoveIt 规划结果
        self.create_subscription(DisplayTrajectory, '/display_planned_path', self._on_display_traj, 10)
        self.create_subscription(DisplayTrajectory, '/move_group/display_planned_path', self._on_display_traj, 10)
        if self.enable_servo_topic:
            self.create_subscription(Float64MultiArray, '/horizon_arm/servo_target', self._on_servo_target, 10)

        self.stream_thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()
        # 缓存最近一次规划（不自动执行）
        self._last_planned_jt: Optional[JointTrajectory] = None
        self.get_logger().info('TrajectoryStreamSdkDriver 已就绪（直接SDK下发）。')

    def _calculate_sync_movement_params_OLD(self, target_positions_deg, current_positions_deg=None):
        """
        统一同步运动算法 - SDK上的格式化层
        适用于所有运动：UI控制、RViz规划、示例脚本、实时控制
        
        Args:
            target_positions_deg: 目标位置(度) [6个关节]
            current_positions_deg: 当前位置(度), None则使用内部状态
            
        Returns:
            (movement_time, sync_speeds_rpm): 运动时间和各轴同步速度
        """
        if not self.sync_algorithm_enabled:
            # 算法关闭时使用默认参数
            default_speed = 60.0
            return 2.0, [default_speed] * 6
        
        # 获取当前位置
        if current_positions_deg is None:
            current_positions_deg = self._current_out_deg[:]
        
        # 计算各轴角度差
        angle_diffs = []
        for i in range(6):
            current_pos = current_positions_deg[i] if i < len(current_positions_deg) else 0.0
            target_pos = target_positions_deg[i] if i < len(target_positions_deg) else 0.0
            angle_diff = abs(target_pos - current_pos)
            angle_diffs.append(angle_diff)
        
        max_angle_diff = max(angle_diffs) if angle_diffs else 1.0
        
        # 动态计算运动时间 - 基于最大角度差
        if max_angle_diff < 1.0:
            # 微小运动
            movement_time = self.min_movement_time
        elif max_angle_diff < 10.0:
            # 小角度运动 (如画正方形)
            movement_time = self.min_movement_time + (max_angle_diff / 10.0) * 0.5
        elif max_angle_diff < 45.0:
            # 中等角度运动
            movement_time = 1.0 + (max_angle_diff / 45.0) * 2.0
        else:
            # 大角度运动
            movement_time = 3.0 + (max_angle_diff / 180.0) * 5.0
        
        # 限制时间范围
        movement_time = max(self.min_movement_time, min(movement_time, self.max_movement_time))
        
        # 计算各轴同步速度 (RPM) - 参考高级算法
        sync_speeds_rpm = []
        for i in range(6):
            if angle_diffs[i] < 0.01:  # 提高精度阈值
                # 几乎不动的轴，逐渐减速
                speed = max(self.min_axis_speed_rpm, self.previous_speeds[i] * 0.9)
            else:
                # 计算输出端角速度 (度/秒)
                output_speed_deg_s = angle_diffs[i] / movement_time
                
                # 转换为电机端速度 (考虑减速比)
                motor_speed_deg_s = output_speed_deg_s * self.reducer_ratios[i]
                
                # 转换为RPM (度/秒 -> RPM: 除以6)
                raw_speed_rpm = motor_speed_deg_s / 6.0
                
                # 应用轴性能系数补偿
                compensated_speed = raw_speed_rpm / self.axis_performance_factors[i]
                
                # 速度平滑滤波 (低通滤波器)
                filtered_speed = (self.speed_filter_alpha * compensated_speed + 
                                (1 - self.speed_filter_alpha) * self.previous_speeds[i])
                
                # 限制速度变化率 (防止突变)
                speed_diff = abs(filtered_speed - self.previous_speeds[i])
                if speed_diff > self.max_speed_change_rpm:
                    if filtered_speed > self.previous_speeds[i]:
                        filtered_speed = self.previous_speeds[i] + self.max_speed_change_rpm
                    else:
                        filtered_speed = self.previous_speeds[i] - self.max_speed_change_rpm
                
                # 限制速度范围 (提高上限以支持更快速度)
                speed = max(self.min_axis_speed_rpm, min(filtered_speed, 200.0))
            
            # 更新历史速度
            self.previous_speeds[i] = speed
            sync_speeds_rpm.append(speed)
        
        # 特殊处理：如果第四轴速度明显偏低，整体提速
        if len(sync_speeds_rpm) > 3 and sync_speeds_rpm[3] < 8.0:
            boost_factor = min(2.0, 8.0 / sync_speeds_rpm[3])
            sync_speeds_rpm = [min(s * boost_factor, 200.0) for s in sync_speeds_rpm]
            self.get_logger().info(f'第四轴补偿: 提速系数 {boost_factor:.2f}')
        
        # 简洁的日志输出（避免刷屏）
        if max_angle_diff > 2.0:  # 只在大幅运动时打印详细日志
            self.get_logger().info(f'=== 同步算法: 最大角度差{max_angle_diff:.1f}°, 时间{movement_time:.1f}s ===')
            moving_axes = []
            for i in range(6):
                if angle_diffs[i] > 1.0:  # 只显示明显运动的轴
                    moving_axes.append(f'J{i+1}:{sync_speeds_rpm[i]:.0f}rpm')
            if moving_axes:
                self.get_logger().info(f'运动轴: {", ".join(moving_axes)}')
        elif max_angle_diff > 0.5:  # 中等运动，更简洁
            active_count = sum(1 for diff in angle_diffs if diff > 0.1)
            avg_speed = sum(sync_speeds_rpm) / len(sync_speeds_rpm)
            self.get_logger().info(f'同步运动: {active_count}轴, 平均{avg_speed:.0f}rpm')
        
        return movement_time, sync_speeds_rpm

    def _apply_sync_algorithm(self, target_angles: List[float]) -> Tuple[float, List[float]]:
        """
        调用独立同步算法模块
        
        Args:
            target_angles: 目标角度列表(度)
            
        Returns:
            Tuple[运动时间, 同步速度列表(RPM)]
        """
        # 更新算法的当前位置状态
        self.sync_algorithm.update_current_position(self._current_out_deg)
        
        # 调用独立算法模块
        movement_time, sync_speeds = self.sync_algorithm.calculate_sync_speeds(
            target_angles, self._current_out_deg
        )
        
        # 获取算法详情用于日志
        algo_info = self.sync_algorithm.get_algorithm_info(target_angles, self._current_out_deg)
        self._log_sync_algorithm(algo_info)
        
        return movement_time, sync_speeds

    def _log_sync_algorithm(self, algo_info: dict):
        """简洁的算法日志输出"""
        max_diff = algo_info['max_angle_diff']
        
        if max_diff > 2.0:  # 大幅运动，详细日志
            self.get_logger().info(
                f'🔄 同步算法: 最大{max_diff:.1f}°, 时间{algo_info["movement_time"]:.1f}s, '
                f'{algo_info["active_axes"]}轴运动, 平均{algo_info["avg_speed"]:.0f}rpm'
            )
        elif max_diff > 0.5:  # 中等运动，简化日志
            self.get_logger().info(
                f'🔄 同步: {algo_info["active_axes"]}轴, {algo_info["avg_speed"]:.0f}rpm'
            )
        # 小运动无日志输出

    def _publish_idle_js(self) -> None:
        if self.js_pub is None:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        # 将当前"输出端角(度)"转换为弧度发布
        js.position = [v * math.pi / 180.0 for v in self._current_out_deg]
        self.js_pub.publish(js)

    def _read_hw_state(self) -> None:
        """定时读取真机状态并更新当前角度"""
        try:
            if self._sdk:
                positions = self._sdk.read_positions_output()
                # 过滤掉nan值
                valid_positions = []
                for i, pos in enumerate(positions):
                    if not math.isnan(pos):
                        valid_positions.append(pos)
                    else:
                        valid_positions.append(self._current_out_deg[i])  # 保持上次有效值
                self._current_out_deg = valid_positions
            elif self.hw_ready and self.controllers:
                # 使用原始SDK读取
                out_deg = []
                for i, ctrl in enumerate(self.controllers):
                    try:
                        pos_motor = float(ctrl.read_parameters.get_position())
                        # 转换为输出端角
                        if self.send_space == 'motor':
                            pos_out = (pos_motor - self.zero_offset_deg[i]) / (self.gear_ratio[i] * self.sign[i])
                        else:
                            pos_out = pos_motor
                        out_deg.append(pos_out)
                    except Exception:
                        out_deg.append(self._current_out_deg[i])  # 读取失败时保持上次值
                self._current_out_deg = out_deg
        except Exception as e:
            self.get_logger().debug(f'读取硬件状态失败: {e}')

    # ------- 映射与控制初始化 -------
    def _load_mapping(self, paths: dict) -> dict:
        mp = Path(self.mapping_path_param)
        candidate = None
        if mp.is_file():
            candidate = mp
        else:
            # 尝试相对 repo 根目录
            mp2 = (paths['repo_root'] / self.mapping_path_param)
            if mp2.is_file():
                candidate = mp2
            else:
                # 默认同仓根的 arm-ros2/mapping.yaml（兄弟目录）
                mp3 = paths['repo_root'] / 'arm-ros2' / 'mapping.yaml'
                if mp3.is_file():
                    candidate = mp3
                else:
                    # 兼容：仓库内放在 Horizon_ARM_ROS2/arm-ros2/mapping.yaml
                    mp4 = paths['horizon_arm_root'] / 'arm-ros2' / 'mapping.yaml'
                    if mp4.is_file():
                        candidate = mp4

        if candidate is None:
            raise FileNotFoundError(f'找不到mapping.yaml，尝试路径: {self.mapping_path_param}')
        with open(candidate, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise RuntimeError('mapping.yaml 格式错误')
        return data

    def _init_controllers(self) -> List[object]:
        # 选择接口类型
        kwargs = {}
        interface_type = 'slcan'
        if self.com_port.startswith('can'):
            interface_type = 'socketcan'
            kwargs['channel'] = self.com_port
            kwargs['bitrate'] = self.baudrate
        else:
            kwargs['port'] = self.com_port
            kwargs['baudrate'] = self.baudrate

        ctrls: List[object] = []
        for mid in self.motor_ids:
            ctrl = self.ControllerClass(motor_id=mid, interface_type=interface_type, shared_interface=True, **kwargs)
            ctrl.connect(motor_id=mid)
            try:
                ctrl.control_actions.enable(multi_sync=False)
            except Exception as e:
                self.get_logger().warning(f'电机{mid} 使能失败: {e}')
            ctrls.append(ctrl)
            time.sleep(0.05)
        self.get_logger().info(f'已连接并尝试使能电机: {self.motor_ids} via {interface_type} ({self.com_port})')
        return ctrls

    def _init_sdk(self) -> None:
        """使用SimpleArmSDK初始化连接"""
        self._sdk = self.SimpleArmSDK(
            motor_ids=self.motor_ids,
            port_or_channel=self.com_port,
            bitrate=self.baudrate,
            sign=self.sign,
            gear_ratio=self.gear_ratio,
            zero_offset_deg=self.zero_offset_deg,
            send_space=self.send_space,
        )
        self._sdk.connect(enable=True)
        self.hw_ready = True
        self.get_logger().info(f'已通过SimpleArmSDK连接 {len(self.motor_ids)} 个电机 @ {self.com_port}')

    # ------- 订阅回调与流式下发 -------
    def _on_display_traj(self, msg: DisplayTrajectory):
        if not msg.trajectory:
            return
        jt: JointTrajectory = msg.trajectory[0].joint_trajectory
        if not jt.points:
            return
        
        # 缓存轨迹
        self._last_planned_jt = jt
        self.get_logger().info(
            f'Received trajectory: joints={jt.joint_names}, points={len(jt.points)}'
        )
        
        # 自动执行轨迹，仅用于RViz可视化（不使用同步算法）
        self.get_logger().info('开始执行轨迹 - RViz可视化模式')
        self._stream_trajectory(jt)

    def _stream_trajectory_with_sync(self, jt: JointTrajectory):
        """
        使用同步算法执行轨迹 - 通过内部调用servo_target逻辑
        """
        # 将 MoveIt 的关节名映射到本节点顺序
        name_to_index = {n: i for i, n in enumerate(jt.joint_names)}
        idx_map = [name_to_index.get(n, -1) for n in self.joint_names]

        # 构造时间与位置序列
        points: List[JointTrajectoryPoint] = list(jt.points)
        if self.skip_first_point and len(points) > 1:
            points = points[1:]

        if not points:
            return

        # 原始时间戳（秒）
        orig_ts: List[float] = [float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9 for p in points]
        # 保证单调
        base_t0 = orig_ts[0]
        orig_ts = [max(0.0, t - base_t0) for t in orig_ts]

        # 原始位置（弧度）按本节点关节顺序提取
        def extract_positions_rad(p: JointTrajectoryPoint) -> List[float]:
            out: List[float] = []
            for j, m in enumerate(idx_map):
                if m >= 0 and m < len(p.positions):
                    out.append(float(p.positions[m]))
                else:
                    out.append(0.0)
            return out

        orig_pos_rad: List[List[float]] = [extract_positions_rad(p) for p in points]

        # 重采样为目标频率
        if self.resample_rate_hz > 0:
            total_time = orig_ts[-1]
            dt = 1.0 / self.resample_rate_hz
            ts = []
            pos_rad_seq = []
            
            t = 0.0
            while t <= total_time:
                # 插值获取当前时刻的位置
                interp_pos = []
                for joint_idx in range(len(self.joint_names)):
                    # 对每个关节进行线性插值
                    joint_positions = [pos[joint_idx] for pos in orig_pos_rad]
                    interp_val = float(np.interp(t, orig_ts, joint_positions))
                    interp_pos.append(interp_val)
                
                ts.append(t)
                pos_rad_seq.append(interp_pos)
                t += dt
        else:
            ts = orig_ts
            pos_rad_seq = orig_pos_rad

        # 开始执行轨迹，使用同步算法
        start_time = time.time()
        self.get_logger().info(f'开始同步轨迹执行，点数: {len(ts)}')

        for seq_index, (t_rel, pos_rad) in enumerate(zip(ts, pos_rad_seq)):
            if self.stop_flag.is_set():
                return
            
            # 等待到正确的时间点
            wait = max(0.0, (start_time + t_rel) - time.time())
            if wait > 0:
                time.sleep(wait)

            # 转换为度并调用同步算法
            out_deg = [v * 180.0 / math.pi for v in pos_rad]
            
            # 限位（输出端）
            for i in range(min(6, len(out_deg))):
                lo, hi = self.pos_limit_deg[i]
                out_deg[i] = max(lo, min(hi, out_deg[i]))

            # 创建内部消息，触发同步算法
            internal_msg = Float64MultiArray()
            internal_msg.data = out_deg
            
            # 直接调用servo_target的处理逻辑（使用同步算法）
            self._process_servo_target_internal(internal_msg)

            # 发布joint_states用于可视化
            if self.js_pub is not None:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self.joint_names
                js.position = [v * math.pi / 180.0 for v in out_deg]
                self.js_pub.publish(js)

            if (seq_index % 10) == 0:
                self.get_logger().info(f'同步轨迹进度: {seq_index+1}/{len(ts)} ({(seq_index/len(ts)*100):.1f}%)')

        self.get_logger().info('✅ 同步轨迹执行完成')

    def _process_servo_target_internal(self, msg: Float64MultiArray):
        """内部调用servo_target逻辑，使用同步算法"""
        values = list(msg.data)
        if len(values) != 6:
            return
            
        # 限位裁剪，并作为当前输出端角
        for i in range(6):
            lo, hi = self.pos_limit_deg[i]
            values[i] = max(lo, min(hi, float(values[i])))
        self._current_out_deg = values[:]

        # === 应用同步运动算法 - 统一格式化层 ===
        movement_time, sync_speeds_rpm = self._calculate_sync_movement_params(values)
        
        # 输出端角 -> 下发角
        if self.send_space == 'motor':
            motor_deg = [self.sign[i] * values[i] * self.gear_ratio[i] + self.zero_offset_deg[i] for i in range(6)]
        else:
            motor_deg = values[:]

        # 使用同步算法计算的速度
        rpm_each = sync_speeds_rpm

        if self._sdk:
            # 使用SimpleArmSDK下发 - 应用同步速度
            try:
                # 取最大同步速度作为SDK的统一速度参数
                unified_speed = max(sync_speeds_rpm)
                self._sdk.goto_output_abs(
                    values,  # 直接传递输出端角
                    max_speed_rpm=unified_speed,
                    acc=int(self.accel_rpm_s),
                    dec=int(self.decel_rpm_s)
                )
            except Exception as e:
                self.get_logger().warning(f'SimpleArmSDK 同步运动失败: {e}')
        elif self.hw_ready and self.controllers:
            for i, ctrl in enumerate(self.controllers):
                try:
                    if self.stream_use_direct:
                        ctrl.control_actions.move_to_position(
                            position=float(motor_deg[i]),
                            speed=float(rpm_each[i]),
                            is_absolute=True,
                            multi_sync=True,
                            timeout=0.5,
                        )
                    else:
                        ctrl.control_actions.move_to_position_trapezoid(
                            position=float(motor_deg[i]),
                            max_speed=float(rpm_each[i]),
                            acceleration=int(self.accel_rpm_s),
                            deceleration=int(self.decel_rpm_s),
                            is_absolute=True,
                            multi_sync=True,
                        )
                except Exception as e:
                    self.get_logger().warning(f'控制器 {i+1} 指令失败: {e}')

    def _stream_trajectory(self, jt: JointTrajectory):
        # 将 MoveIt 的关节名映射到本节点顺序
        name_to_index = {n: i for i, n in enumerate(jt.joint_names)}
        idx_map = [name_to_index.get(n, -1) for n in self.joint_names]

        # 构造时间与位置序列
        points: List[JointTrajectoryPoint] = list(jt.points)
        if self.skip_first_point and len(points) > 1:
            points = points[1:]

        if not points:
            return

        # 原始时间戳（秒）
        orig_ts: List[float] = [float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9 for p in points]
        # 保证单调
        base_t0 = orig_ts[0]
        orig_ts = [max(0.0, t - base_t0) for t in orig_ts]

        # 原始位置（弧度）按本节点关节顺序提取
        def extract_positions_rad(p: JointTrajectoryPoint) -> List[float]:
            out: List[float] = []
            for j, m in enumerate(idx_map):
                if m >= 0 and m < len(p.positions):
                    out.append(float(p.positions[m]))
                else:
                    out.append(0.0)
            return out

        orig_pos_rad: List[List[float]] = [extract_positions_rad(p) for p in points]

        # 重采样到固定频率
        ts: List[float]
        pos_rad_seq: List[List[float]]
        if self.resample_rate_hz and self.resample_rate_hz > 0.0:
            period = 1.0 / self.resample_rate_hz
            total = orig_ts[-1]
            if total <= 0.0:
                # 轨迹时间全为0，按点序列均匀采样
                n = len(orig_pos_rad)
                ts = [i * period for i in range(n)]
                pos_rad_seq = orig_pos_rad
            else:
                n = int(math.ceil(total / period)) + 1
                ts = [i * period for i in range(n)]
                pos_rad_seq = []
                # 简单零阶保持
                k = 0
                for t in ts:
                    while k + 1 < len(orig_ts) and orig_ts[k + 1] <= t:
                        k += 1
                    pos_rad_seq.append(orig_pos_rad[k])
        else:
            ts = orig_ts
            pos_rad_seq = orig_pos_rad

        start_time = time.time()
        last_values_deg: List[float] = []

        for seq_index, (t_rel, pos_rad) in enumerate(zip(ts, pos_rad_seq)):
            if self.stop_flag.is_set():
                return
            wait = max(0.0, (start_time + t_rel) - time.time())
            if wait > 0:
                time.sleep(wait)

            # rad -> deg（输出端）
            out_deg = [v * 180.0 / math.pi for v in pos_rad]
            # 限位（输出端）
            for i in range(min(6, len(out_deg))):
                lo, hi = self.pos_limit_deg[i]
                out_deg[i] = max(lo, min(hi, out_deg[i]))

            # 记录当前输出端角（供空闲态JointState定时发布）
            self._current_out_deg = out_deg[:]

            # 输出端角 -> 下发角（电机或输出端）
            if self.send_space == 'motor':
                motor_deg = [self.sign[i] * out_deg[i] * self.gear_ratio[i] + self.zero_offset_deg[i] for i in range(6)]
            else:
                motor_deg = out_deg[:]  # 直接输出端

            # 速度选择（RPM）
            if self.stream_speed_rpm and self.stream_speed_rpm > 0:
                rpm_each = [self.stream_speed_rpm for _ in range(6)]
            else:
                # 将关节速度上限(°/s)换算为电机RPM
                rpm_each = [
                    (self.vel_limit_deg_s[i] * (self.gear_ratio[i] if self.send_space == 'motor' else 1.0)) / 6.0
                    for i in range(6)
                ]

            # 发送到硬件（若就绪）；否则仅发布 JointState
            if self._sdk:
                # 使用SimpleArmSDK下发
                try:
                    self._sdk.goto_output_abs(
                        out_deg,  # 直接传递输出端角
                        max_speed_rpm=self.stream_speed_rpm if self.stream_speed_rpm > 0 else max(rpm_each),
                        acc=int(self.accel_rpm_s),
                        dec=int(self.decel_rpm_s)
                    )
                except Exception as e:
                    self.get_logger().warning(f'SimpleArmSDK 指令发送失败: {e}')
            elif self.hw_ready and self.controllers:
                for i, ctrl in enumerate(self.controllers):
                    try:
                        if self.stream_use_direct:
                            ctrl.control_actions.move_to_position(
                                position=float(motor_deg[i]),
                                speed=float(rpm_each[i]),
                                is_absolute=True,
                                multi_sync=True,
                                timeout=1.5,
                            )
                        else:
                            ctrl.control_actions.move_to_position_trapezoid(
                                position=float(motor_deg[i]),
                                max_speed=float(rpm_each[i]),
                                acceleration=int(self.accel_rpm_s),
                                deceleration=int(self.decel_rpm_s),
                                is_absolute=True,
                                multi_sync=True,
                                timeout=1.5,
                            )
                    except Exception as e:
                        self.get_logger().warning(f'轴{i+1} 指令发送失败: {e}')

                # 触发同步运动
                try:
                    if self.controllers:
                        self.controllers[0].control_actions.sync_motion()
                except Exception as e:
                    self.get_logger().warning(f'多机同步触发失败: {e}')

            # JointState 回显（与空闲定时发布并存，保证轨迹期间也有较新状态）
            if self.js_pub is not None:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = self.joint_names
                js.position = [v * math.pi / 180.0 for v in out_deg]
                self.js_pub.publish(js)

            last_values_deg = out_deg
            # 简化轨迹日志，避免刷屏
            if (seq_index % 25) == 0 or seq_index == len(ts) - 1:
                progress = (seq_index + 1) / len(ts) * 100
                self.get_logger().info(f'轨迹执行: {progress:.0f}% ({seq_index+1}/{len(ts)})')

        # 保持末端状态一段时间
        if self.hold_final_seconds > 0 and last_values_deg:
            end_time = time.time() + self.hold_final_seconds
            while time.time() < end_time and not self.stop_flag.is_set():
                if self.js_pub is not None:
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = self.joint_names
                    js.position = [v * math.pi / 180.0 for v in last_values_deg]
                    self.js_pub.publish(js)
                time.sleep(0.05)

    def _on_servo_target(self, msg: Float64MultiArray) -> None:
        # 期望长度6，单位deg；仅执行，不缓存
        try:
            # 若尚未连接硬件，则在第一次收到实时命令时按需连接（忽略 hardware_mode 限制）
            if not (self.hw_ready or self._sdk):
                try:
                    if self.SimpleArmSDK is not None:
                        self._init_sdk()
                    else:
                        self.controllers = self._init_controllers()
                        self.hw_ready = (len(self.controllers) == 6)
                    self.get_logger().info('实时话题触发：已连接硬件用于实时跟随')
                except Exception as e:
                    self.get_logger().warning(f'实时接收时硬件连接失败: {e}')
            values = list(msg.data)
            if len(values) != 6:
                return
            # 限位裁剪，并作为当前输出端角
            for i in range(6):
                lo, hi = self.pos_limit_deg[i]
                values[i] = max(lo, min(hi, float(values[i])))
            self._current_out_deg = values[:]

            # 输出端角 -> 下发角
            if self.send_space == 'motor':
                motor_deg = [self.sign[i] * values[i] * self.gear_ratio[i] + self.zero_offset_deg[i] for i in range(6)]
            else:
                motor_deg = values[:]

            # === 应用独立同步算法模块 ===
            movement_time, sync_speeds_rpm = self._apply_sync_algorithm(values)
            rpm_each = sync_speeds_rpm

            if self._sdk:
                # 使用SimpleArmSDK下发 - 应用同步速度
                try:
                    # 取最大同步速度作为SDK的统一速度参数
                    unified_speed = max(sync_speeds_rpm)
                    self._sdk.goto_output_abs(
                        values,  # 直接传递输出端角
                        max_speed_rpm=unified_speed,
                        acc=int(self.accel_rpm_s),
                        dec=int(self.decel_rpm_s)
                    )
                    self.get_logger().info(f'SDK下发同步运动: 统一速度{unified_speed:.2f}rpm')
                except Exception as e:
                    self.get_logger().warning(f'SimpleArmSDK 同步运动失败: {e}')
            elif self.hw_ready and self.controllers:
                for i, ctrl in enumerate(self.controllers):
                    try:
                        if self.stream_use_direct:
                            ctrl.control_actions.move_to_position(
                                position=float(motor_deg[i]),
                                speed=float(rpm_each[i]),
                                is_absolute=True,
                                multi_sync=True,
                                timeout=0.5,
                            )
                        else:
                            ctrl.control_actions.move_to_position_trapezoid(
                                position=float(motor_deg[i]),
                                max_speed=float(rpm_each[i]),
                                acceleration=int(self.accel_rpm_s),
                                deceleration=int(self.decel_rpm_s),
                                is_absolute=True,
                                multi_sync=True,
                                timeout=0.5,
                            )
                    except Exception as e:
                        self.get_logger().warning(f'轴{i+1} 实时跟随失败: {e}')
                try:
                    if self.controllers:
                        self.controllers[0].control_actions.sync_motion()
                except Exception as e:
                    self.get_logger().warning(f'实时同步失败: {e}')
        except Exception:
            # 忽略单帧异常，保持循环
            return


def main():
    rclpy.init()
    node = TrajectoryStreamSdkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_flag.set()
        # 关闭硬件连接
        if node._sdk:
            node._sdk.close(disable=True)
        elif node.controllers:
            for ctrl in node.controllers:
                try:
                    ctrl.control_actions.disable()
                    ctrl.disconnect()
                except Exception:
                    pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()