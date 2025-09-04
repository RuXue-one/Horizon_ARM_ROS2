#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
同步运动算法模块
负责多轴机械臂的协调运动和速度同步计算
"""

import math
from typing import List, Tuple, Optional
import time


class SyncMovementAlgorithm:
    """同步运动算法类"""
    
    def __init__(self, reducer_ratios: List[float], axis_performance_factors: Optional[List[float]] = None):
        """
        初始化同步算法
        
        Args:
            reducer_ratios: 各轴减速比 [J1, J2, J3, J4, J5, J6]
            axis_performance_factors: 各轴性能系数，用于补偿硬件差异
        """
        self.reducer_ratios = reducer_ratios
        self.axis_performance_factors = axis_performance_factors or [1.0, 0.8, 0.9, 0.6, 1.1, 1.2]
        
        # 运动时间控制
        self.min_movement_time = 0.3  # 最小运动时间(秒)
        self.max_movement_time = 10.0  # 最大运动时间(秒)
        self.min_axis_speed_rpm = 3.0  # 最小轴速度
        
        # 速度平滑参数
        self.previous_speeds = [0.0] * 6
        self.speed_filter_alpha = 0.3  # 低通滤波系数
        self.max_speed_change_rpm = 50.0  # 最大速度变化率
        
        # 当前状态
        self.current_angles = [0.0] * 6
        
    def calculate_sync_speeds(self, target_angles: List[float], 
                            current_angles: Optional[List[float]] = None) -> Tuple[float, List[float]]:
        """
        计算同步运动速度
        
        Args:
            target_angles: 目标角度列表(度)
            current_angles: 当前角度列表(度)，如果为None则使用内部状态
            
        Returns:
            Tuple[运动时间(秒), 各轴速度列表(RPM)]
        """
        if current_angles is None:
            current_angles = self.current_angles
        else:
            self.current_angles = current_angles[:]
        
        # 计算各轴角度差
        angle_diffs = [abs(target_angles[i] - current_angles[i]) for i in range(6)]
        max_angle_diff = max(angle_diffs)
        
        # 动态计算运动时间
        movement_time = self._calculate_movement_time(angle_diffs, max_angle_diff)
        
        # 计算各轴同步速度
        sync_speeds_rpm = []
        for i in range(6):
            if angle_diffs[i] > 0.01:  # 有意义的运动
                # 计算输出端角速度
                output_speed = angle_diffs[i] / movement_time
                
                # 转换为电机端速度（考虑减速比）
                motor_speed = output_speed * self.reducer_ratios[i]
                
                # 转换为RPM（度/秒 -> RPM: 除以6）
                raw_speed = motor_speed / 6.0
                
                # 应用轴性能系数
                adjusted_speed = raw_speed / self.axis_performance_factors[i]
                
                # 速度平滑滤波
                filtered_speed = (self.speed_filter_alpha * adjusted_speed + 
                                (1 - self.speed_filter_alpha) * self.previous_speeds[i])
                
                # 限制速度变化率
                speed_diff = abs(filtered_speed - self.previous_speeds[i])
                if speed_diff > self.max_speed_change_rpm:
                    if filtered_speed > self.previous_speeds[i]:
                        filtered_speed = self.previous_speeds[i] + self.max_speed_change_rpm
                    else:
                        filtered_speed = self.previous_speeds[i] - self.max_speed_change_rpm
                
                # 限制速度范围 (提高上限支持更快运动)
                final_speed = max(self.min_axis_speed_rpm, min(filtered_speed, 200.0))
                sync_speeds_rpm.append(final_speed)
                self.previous_speeds[i] = final_speed
            else:
                # 几乎不动的关节
                sync_speeds_rpm.append(self.min_axis_speed_rpm)
                self.previous_speeds[i] = self.min_axis_speed_rpm
        
        # 第四轴补偿（如果第四轴速度过低）
        if len(sync_speeds_rpm) > 3 and sync_speeds_rpm[3] < 8.0:
            boost_factor = min(2.0, 8.0 / sync_speeds_rpm[3])
            sync_speeds_rpm = [min(s * boost_factor, 200.0) for s in sync_speeds_rpm]
        
        return movement_time, sync_speeds_rpm
    
    def _calculate_movement_time(self, angle_diffs: List[float], max_angle_diff: float) -> float:
        """
        动态计算运动时间
        
        Args:
            angle_diffs: 各轴角度差列表
            max_angle_diff: 最大角度差
            
        Returns:
            运动时间(秒)
        """
        if max_angle_diff < 0.1:
            return self.min_movement_time
        
        # 基于最大角度差的基础时间
        base_time = max_angle_diff / 60.0  # 基础速度60度/秒
        
        # 考虑多轴协调的时间修正
        active_axes_count = sum(1 for diff in angle_diffs if diff > 0.5)
        if active_axes_count > 3:
            base_time *= 1.2  # 多轴运动需要更多时间
        
        # 限制在合理范围内
        movement_time = max(self.min_movement_time, min(base_time, self.max_movement_time))
        
        return movement_time
    
    def get_algorithm_info(self, target_angles: List[float], 
                          current_angles: Optional[List[float]] = None) -> dict:
        """
        获取算法计算详情（用于调试和日志）
        
        Args:
            target_angles: 目标角度列表
            current_angles: 当前角度列表
            
        Returns:
            包含算法计算详情的字典
        """
        if current_angles is None:
            current_angles = self.current_angles
        
        angle_diffs = [abs(target_angles[i] - current_angles[i]) for i in range(6)]
        max_angle_diff = max(angle_diffs)
        movement_time, sync_speeds = self.calculate_sync_speeds(target_angles, current_angles)
        
        return {
            'max_angle_diff': max_angle_diff,
            'movement_time': movement_time,
            'angle_diffs': angle_diffs,
            'sync_speeds': sync_speeds,
            'active_axes': sum(1 for diff in angle_diffs if diff > 0.1),
            'avg_speed': sum(sync_speeds) / len(sync_speeds)
        }
    
    def reset_state(self):
        """重置算法状态"""
        self.previous_speeds = [0.0] * 6
        self.current_angles = [0.0] * 6
    
    def update_current_position(self, angles: List[float]):
        """更新当前位置"""
        self.current_angles = angles[:]


# 工厂函数
def create_sync_algorithm(reducer_ratios: List[float]) -> SyncMovementAlgorithm:
    """
    创建同步算法实例的工厂函数
    
    Args:
        reducer_ratios: 减速比配置
        
    Returns:
        同步算法实例
    """
    return SyncMovementAlgorithm(reducer_ratios)
