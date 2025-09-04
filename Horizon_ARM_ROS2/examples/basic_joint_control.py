#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基础关节控制示例
演示如何通过话题控制单个关节和多关节协调运动

使用前提：
1. 启动 trajectory_stream_sdk_driver.py 节点
2. 确保硬件已连接（或使用仿真模式）
3. 启用实时跟随功能

运行命令：
  python3 examples/basic_joint_control.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math

class BasicJointController(Node):
    def __init__(self):
        super().__init__('basic_joint_controller')
        
        # 创建发布者，向机械臂发送目标位置
        self.target_pub = self.create_publisher(
            Float64MultiArray, 
            '/horizon_arm/servo_target', 
            10
        )
        
        # 订阅关节状态以验证连接
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10
        )
        
        self.joint_connected = False
        self.current_positions = [0.0] * 6
        
        self.get_logger().info('基础关节控制示例已启动')
        self.get_logger().info('等待 /joint_states 话题连接...')
        
    def _on_joint_states(self, msg):
        """接收关节状态回调"""
        if not self.joint_connected:
            self.joint_connected = True
            self.get_logger().info('✓ /joint_states 话题已连接')
        
        if len(msg.position) >= 6:
            # 转换弧度到度数并更新当前位置
            self.current_positions = [math.degrees(pos) for pos in msg.position[:6]]
        
    def move_single_joint(self, joint_index, target_angle, timeout=10.0):
        """控制单个关节运动"""
        if not self.joint_connected:
            self.get_logger().error('❌ /joint_states 话题未连接，请先启动 trajectory_stream_sdk_driver 节点')
            return False
            
        msg = Float64MultiArray()
        # 使用当前位置作为基础，只改变指定关节
        msg.data = self.current_positions[:]
        # 设置指定关节的目标角度
        msg.data[joint_index] = target_angle
        
        self.target_pub.publish(msg)
        self.get_logger().info(f'📤 关节{joint_index+1}移动到{target_angle}度 (当前: {self.current_positions[joint_index]:.1f}度)')
        
        # 等待到位验证
        return self._wait_for_position(joint_index, target_angle, timeout)
    
    def _wait_for_position(self, joint_index, target_angle, timeout, tolerance=2.0):
        """等待关节到达目标位置"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if abs(self.current_positions[joint_index] - target_angle) <= tolerance:
                self.get_logger().info(f'✅ 关节{joint_index+1}已到位 (误差: {abs(self.current_positions[joint_index] - target_angle):.1f}度)')
                return True
        
        self.get_logger().warning(f'⚠️ 关节{joint_index+1}超时未到位 (当前: {self.current_positions[joint_index]:.1f}度, 目标: {target_angle}度)')
        return False
        
    def move_all_joints(self, joint_angles, timeout=15.0):
        """控制所有关节协调运动"""
        if not self.joint_connected:
            self.get_logger().error('❌ /joint_states 话题未连接，请先启动 trajectory_stream_sdk_driver 节点')
            return False
            
        if len(joint_angles) != 6:
            self.get_logger().error('需要提供6个关节角度值')
            return False
            
        msg = Float64MultiArray()
        msg.data = joint_angles[:]
        
        self.target_pub.publish(msg)
        self.get_logger().info(f'📤 所有关节移动到: {[f"{angle:.1f}" for angle in joint_angles]}')
        
        # 等待所有关节到位
        return self._wait_for_all_positions(joint_angles, timeout)
    
    def _wait_for_all_positions(self, target_angles, timeout, tolerance=2.0):
        """等待所有关节到达目标位置"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 检查所有关节是否都到位
            all_in_position = True
            for i in range(6):
                if abs(self.current_positions[i] - target_angles[i]) > tolerance:
                    all_in_position = False
                    break
            
            if all_in_position:
                self.get_logger().info('✅ 所有关节已到位')
                return True
        
        # 报告哪些关节未到位
        out_of_position = []
        for i in range(6):
            error = abs(self.current_positions[i] - target_angles[i])
            if error > tolerance:
                out_of_position.append(f'关节{i+1}(误差{error:.1f}度)')
        
        self.get_logger().warning(f'⚠️ 超时未完全到位: {", ".join(out_of_position)}')
        return False
        
    def demo_sequence(self):
        """演示动作序列"""
        if not self.joint_connected:
            self.get_logger().error('❌ 无法开始演示：关节状态未连接')
            return
            
        self.get_logger().info('🚀 开始演示序列...')
        
        # 等待系统稳定
        time.sleep(1)
        
        # 1. 回到零位
        self.get_logger().info('1️⃣ 回到零位')
        if not self.move_all_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
            return
        time.sleep(3)
        
        # 2. 单关节测试
        self.get_logger().info('2️⃣ 单关节测试')
        for i in range(6):
            if not self.move_single_joint(i, 30.0):  # 每个关节转30度
                return
            time.sleep(1)  # 减少等待时间，因为已有到位验证
            if not self.move_single_joint(i, 0.0):   # 回到零位
                return
            time.sleep(1)
        
        # 3. 多关节协调运动
        self.get_logger().info('3️⃣ 多关节协调运动')
        poses = [
            [30.0, 30.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 45.0, -30.0, 0.0, 0.0, 0.0],
            [-30.0, 0.0, 45.0, 30.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 30.0, 45.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 最后回零
        ]
        
        for i, pose in enumerate(poses):
            self.get_logger().info(f'   姿态 {i+1}/{len(poses)}')
            if not self.move_all_joints(pose):
                return
            time.sleep(1)  # 减少等待时间，因为已有到位验证
        
        self.get_logger().info('✅ 演示序列完成')

def main():
    rclpy.init()
    
    controller = BasicJointController()
    
    try:
        # 等待连接建立
        controller.get_logger().info('⏳ 等待系统连接...')
        start_time = time.time()
        
        # 最多等待10秒建立连接
        while not controller.joint_connected and (time.time() - start_time < 10):
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        if controller.joint_connected:
            controller.get_logger().info('✅ 系统连接成功，开始运行演示')
            # 运行演示序列
            controller.demo_sequence()
            controller.get_logger().info('🏁 演示完成，程序将在5秒后退出')
            time.sleep(5)
        else:
            controller.get_logger().error('❌ 连接超时！请确保以下服务正在运行：')
            controller.get_logger().error('   1. ros2 launch horizon_arm_bridge bringup_sdk_ui.launch.py')
            controller.get_logger().error('   2. 或者 ros2 run horizon_arm_bridge trajectory_stream_sdk_driver.py')
        
    except KeyboardInterrupt:
        controller.get_logger().info('👋 用户中断程序')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
