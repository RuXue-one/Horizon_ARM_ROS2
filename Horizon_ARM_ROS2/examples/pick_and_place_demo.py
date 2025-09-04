#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取放置演示
模拟简单的抓取和放置动作序列
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math

class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # 发布目标位置
        self.target_pub = self.create_publisher(
            Float64MultiArray,
            '/horizon_arm/servo_target',
            10
        )
        
        # 订阅关节状态以验证连接和到位
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10
        )
        
        self.joint_connected = False
        self.current_positions = [0.0] * 6
        
        # 定义预设位置
        self.positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'observe': [0.0, -30.0, 45.0, 0.0, -15.0, 0.0],
            'pick_approach': [45.0, 15.0, -30.0, 0.0, 45.0, 0.0],
            'pick_grasp': [45.0, 30.0, -45.0, 0.0, 60.0, 0.0],
            'pick_lift': [45.0, 15.0, -30.0, 0.0, 45.0, 0.0],
            'place_approach': [-45.0, 15.0, -30.0, 0.0, 45.0, 0.0],
            'place_drop': [-45.0, 30.0, -45.0, 0.0, 60.0, 0.0],
            'place_retreat': [-45.0, 15.0, -30.0, 0.0, 45.0, 0.0],
        }
        
        self.get_logger().info('抓取放置演示已启动')
        self.get_logger().info('等待 /joint_states 话题连接...')
    
    def _on_joint_states(self, msg):
        """接收关节状态回调"""
        if not self.joint_connected:
            self.joint_connected = True
            self.get_logger().info('✅ /joint_states 话题已连接')
        
        if len(msg.position) >= 6:
            # 转换弧度到度数并更新当前位置
            self.current_positions = [math.degrees(pos) for pos in msg.position[:6]]
        
    def move_to_position(self, position_name, timeout=15.0):
        """移动到指定位置并等待到位"""
        if not self.joint_connected:
            self.get_logger().error('❌ /joint_states 话题未连接，请先启动 trajectory_stream_sdk_driver 节点')
            return False
            
        if position_name not in self.positions:
            self.get_logger().error(f'未知位置: {position_name}')
            return False
            
        angles = self.positions[position_name]
        msg = Float64MultiArray()
        msg.data = angles
        
        self.target_pub.publish(msg)
        self.get_logger().info(f'📤 移动到位置: {position_name} {angles}')
        
        # 等待到位验证
        return self._wait_for_all_positions(angles, timeout)
    
    def _wait_for_all_positions(self, target_angles, timeout, tolerance=3.0):
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
                self.get_logger().info('✅ 已到达目标位置')
                return True
        
        # 报告哪些关节未到位
        out_of_position = []
        for i in range(6):
            error = abs(self.current_positions[i] - target_angles[i])
            if error > tolerance:
                out_of_position.append(f'关节{i+1}(误差{error:.1f}度)')
        
        self.get_logger().warning(f'⚠️ 超时未完全到位: {", ".join(out_of_position)}')
        return False
        
    def simulate_gripper_action(self, action):
        """模拟夹爪动作"""
        if action == 'close':
            self.get_logger().info('🤏 夹爪闭合 - 抓取物体')
        elif action == 'open':
            self.get_logger().info('✋ 夹爪张开 - 释放物体')
        
        # 模拟夹爪动作时间
        time.sleep(1.0)
        
    def pick_and_place_sequence(self):
        """执行完整的抓取放置序列"""
        if not self.joint_connected:
            self.get_logger().error('❌ 无法开始演示：关节状态未连接')
            return False
            
        self.get_logger().info('🚀 开始抓取放置演示...')
        
        # 1. 初始化到Home位置
        self.get_logger().info('1️⃣ 初始化位置')
        if not self.move_to_position('home'):
            return False
        
        # 2. 移动到观察位置
        self.get_logger().info('2️⃣ 移动到观察位置')
        if not self.move_to_position('observe'):
            return False
        
        # 3. 接近抓取位置
        self.get_logger().info('3️⃣ 接近抓取位置')
        if not self.move_to_position('pick_approach'):
            return False
        
        # 4. 下降到抓取位置
        self.get_logger().info('4️⃣ 下降到抓取位置')
        if not self.move_to_position('pick_grasp'):
            return False
        
        # 5. 执行抓取
        self.get_logger().info('5️⃣ 执行抓取')
        self.simulate_gripper_action('close')
        
        # 6. 抬升物体
        self.get_logger().info('6️⃣ 抬升物体')
        if not self.move_to_position('pick_lift'):
            return False
        
        # 7. 移动到放置位置上方
        self.get_logger().info('7️⃣ 移动到放置位置上方')
        if not self.move_to_position('place_approach'):
            return False
        
        # 8. 下降到放置位置
        self.get_logger().info('8️⃣ 下降到放置位置')
        if not self.move_to_position('place_drop'):
            return False
        
        # 9. 释放物体
        self.get_logger().info('9️⃣ 释放物体')
        self.simulate_gripper_action('open')
        
        # 10. 撤离
        self.get_logger().info('🔟 撤离放置位置')
        if not self.move_to_position('place_retreat'):
            return False
        
        # 11. 返回Home位置
        self.get_logger().info('1️⃣1️⃣ 返回初始位置')
        if not self.move_to_position('home'):
            return False
        
        self.get_logger().info('✅ 抓取放置演示完成!')
        return True
        
    def interactive_demo(self):
        """交互式演示，用户可以选择动作"""
        self.get_logger().info('交互式抓取放置演示')
        self.get_logger().info('可用位置:')
        for name in self.positions.keys():
            self.get_logger().info(f'  - {name}')
            
        # 在实际应用中，这里可以添加键盘输入处理
        # 现在直接运行完整序列
        self.pick_and_place_sequence()

def main():
    rclpy.init()
    
    demo = PickAndPlaceDemo()
    
    try:
        # 等待连接建立
        demo.get_logger().info('⏳ 等待系统连接...')
        start_time = time.time()
        
        # 最多等待10秒建立连接
        while not demo.joint_connected and (time.time() - start_time < 10):
            rclpy.spin_once(demo, timeout_sec=0.1)
        
        if demo.joint_connected:
            demo.get_logger().info('✅ 系统连接成功，开始抓取放置演示')
            # 运行演示
            demo.interactive_demo()
            demo.get_logger().info('🏁 演示完成，程序将在3秒后退出')
            time.sleep(3)
        else:
            demo.get_logger().error('❌ 连接超时！请确保以下服务正在运行：')
            demo.get_logger().error('   1. ros2 launch horizon_arm_bridge bringup_sdk_ui.launch.py')
            demo.get_logger().error('   2. 或者 ros2 run horizon_arm_bridge trajectory_stream_sdk_driver.py')
        
    except KeyboardInterrupt:
        demo.get_logger().info('👋 用户中断程序')
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
