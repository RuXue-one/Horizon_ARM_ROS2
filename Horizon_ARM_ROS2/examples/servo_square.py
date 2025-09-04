#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
方形轨迹实时控制示例
通过 /horizon_arm/servo_target 话题发布目标角度（度），让机械臂画方形轨迹

默认运行时间：30秒后自动停止
使用 Ctrl+C 可随时手动停止
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ServoSquare(Node):
    def __init__(self, duration_seconds=30):
        super().__init__('servo_square_demo')
        self.pub = self.create_publisher(Float64MultiArray, '/horizon_arm/servo_target', 10)
        self.timer = self.create_timer(0.02, self.on_timer)  # 50Hz
        self.t0 = time.time()
        self.base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.duration = duration_seconds
        self.cycle_count = 0
        self.last_cycle_time = 0.0
        
        self.get_logger().info(f'🔥 方形轨迹演示开始，将运行 {duration_seconds} 秒')
        self.get_logger().info('   使用 Ctrl+C 可随时停止')

    def on_timer(self):
        t = time.time() - self.t0
        
        # 检查是否达到运行时间限制
        if t >= self.duration:
            self.get_logger().info(f'⏰ 运行时间已达 {self.duration} 秒，自动停止')
            self.stop_motion()
            return
        
        # 检测新的循环周期
        cycle_time = t % 8.0
        if cycle_time < self.last_cycle_time:
            self.cycle_count += 1
            self.get_logger().info(f'🔄 完成第 {self.cycle_count} 个循环 (耗时: {t:.1f}s)')
        self.last_cycle_time = cycle_time
        
        # 生成一个在 J1/J2 平面内的方形路径（±10°）
        p = cycle_time / 8.0
        if p < 0.25:
            a1 = 10.0 * (p / 0.25)
            a2 = 0.0
        elif p < 0.5:
            a1 = 10.0
            a2 = 10.0 * ((p - 0.25) / 0.25)
        elif p < 0.75:
            a1 = 10.0 * (1.0 - (p - 0.5) / 0.25)
            a2 = 10.0
        else:
            a1 = 0.0
            a2 = 10.0 * (1.0 - (p - 0.75) / 0.25)
            
        target = list(self.base)
        target[0] += a1
        target[1] += a2
        msg = Float64MultiArray()
        msg.data = target
        self.pub.publish(msg)
    
    def stop_motion(self):
        """停止运动，回到零位"""
        self.get_logger().info('🛑 停止运动，回到零位...')
        # 发送零位指令
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
        # 停止定时器
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info('✅ 已停止方形轨迹演示')


def main():
    import sys
    
    # 允许用户指定运行时间
    duration = 30  # 默认30秒
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
            if duration <= 0:
                print("运行时间必须大于0")
                sys.exit(1)
        except ValueError:
            print("无效的时间参数，使用默认值30秒")
    
    rclpy.init()
    node = ServoSquare(duration_seconds=duration)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 用户手动停止')
        node.stop_motion()
        time.sleep(1)  # 给停止指令一点时间发送
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
