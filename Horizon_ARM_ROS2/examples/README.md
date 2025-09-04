# 📚 Horizon ARM 示例脚本

本目录包含完整的应用示例，展示ROS2机械臂控制的各种使用方法。

## 🚀 快速开始

### 前置条件
1. 确保主系统已启动：
   ```bash
   ros2 launch horizon_arm_bridge bringup_sdk_ui.launch.py
   ```

2. 在新终端中运行示例：
   ```bash
   source install/setup.bash
   python3 examples/示例文件名.py
   ```

## 📖 核心示例

### 1️⃣ 基础关节控制 (`basic_joint_control.py`)
**功能说明**: 关节空间控制基础
- 单关节控制
- 多关节协调运动
- 运动序列编程

**运行方式**:
```bash
python3 examples/basic_joint_control.py
```

### 2️⃣ 实时方形轨迹 (`servo_square.py`)
**功能说明**: 实时控制模式
- 连续轨迹控制
- 实时话题发布
- 平滑运动实现

**运行方式**:
```bash
python3 examples/servo_square.py
```

### 3️⃣ 抓取放置演示 (`pick_and_place_demo.py`)
**功能说明**: 典型工业应用
- 抓取放置流程
- 预设位置管理
- 复杂动作序列

**运行方式**:
```bash
python3 examples/pick_and_place_demo.py
```

## 🔧 实用工具

### 🗺️ 映射校验 (`map_check.py`)
**功能**: 验证关节映射配置
```bash
python3 examples/map_check.py
```

## 🎯 可视化功能

### MoveIt + RViz集成
**运动规划与可视化**: 一键启动后RViz自动配置完整功能
- 🎨 MoveIt运动规划：拖拽末端执行器进行路径规划
- 📊 轨迹可视化：显示规划路径和虚拟执行过程  
- 🔧 TF坐标系：显示机械臂各关节的坐标轴
- 📍 实时状态：显示当前关节位置和目标位置

### 使用方法
1. 启动系统：`ros2 launch horizon_arm_bridge bringup_sdk_ui.launch.py`
2. 在RViz中拖拽橙色球体（末端执行器）到目标位置
3. 点击"Plan"查看规划路径
4. 点击"Plan & Execute"观看虚拟执行演示

## 💡 开发指南

### 使用流程
1. **基础控制**: `basic_joint_control.py` → 关节控制原理
2. **实时控制**: `servo_square.py` → 连续运动控制
3. **实际应用**: `pick_and_place_demo.py` → 工业应用流程

### 扩展开发
- **参数调试**: 修改`basic_joint_control.py`中的角度值和运动序列
- **轨迹定制**: 在`servo_square.py`基础上设计圆形、三角形等轨迹
- **应用定制**: 自定义`pick_and_place_demo.py`的抓取位置和序列
- **复杂任务**: 组合多个示例，设计复杂的机械臂任务

## ⚠️ 安全提醒

1. **硬件安全**: 运行示例前确保机械臂周围无障碍物
2. **急停准备**: 随时准备使用UI中的急停按钮
3. **参数检查**: 修改角度值前确认在安全范围内（参考mapping.yaml）
4. **循序渐进**: 建议从小幅度运动开始测试

## 🔧 自定义开发

### 常用ROS2接口
- **发布目标**: `/horizon_arm/servo_target` (Float64MultiArray)
- **订阅状态**: `/joint_states` (JointState)

### 基础代码模板
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/horizon_arm/servo_target', 10)
        
    def move_joints(self, angles):
        msg = Float64MultiArray()
        msg.data = angles  # [j1, j2, j3, j4, j5, j6] 角度值
        self.target_pub.publish(msg)
```

---
**开始您的机械臂控制开发之旅！🎉**
