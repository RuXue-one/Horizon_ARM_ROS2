#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    mapping_path_arg = DeclareLaunchArgument(
        'mapping_path',
        default_value='arm-ros2/mapping.yaml',
        description='机械臂映射配置文件路径'
    )
    
    publish_joint_states_arg = DeclareLaunchArgument(
        'publish_joint_states',
        default_value='true',
        description='是否发布joint_states话题'
    )
    
    # 获取包路径
    bridge_pkg = FindPackageShare('horizon_arm_bridge')
    
    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                FindPackageShare('new_arm'), 'urdf', 'new_arm.urdf'
            ])
        }]
    )
    
    # 驱动桥接节点
    trajectory_driver = Node(
        package='horizon_arm_bridge',
        executable='trajectory_stream_sdk_driver.py',
        name='trajectory_stream_sdk_driver',
        output='screen',
        parameters=[{
            'mapping_path': LaunchConfiguration('mapping_path'),
            'publish_joint_states': LaunchConfiguration('publish_joint_states'),
            'hardware_mode': 'auto'
        }]
    )
    
    # UI控制节点（通过脚本启动）
    ui_controller = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution([
            FindPackageShare('horizon_arm_bridge'), '..', '..', 'arm-ros2', 'review_ui.py'
        ]), '--mapping', LaunchConfiguration('mapping_path')],
        name='review_ui',
        output='screen'
    )
    
    return LaunchDescription([
        mapping_path_arg,
        publish_joint_states_arg,
        robot_state_publisher,
        trajectory_driver,
        ui_controller
    ])
