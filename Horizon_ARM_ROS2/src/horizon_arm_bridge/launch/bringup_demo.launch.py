#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('horizon_arm_moveit_config'), 'rviz', 'new_arm.rviz'
        ]),
        description='RViz配置文件路径'
    )
    
    # 获取包路径
    moveit_pkg = FindPackageShare('horizon_arm_moveit_config')
    new_arm_pkg = FindPackageShare('new_arm')
    
    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([
                new_arm_pkg, 'urdf', 'new_arm.urdf'
            ])
        }]
    )
    
    # 关节状态发布（演示用）
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz可视化
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'robot_description': PathJoinSubstitution([
                new_arm_pkg, 'urdf', 'new_arm.urdf'
            ])
        }]
    )
    
    # MoveIt演示（可选）
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_pkg, 'launch', 'demo.launch.py'])
        ])
    )
    
    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        # moveit_demo,  # 取消注释以启用MoveIt
    ])
