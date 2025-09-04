from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    desc_pkg = get_package_share_directory('new_arm')
    moveit_pkg = get_package_share_directory('horizon_arm_moveit_config')

    # robot_state_publisher + rviz from description package (传入rviz配置)
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'display.launch.py')),
        launch_arguments={'rviz_config': os.path.join(moveit_pkg, 'rviz', 'new_arm.rviz')}.items()
    )

    # Move Group
    urdf_path = os.path.join(desc_pkg, 'urdf', 'new_arm.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_xml = f.read()

    # 将 YAML 文件读为 dict，避免 rcl 参数文件语法错误
    def load_yaml(path):
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    # Helpers to wrap params under MoveIt expected keys
    kin_yaml = load_yaml(os.path.join(moveit_pkg, 'config', 'kinematics.yaml'))
    ompl_yaml = load_yaml(os.path.join(moveit_pkg, 'config', 'ompl_planning.yaml'))
    limits_yaml = load_yaml(os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'))
    controllers_yaml = load_yaml(os.path.join(moveit_pkg, 'config', 'moveit_controllers.yaml'))

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': urdf_xml},
            {'robot_description_semantic': open(os.path.join(moveit_pkg, 'config', 'new_arm.srdf'), 'r', encoding='utf-8').read()},
            {'robot_description_kinematics': kin_yaml},
            {'robot_description_planning': limits_yaml},
            ompl_yaml,
            controllers_yaml,
            {'start_state_max_bounds_error': 0.5},
        ]
    )

    # ros2_control节点 - 仅用于虚拟演示
    ros2_controllers_yaml = load_yaml(os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml'))
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': urdf_xml},
            ros2_controllers_yaml
        ],
        output="screen",
    )

    # 控制器spawner
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        shell=False,
        output='screen'
    )

    manipulator_controller_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active', 
            'fake_manipulator_controller'
        ],
        shell=False,
        output='screen'
    )

    return LaunchDescription([
        display,
        move_group,
        # 延迟启动ros2_control，确保move_group先启动
        TimerAction(period=3.0, actions=[ros2_control_node]),
        # 只启动fake_manipulator_controller用于轨迹演示，不启动joint_state_broadcaster
        TimerAction(period=5.0, actions=[manipulator_controller_spawner]),
    ])


