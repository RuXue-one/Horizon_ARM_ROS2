from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def load_urdf():
    pkg_share = get_package_share_directory('new_arm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'new_arm.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        return f.read()


def load_srdf():
    try:
        moveit_pkg = get_package_share_directory('horizon_arm_moveit_config')
        srdf_path = os.path.join(moveit_pkg, 'config', 'new_arm.srdf')
        with open(srdf_path, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception:
        return ''


def load_kinematics():
    try:
        import yaml
        moveit_pkg = get_package_share_directory('horizon_arm_moveit_config')
        with open(os.path.join(moveit_pkg, 'config', 'kinematics.yaml'), 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def load_limits():
    try:
        import yaml
        moveit_pkg = get_package_share_directory('horizon_arm_moveit_config')
        with open(os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'), 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='是否使用 joint_state_publisher_gui（默认关闭）')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='',
        description='RViz 配置文件路径（可选）')
    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = {'robot_description': load_urdf()}
    robot_description_semantic = {'robot_description_semantic': load_srdf()}
    robot_description_kinematics = {'robot_description_kinematics': load_kinematics()}
    robot_description_planning = {'robot_description_planning': load_limits()}

    nodes = []

    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
    )

    nodes.append(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[],
            condition=IfCondition(use_gui),
        )
    )

    # 如果传入了 rviz_config，则加载该配置
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                robot_description_planning,
            ],
            additional_env={
                # 'LIBGL_ALWAYS_SOFTWARE': '1',
                # 'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
            }
        )
    )

    return LaunchDescription([use_gui_arg, rviz_config_arg] + nodes)


