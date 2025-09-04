from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 参数
    mapping_path = LaunchConfiguration('mapping_path')
    resample_rate_hz = LaunchConfiguration('resample_rate_hz')
    skip_first_point = LaunchConfiguration('skip_first_point')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    hold_final_seconds = LaunchConfiguration('hold_final_seconds')
    hardware_mode = LaunchConfiguration('hardware_mode')
    js_idle_rate_hz = LaunchConfiguration('js_idle_rate_hz')
    enable_servo_topic = LaunchConfiguration('enable_servo_topic')
    enable_hw_on_start = LaunchConfiguration('enable_hw_on_start')

    decl = [
        DeclareLaunchArgument('mapping_path', default_value='arm-ros2/mapping.yaml'),
        DeclareLaunchArgument('resample_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('skip_first_point', default_value='true'),
        # 默认由驱动发布 joint_states（保证 RViz 正常工作）；
        # 若在 UI 中开启反向仿真，会临时关闭驱动端发布以避免闪烁。
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        DeclareLaunchArgument('hold_final_seconds', default_value='2.0'),
        DeclareLaunchArgument('hardware_mode', default_value='none'),
        DeclareLaunchArgument('js_idle_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('enable_servo_topic', default_value='true'),
        DeclareLaunchArgument('enable_hw_on_start', default_value='false'),
    ]

    # MoveIt + RViz
    moveit_share = get_package_share_directory('horizon_arm_moveit_config')
    moveit_demo = os.path.join(moveit_share, 'launch', 'demo.launch.py')
    moveit = IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit_demo))

    # SDK 驱动（RViz→驱动→真机）
    driver = Node(
        package='horizon_arm_bridge',
        executable='trajectory_stream_sdk_driver.py',
        name='trajectory_stream_sdk_driver',
        output='screen',
        parameters=[{
            'mapping_path': mapping_path,
            'resample_rate_hz': ParameterValue(resample_rate_hz, value_type=float),
            'skip_first_point': ParameterValue(skip_first_point, value_type=bool),
            'publish_joint_states': ParameterValue(publish_joint_states, value_type=bool),
            'hold_final_seconds': ParameterValue(hold_final_seconds, value_type=float),
            'hardware_mode': hardware_mode,
            'js_idle_rate_hz': ParameterValue(js_idle_rate_hz, value_type=float),
            'enable_servo_topic': ParameterValue(enable_servo_topic, value_type=bool),
            'enable_hw_on_start': ParameterValue(enable_hw_on_start, value_type=bool),
        }]
    )


    # UI（启用接收，用于显示RViz目标/回显）
    ui_cmd = (
        'cd ~/Horizon_ARM_ROS2 && '
        'source ~/Horizon_ARM_ROS2/install/setup.bash && '
        'python3 ~/Horizon_ARM_ROS2/arm-ros2/review_ui.py --mapping ~/Horizon_ARM_ROS2/arm-ros2/mapping.yaml --com-port /dev/ttyACM0'
    )
    ui = ExecuteProcess(cmd=['bash', '-lc', ui_cmd], output='screen', shell=False)

    return LaunchDescription(decl + [moveit, driver, ui])



