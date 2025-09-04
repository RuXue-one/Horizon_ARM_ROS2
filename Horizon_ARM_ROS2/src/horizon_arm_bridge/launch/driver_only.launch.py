from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_path = LaunchConfiguration('mapping_path')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    enable_servo_topic = LaunchConfiguration('enable_servo_topic')
    js_idle_rate_hz = LaunchConfiguration('js_idle_rate_hz')

    decl = [
        DeclareLaunchArgument('mapping_path', default_value='arm-ros2/mapping.yaml'),
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        DeclareLaunchArgument('enable_servo_topic', default_value='true'),
        DeclareLaunchArgument('js_idle_rate_hz', default_value='30.0'),
    ]

    driver = Node(
        package='horizon_arm_bridge',
        executable='trajectory_stream_sdk_driver.py',
        name='trajectory_stream_sdk_driver',
        output='screen',
        parameters=[{
            'mapping_path': mapping_path,
            'publish_joint_states': ParameterValue(publish_joint_states, value_type=bool),
            'enable_servo_topic': ParameterValue(enable_servo_topic, value_type=bool),
            'js_idle_rate_hz': ParameterValue(js_idle_rate_hz, value_type=float),
        }]
    )

    return LaunchDescription(decl + [driver])
