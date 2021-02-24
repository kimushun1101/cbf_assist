import os

from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config = os.path.join(get_package_share_directory('suitcase_teleop'), 'config', 'telop_param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value='f710'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')]),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),

        Node(
            package='suitcase_teleop', executable='joy_control',
            name='joy_control',
            remappings=[('cmd_vel', '/human_vel')],
            parameters=[config]),

        Node(  
            package='cbf_assist', executable='controller',
            remappings=[('cmd_vel', '/cmd_vel2')],
            output="screen",
            parameters=[
                {   "ctrl_param_K": 0.1,
                    "ctrl_param_C": 0.1,
                    "ctrl_param_L": 0.001
                }
            ]
        ),
    ])
