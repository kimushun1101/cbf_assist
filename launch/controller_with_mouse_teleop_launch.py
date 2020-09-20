from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mouse_teleop',
            node_executable='mouse_teleop',
            node_name='mouse_teleop',
            remappings=[('mouse_vel', '/human_vel'),]
        ),
        Node(  
            package='cbf_assist',
            node_executable='controller',
        )
    ])
