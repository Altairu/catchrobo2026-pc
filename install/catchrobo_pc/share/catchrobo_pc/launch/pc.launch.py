"""catchrobo_pc launch: WebGUIノードを起動する"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catchrobo_pc',
            executable='web_gui_node',
            name='web_gui_node',
            output='screen',
        ),
    ])
