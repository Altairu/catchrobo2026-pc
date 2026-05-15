"""catchrobo_pc launch: WebGUIノードを起動し、デバッグモニターをWezTermの新タブで開く"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# source済みのsetupスクリプトを引き継いで新タブでdebug_nodeを起動するコマンド
_DEBUG_CMD = (
    'source /opt/ros/humble/setup.bash'
    ' && source "$(dirname $(dirname $(which ros2)))/../catchrobo2026-pc/install/setup.bash" 2>/dev/null'
    ' || source ~/catchrobo2026-pc/install/setup.bash;'
    ' ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}'
    ' ros2 run catchrobo_pc debug_node;'
    ' echo "[終了] Enterで閉じる"; read'
)


def generate_launch_description():
    return LaunchDescription([
        # WebGUI ノード (メインプロセス)
        Node(
            package='catchrobo_pc',
            executable='web_gui_node',
            name='web_gui_node',
            output='screen',
        ),
        # デバッグモニターを WezTerm の新タブで起動
        ExecuteProcess(
            cmd=['wezterm', 'cli', 'spawn', '--', 'bash', '-c', _DEBUG_CMD],
            output='screen',
            shell=False,
        ),
    ])
