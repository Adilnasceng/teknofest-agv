#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Goal Sound Node - ROS2 Humble uyumlu
    goal_sound_node = Node(
        package='diffdrive_arduino',
        executable='goal_sound_node.py',
        name='goal_sound_node',
        output='screen',
        parameters=[{
            'enable_goal_sounds': True,      # Hedef seslerini aktif et
            'sound_delay': 1.0,              # Ses çalmadan önce 1.5 saniye bekle
            'debug_mode': False,              # Debug logları pasif
        }]
    )

    return LaunchDescription([
        goal_sound_node
    ])