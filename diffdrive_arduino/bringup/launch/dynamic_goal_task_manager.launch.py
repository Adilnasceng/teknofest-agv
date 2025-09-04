from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Dynamic Goal Task Manager Node
    goal_task_manager_node = Node(
        package='diffdrive_arduino',
        executable='dynamic_goal_task_manager_node.py',
        name='dynamic_goal_task_manager',
        output='screen',
        parameters=[{
            'total_goals': 6,
            'forward_speed': 0.2,
            'backward_speed': -0.2,
            'forward_duration': 3.0,
            'backward_duration': 3.0,
            'task_delay': 2.0,
            'post_task_wait': 5.0,
            'return_to_start': True,  # YENİ: Başlangıç konumuna dönüş
            'debug_mode': True,
        }]
    )

    return LaunchDescription([
        goal_task_manager_node
    ])