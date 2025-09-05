from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Dynamic Goal Task Manager Node with namespace
    goal_task_manager_node = Node(
        package='diffdrive_arduino',
        executable='dynamic_goal_task_manager_node.py',
        name='enhanced_task_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'base_goals': 2,
            'navigation_wait': 5.0,
            'forward_speed': 0.2,
            'backward_speed': -0.2,
            'forward_duration': 3.0,
            'backward_duration': 3.0,
            'task_delay': 2.0,
            'enable_obstacle_control': True,  # Yönlenme görevlerinde engel algılama aktif
            'post_task_wait': 5.0,
            'return_to_start': True,
            'debug_mode': True,
        }]
    )

    return LaunchDescription([
        goal_task_manager_node
    ])