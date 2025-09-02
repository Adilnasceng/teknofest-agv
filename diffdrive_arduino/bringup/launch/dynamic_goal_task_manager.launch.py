from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Line Follower Node
    line_follower_node = Node(
        package='diffdrive_arduino',
        executable='line_follower_node.py',
        name='line_follower_node',
        output='screen'
    )
    
    # Dynamic Goal Task Manager Node
    goal_task_manager_node = Node(
        package='diffdrive_arduino',
        executable='dynamic_goal_task_manager_node.py',
        name='dynamic_goal_task_manager',
        output='screen',
        parameters=[{
            'total_goals': 6,
            
            # Çizgi takibi parametreleri
            'line_follow_duration': 10.0,  # Çizgi takibi süresi (saniye)
            
            # Özel hareket parametreleri
            'forward_speed': 0.2,          # Kutu alma için ileri hız
            'forward_duration': 3.0,       # Kutu alma için ileri süresi
            'turn_speed': 0.5,             # Kutu bırakma için dönüş hızı (rad/s)
            'turn_duration': 3.14,         # Kutu bırakma için 180° dönüş süresi (pi saniye)
            
            # Genel parametreler
            'task_delay': 2.0,
            'post_task_wait': 5.0,
            'return_to_start': True,
            'debug_mode': True,
        }]
    )

    return LaunchDescription([
        line_follower_node,      # Çizgi takibi node'u
        goal_task_manager_node   # Ana görev yöneticisi
    ])