from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = False  # İstersen bunu değiştir

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            {"use_sim_time": use_sim_time},
         #   {"map_file_name": "map"},
            {"map_update_interval": 5.0},
            {"max_laser_range": 10.0},
            {"min_laser_range": 0.3},  # Burada min_laser_range 0.3 olarak ayarlandı
            {"min_map_size": 10},
            {"resolution": 0.05},
            {"map_start_pose": [0.0, 0.0, 0.0]},  # x, y, theta (radyan)

        ],
        output="screen",
    )

    return LaunchDescription([
        slam_toolbox_node
    ])
