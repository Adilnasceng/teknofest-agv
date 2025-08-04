import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    # Mutlak dosya yolları
    nav2_yaml = '/home/ubuntu/denem/diffdrive_arduino/bringup/config/nav_params.yaml'
    map_file = '/home/ubuntu/denem/diffdrive_arduino/bringup/maps/my_map.yaml'

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        # 'recoveries_server',  # Eğer bu paket yoksa burayı yorum satırı yap
        'bt_navigator'
    ]

    remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]
        ),

        # Recoveries Server node'u kaldırıldı, yoksa ekleyebilirsin
        # Node(
        #     package='nav2_recoveries',
        #     executable='recoveries_server',
        #     name='recoveries_server',
        #     parameters=[nav2_yaml],
        #     output='screen'
        # ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        )
    ])
