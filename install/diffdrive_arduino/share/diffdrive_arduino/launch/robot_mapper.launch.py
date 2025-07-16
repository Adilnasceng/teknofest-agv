from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("robot_lidar_mapper"), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Get lidar parameters
    lidar_config = PathJoinSubstitution(
        [FindPackageShare("robot_lidar_mapper"), "config", "lidar_params.yaml"]
    )
    
    # Nodes to launch
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],

    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
#    rviz_config = PathJoinSubstitution([
 #       FindPackageShare('robot_lidar_mapper'),
  #      'config',
  #      'rviz.rviz'
  #  ])

    # RViz2 düğümünü başlat
#    rviz2_node = Node(
 #       package='rviz2',
  #      executable='rviz2',
  #      name='rviz2',
  #      arguments=['-d', rviz_config],
   #     output='screen'
 #   )
    # Lidar scanner node
   # lidar_scanner_node = Node(
    #    package="robot_lidar_mapper",
     #   executable="lidar_scanner.py",
     #   name="lidar_scanner",
#        parameters=[lidar_config, {"use_sim_time": use_sim_time}],
 #   )
    
    # Motor controller node
#    motor_controller_node = Node(
 #       package="robot_lidar_mapper",
 #       executable="motor_controller.py",
  #      name="motor_controller",
   #     parameters=[{"use_sim_time": use_sim_time}],
   # )
    
    # Teleop node
    #teleop_node = Node(
     #   package="robot_lidar_mapper",
      #  executable="teleop_key.py",
       # name="teleop_key",
        #output="screen",
    #)
    
    # SLAM Toolbox
   # slam_toolbox_node = Node(
#        package="slam_toolbox",
#        executable="async_slam_toolbox_node",
 #       name="slam_toolbox",
  #      parameters=[
   #         {"use_sim_time": use_sim_time},
    #        {"map_file_name": "map"},
 #           {"map_update_interval": 5.0},
  #          {"max_laser_range": 10.0},
   #         {"min_map_size": 10},
    #        {"resolution": 0.05},
     #   ],
    #    output="screen",
   # )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
   # ld.add_action(lidar_scanner_node)
    #ld.add_action(motor_controller_node)
   # ld.add_action(teleop_node)
    #ld.add_action(slam_toolbox_node)
  #  ld.add_action(rviz2_node)
    return ld
