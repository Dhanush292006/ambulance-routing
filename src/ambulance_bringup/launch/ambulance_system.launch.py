from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    share = FindPackageShare('ambulance_bringup')
    share_path = get_package_share_directory('ambulance_bringup')
    world = PathJoinSubstitution([FindPackageShare('ambulance_gazebo'), 'worlds', 'smart_city.sdf'])
    gui_config = os.path.join(get_package_share_directory('ambulance_gazebo'), 'gui.config')
    nav2 = os.path.join(share_path, 'config', 'nav2.yaml')
    map_yaml = os.path.join(share_path, 'maps', 'city_roads.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_navigation', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        Node(package='ambulance_core', executable='digital_twin_node', name='digital_twin'),
        Node(package='ambulance_core', executable='ambulance_mission_manager', name='mission_manager', condition=IfCondition(LaunchConfiguration('use_navigation'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/ambulance/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/ambulance/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/ambulance/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
             remappings=[('/ambulance/scan', '/scan')],
             condition=IfCondition(LaunchConfiguration('use_gazebo')), name='gazebo_ros_bridge'),
        Node(package='ambulance_core', executable='ambulance_odom_tf',
             condition=IfCondition(LaunchConfiguration('use_gazebo')), name='ambulance_odom_tf',
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             condition=IfCondition(LaunchConfiguration('use_navigation')),
             arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='ambulance_core', executable='lidar_obstacle_detector',
             condition=IfCondition(LaunchConfiguration('use_gazebo')), name='lidar_obstacle_detector',
             parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_navigation')),
            launch_arguments={'slam': 'False', 'map': LaunchConfiguration('map'), 'params_file': nav2,
                              'use_sim_time': 'True', 'autostart': 'True',
                              'use_composition': 'False', 'use_localization': 'True'}.items()),
        ExecuteProcess(condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_gazebo'), "' == 'true' and '", LaunchConfiguration('headless'), "' == 'true'"])),
                       cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world], output='screen'),
        ExecuteProcess(condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_gazebo'), "' == 'true' and '", LaunchConfiguration('headless'), "' == 'false'"])),
                       cmd=['gz', 'sim', '--gui-config', gui_config, '-r', world], output='screen')
    ])
