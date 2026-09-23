from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('use_gazebo',default_value='false'), DeclareLaunchArgument('use_navigation',default_value='false'), DeclareLaunchArgument('rviz',default_value='false'),
      Node(package='ambulance_core',executable='digital_twin_node',name='digital_twin'),
      Node(package='ros_gz_bridge', executable='parameter_bridge',
           arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock', '/ambulance/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', '/ambulance/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry', '/ambulance/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
           remappings=[('/ambulance/scan', '/scan')],
           condition=IfCondition(LaunchConfiguration('use_gazebo')),
           name='ambulance_cmd_vel_bridge'),
      Node(package='ambulance_core', executable='ambulance_odom_tf', condition=IfCondition(LaunchConfiguration('use_gazebo')), name='ambulance_odom_tf', parameters=[{'use_sim_time': True}]),
      Node(package='ambulance_core', executable='lidar_obstacle_detector', condition=IfCondition(LaunchConfiguration('use_gazebo')), name='lidar_obstacle_detector', parameters=[{'use_sim_time': True}]),
      IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'),'launch','bringup_launch.py'])), condition=IfCondition(LaunchConfiguration('use_navigation')), launch_arguments={'slam':'True','use_sim_time':'True','use_localization':'True','autostart':'True','use_composition':'False'}.items()),
      ExecuteProcess(condition=IfCondition(LaunchConfiguration('use_gazebo')),cmd=['gz','sim','-r',PathJoinSubstitution([FindPackageShare('ambulance_gazebo'),'worlds','smart_city.sdf'])])
    ])
