from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('use_gazebo',default_value='false'), DeclareLaunchArgument('rviz',default_value='false'),
      Node(package='ambulance_core',executable='digital_twin_node',name='digital_twin'),
      ExecuteProcess(condition=IfCondition(LaunchConfiguration('use_gazebo')),cmd=['gz','sim','-r',PathJoinSubstitution([FindPackageShare('ambulance_gazebo'),'worlds','smart_city.sdf'])])
    ])
