from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([
    # Declare launch arguments
    DeclareLaunchArgument('robot', default_value='ur5e'),
    DeclareLaunchArgument('controller', default_value='vel'),
    DeclareLaunchArgument('feedback_mode', default_value='PositionFeedback'),
    DeclareLaunchArgument('use_rviz', default_value='true'),

    # Include the other launch file
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([FindPackageShare('ohrc_teleoperation'), '/launch/ohrc_teleoperation.launch.py']),
      launch_arguments={
        'interface': 'marker',
        'robot': LaunchConfiguration('robot'),
        'controller': LaunchConfiguration('controller'),
        'feedback_mode': LaunchConfiguration('feedback_mode'),
        'use_rviz': LaunchConfiguration('use_rviz')
      }.items()
    ),
  ])
