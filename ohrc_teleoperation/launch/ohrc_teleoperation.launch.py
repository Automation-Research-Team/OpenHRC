import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

# def get_user_frame_args(): 
#   print(str(['', LaunchConfiguration('user_frame_viewpoint'), '_']))
#   if ['', LaunchConfiguration('user_frame_viewpoint'), "'"] == 'back':
#     return ['--x', '0', '--y', '0','--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'user_frame']


def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument('robot', default_value='ur5e'),
    DeclareLaunchArgument('controller', default_value='vel'),  # vel or vel_trj or vel_pos
    DeclareLaunchArgument('interface', default_value='marker'),
    DeclareLaunchArgument('feedback_mode', default_value='PositionFeedback'),
    DeclareLaunchArgument('use_ft_filter', default_value='true'),
    DeclareLaunchArgument('user_frame_viewpoint', default_value='back'),

    DeclareLaunchArgument('hw_config', default_value=[FindPackageShare('ohrc_hw_config'), '/config/', LaunchConfiguration('robot'), '/', LaunchConfiguration('robot'), '_hw_config.yaml']),
    DeclareLaunchArgument('admittance_config', default_value=[FindPackageShare('ohrc_control'), '/config/admittance_config.yaml']),
    DeclareLaunchArgument('control_config', default_value=[FindPackageShare('ohrc_hw_config'), '/config/', LaunchConfiguration('robot'), '/', LaunchConfiguration('robot'), '_control_config_', LaunchConfiguration('controller'), '.yaml']),
    DeclareLaunchArgument('interface_config', default_value=[FindPackageShare('ohrc_teleoperation'), '/config/', LaunchConfiguration('interface'), '_config.yaml']),

    DeclareLaunchArgument('use_rviz', default_value='true'),
    DeclareLaunchArgument('rviz_config', default_value=[FindPackageShare('ohrc_teleoperation'), '/config/teleoperation.rviz']),
 

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([FindPackageShare('ohrc_control'), '/launch/ohrc_control.launch.py']),
      launch_arguments={
        'app': 'teleoperation',
        'robot': LaunchConfiguration('robot'),
        'controller': LaunchConfiguration('controller'),
        'interface': LaunchConfiguration('interface'),
        'feedback_mode': LaunchConfiguration('feedback_mode'),
        'use_ft_filter': LaunchConfiguration('use_ft_filter'),
        'hw_config': LaunchConfiguration('hw_config'),
        'admittance_config': LaunchConfiguration('admittance_config'),
        'control_config': LaunchConfiguration('control_config'),
        'interface_config': LaunchConfiguration('interface_config'),
        'use_rviz': LaunchConfiguration('use_rviz'),
        'rviz_config': LaunchConfiguration('rviz_config')
      }.items()
    ),

    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='user_frame_broadcaster',
      # arguments=get_user_frame_args(),
      arguments=['--x', '0', '--y', '0','--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'user_frame'],
      condition=IfCondition(PythonExpression(["'", LaunchConfiguration('user_frame_viewpoint'), "' == 'back'"]))
    ),
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='user_frame_broadcaster',
      arguments=['--x', '0', '--y', '0','--z', '0', '--yaw', '3.141592', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'user_frame'],
      condition=IfCondition(PythonExpression(["'", LaunchConfiguration('user_frame_viewpoint'), "' == 'face'"]))
    ),
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['--x', '0', '--y', '0','--z', '0', '--yaw', '1.57079', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'user_frame'],
      condition=IfCondition(PythonExpression(["'", LaunchConfiguration('user_frame_viewpoint'), "' == 'right'"]))
    ),
    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='user_frame_broadcaster',
      arguments=['--x', '0', '--y', '0','--z', '0', '--yaw', '-1.57079', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'user_frame'],
      condition=IfCondition(PythonExpression(["'", LaunchConfiguration('user_frame_viewpoint'), "' == 'left'"]))
    ),
  ])
