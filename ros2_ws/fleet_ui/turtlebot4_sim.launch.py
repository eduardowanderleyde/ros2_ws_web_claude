#!/usr/bin/env python3
"""
TurtleBot4 Standard — simulação completa com Gazebo + SLAM + Nav2.

Uso:
  ros2 launch ~/Documentos/ros2_ws/ros2_ws/fleet_ui/turtlebot4_sim.launch.py
  ros2 launch ~/Documentos/ros2_ws/ros2_ws/fleet_ui/turtlebot4_sim.launch.py world:=maze
  ros2 launch ~/Documentos/ros2_ws/ros2_ws/fleet_ui/turtlebot4_sim.launch.py world:=depot

Mundos disponíveis: warehouse (padrão), maze, depot
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Mundo Gazebo: warehouse | maze | depot'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Modelo TurtleBot4'),
    DeclareLaunchArgument('x', default_value='0.0', description='Posição X inicial'),
    DeclareLaunchArgument('y', default_value='0.0', description='Posição Y inicial'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw inicial (rad)'),
]


def generate_launch_description():
    pkg_tb4_gz    = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_nav   = get_package_share_directory('turtlebot4_navigation')

    # ── Gazebo + spawn do robô ─────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_gz, 'launch', 'turtlebot4_gz.launch.py'])
        ),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('model', LaunchConfiguration('model')),
            ('x',     LaunchConfiguration('x')),
            ('y',     LaunchConfiguration('y')),
            ('yaw',   LaunchConfiguration('yaw')),
        ],
    )

    # ── SLAM Toolbox (inicia 8s depois do Gazebo) ──────────────────────────────
    slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
            ),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('sync', 'true'),
            ],
        )],
    )

    # ── Nav2 (inicia 12s depois do Gazebo) ────────────────────────────────────
    nav2 = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])
            ),
            launch_arguments=[
                ('use_sim_time', 'true'),
            ],
        )],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(slam)
    ld.add_action(nav2)
    return ld
