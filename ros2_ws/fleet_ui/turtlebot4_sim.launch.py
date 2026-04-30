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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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

    # ── TF publisher: /sim_ground_truth_pose → TF odom→base_link ─────────────
    # O bridge Gazebo não publica /model/turtlebot4/tf no Harmonic.
    # Este nó lê o ground truth e publica odom→base_link em /tf para
    # que SLAM, Nav2 e o orquestrador possam localizar o robô.
    tf_publisher = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=['python3',
                 '/home/eduardo/Documentos/ros2_ws/ros2_ws/fleet_ui/tb4_tf_publisher.py'],
            output='screen',
        )],
    )

    # ── Relay: /sim_ground_truth_pose → /odom (odometria para coleta) ─────────
    # O TB4 não publica /odom directamente — usa sim_ground_truth_pose.
    odom_relay = TimerAction(
        period=6.0,
        actions=[Node(
            package='topic_tools',
            executable='relay',
            name='tb4_odom_relay',
            arguments=['/sim_ground_truth_pose', '/odom'],
            output='screen',
        )],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(tf_publisher)
    ld.add_action(odom_relay)
    ld.add_action(slam)
    ld.add_action(nav2)
    return ld
