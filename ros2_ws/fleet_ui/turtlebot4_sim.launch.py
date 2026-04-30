#!/usr/bin/env python3
"""
TurtleBot4 Standard — simulação via nav2_minimal_tb4_sim (Gazebo Harmonic nativo).

Usa o pacote mínimo do Nav2 que resolve problemas de compatibilidade do
irobot_create_gz_plugins com Gazebo Harmonic (Issues #81, #94, #563).

Mundos disponíveis: warehouse (padrão), depot

Uso:
  ros2 launch ~/Documentos/ros2_ws/ros2_ws/fleet_ui/turtlebot4_sim.launch.py
  ros2 launch ~/Documentos/ros2_ws/ros2_ws/fleet_ui/turtlebot4_sim.launch.py world:=depot
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Mundo: warehouse | depot'),
    DeclareLaunchArgument('x_pose', default_value='0.0'),
    DeclareLaunchArgument('y_pose', default_value='0.0'),
    DeclareLaunchArgument('yaw',    default_value='0.0'),
]


def generate_launch_description():
    pkg_minimal = get_package_share_directory('nav2_minimal_tb4_sim')
    pkg_nav4    = get_package_share_directory('turtlebot4_navigation')

    world_path = PathJoinSubstitution([pkg_minimal, 'worlds',
                                       [LaunchConfiguration('world'), '.sdf']])

    # ── Simulação mínima TB4 (Gazebo Harmonic nativo) ─────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_minimal, 'launch', 'simulation.launch.py')
        ),
        launch_arguments=[
            ('world',   world_path),
            ('headless', 'False'),
            ('x_pose',  LaunchConfiguration('x_pose')),
            ('y_pose',  LaunchConfiguration('y_pose')),
            ('yaw',     LaunchConfiguration('yaw')),
        ],
    )

    # ── SLAM Toolbox (8s após Gazebo) ─────────────────────────────────────────
    slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav4, 'launch', 'slam.launch.py')
            ),
            launch_arguments=[('use_sim_time', 'true'), ('sync', 'true')],
        )],
    )

    # ── Nav2 (12s após Gazebo) ────────────────────────────────────────────────
    nav2 = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav4, 'launch', 'nav2.launch.py')
            ),
            launch_arguments=[('use_sim_time', 'true')],
        )],
    )

    # ── TF publisher: sim_ground_truth_pose → odom→base_link ─────────────────
    # Garante que o orquestrador consegue fazer lookup map→base_link.
    tf_pub = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(
            cmd=['python3',
                 '/home/eduardo/Documentos/ros2_ws/ros2_ws/fleet_ui/tb4_tf_publisher.py'],
            output='screen',
        )],
    )

    # ── Relay sim_ground_truth_pose → /odom (para coleta de dados) ────────────
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
    ld.add_action(sim)
    ld.add_action(odom_relay)
    ld.add_action(tf_pub)
    ld.add_action(slam)
    ld.add_action(nav2)
    return ld
