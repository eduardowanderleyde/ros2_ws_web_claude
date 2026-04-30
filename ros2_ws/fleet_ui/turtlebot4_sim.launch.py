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
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _make_nav2_params(_ctx):
    """Lê o nav2.yaml do TB4, força enable_stamped_cmd_vel=False e grava num temp file.

    O bridge nav2_minimal_tb4_sim usa geometry_msgs/Twist (não TwistStamped),
    por isso o Nav2 tem de publicar sem stamp. Fix para Issue #94.
    """
    pkg_nav4 = get_package_share_directory('turtlebot4_navigation')
    src = os.path.join(pkg_nav4, 'config', 'nav2.yaml')
    with open(src) as f:
        cfg = yaml.safe_load(f)

    # Percorre todos os nós e desactiva stamped cmd_vel
    def _patch(d):
        if isinstance(d, dict):
            for k, v in d.items():
                if k == 'enable_stamped_cmd_vel':
                    d[k] = False
                else:
                    _patch(v)

    _patch(cfg)

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='_nav2.yaml', delete=False)
    yaml.safe_dump(cfg, tmp)
    tmp.close()
    return tmp.name


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
    # Gera params com enable_stamped_cmd_vel=False (bridge usa Twist, não TwistStamped)
    nav2_params_file = _make_nav2_params(None)
    nav2 = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav4, 'launch', 'nav2.launch.py')
            ),
            launch_arguments=[
                ('use_sim_time', 'true'),
                ('params_file', nav2_params_file),
            ],
        )],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sim)
    ld.add_action(slam)
    ld.add_action(nav2)
    return ld
