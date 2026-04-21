#!/usr/bin/env python3
"""Launch multi-robot fleet simulation.

Inicia:
  - Gazebo (turtlebot3_world) — servidor + cliente
  - Para cada robô (tb1, tb2):
      * SDF modificado com frame IDs namespaceados  (tb1/odom, tb1/base_footprint…)
      * Spawn do modelo no Gazebo
      * gz_ros2_bridge (tópicos tb1/*)
      * robot_state_publisher com frame_prefix=tb1/
      * Nav2 + SLAM Toolbox (namespace=tb1)
  - fleet_orchestrator  (robots: [tb1, tb2])
  - fleet_data_collector (robots: [tb1, tb2])

Uso:
  ros2 launch fleet_bringup multi_robot_sim.launch.py
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_robot_sdf(robot_id: str) -> str:
    """Gera SDF temporário com frame IDs prefixados pelo robot_id.

    O SDF padrão do TurtleBot3 tem:
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <tf_topic>/tf</tf_topic>   ← absoluto, não é scoped pelo Gz

    Para multi-robô precisamos de frames únicos no TF tree global e
    do tópico tf scoped ao modelo (Gz adiciona "tb1/" ao prefixar o tópico
    relativo automaticamente).
    """
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        f'turtlebot3_{TURTLEBOT3_MODEL}',
        'model.sdf',
    )
    with open(sdf_path) as fh:
        content = fh.read()

    # Prefix frame IDs so each robot has unique TF frames
    content = content.replace(
        '<frame_id>odom</frame_id>',
        f'<frame_id>{robot_id}/odom</frame_id>',
    )
    content = content.replace(
        '<child_frame_id>base_footprint</child_frame_id>',
        f'<child_frame_id>{robot_id}/base_footprint</child_frame_id>',
    )
    # Make tf_topic relative so Gz scopes it to "tb1/tf" (not global "/tf")
    content = content.replace(
        '<tf_topic>/tf</tf_topic>',
        '<tf_topic>tf</tf_topic>',
    )

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix='.sdf',
        prefix=f'fleet_{robot_id}_',
        delete=False,
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def _robot_nodes(robot_id: str, x: float, y: float,
                 bridge_yaml: str, nav2_yaml: str) -> list:
    """Retorna todos os nós/includes necessários para um robô."""
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

    sdf_file = _make_robot_sdf(robot_id)

    # URDF/SDF para o robot_state_publisher (lê o original — RSP usa frame_prefix)
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        f'turtlebot3_{TURTLEBOT3_MODEL}',
        'model.sdf',
    )
    with open(urdf_path) as fh:
        robot_desc = fh.read()

    # 1. Spawn no Gazebo — TimerAction dá tempo ao Gz server para estar pronto
    spawn = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_id}',
            arguments=[
                '-name', robot_id,
                '-file', sdf_file,
                '-x', str(x),
                '-y', str(y),
                '-z', '0.01',
            ],
            output='screen',
        )],
    )

    # 2. Bridge Gz ↔ ROS — inicia logo após spawn para sensores estarem prontos antes do Nav2
    bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{robot_id}',
            arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}'],
            output='screen',
        )],
    )

    # 3. Robot State Publisher com frame_prefix para isolar TF
    #    Subscreve joint_states em /tb1/joint_states via remapping
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{robot_id}',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': f'{robot_id}/',
            'use_sim_time': True,
        }],
        remappings=[
            ('joint_states', f'/{robot_id}/joint_states'),
            ('/tf', f'/{robot_id}/tf'),
            ('/tf_static', f'/{robot_id}/tf_static'),
        ],
        output='screen',
    )

    # 4. Nav2 + SLAM — inicia após bridge (t=6s) para já ter sensores disponíveis
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': robot_id,
                    'use_namespace': 'true',
                    'slam': 'True',
                    'use_sim_time': 'true',
                    'autostart': 'true',
                    'params_file': nav2_yaml,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'log_level': 'warn',
                }.items(),
            )
        ],
    )

    return [spawn, bridge, rsp, nav2]


# ---------------------------------------------------------------------------
# OpaqueFunction: instancia nós para cada robô em tempo de launch
# ---------------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('fleet_bringup')
    fleet_orch_pkg = get_package_share_directory('fleet_orchestrator')

    tb1_bridge = os.path.join(pkg, 'params', 'tb1_bridge.yaml')
    tb2_bridge = os.path.join(pkg, 'params', 'tb2_bridge.yaml')
    tb1_nav2   = os.path.join(pkg, 'params', 'tb1_nav2.yaml')
    tb2_nav2   = os.path.join(pkg, 'params', 'tb2_nav2.yaml')
    multi_yaml = os.path.join(pkg, 'config', 'multi_robot_sim.yaml')

    nodes = []
    nodes += _robot_nodes('tb1', -2.0, -0.5, tb1_bridge, tb1_nav2)
    nodes += _robot_nodes('tb2', -2.0,  0.5, tb2_bridge, tb2_nav2)

    orchestrator = Node(
        package='fleet_orchestrator',
        executable='fleet_orchestrator',
        name='fleet_orchestrator',
        output='screen',
        parameters=[ParameterFile(multi_yaml, allow_substs=True)],
    )
    collector = Node(
        package='fleet_data_collector',
        executable='sensor_collector',
        name='sensor_collector',
        output='screen',
        parameters=[ParameterFile(multi_yaml, allow_substs=True)],
    )

    return nodes + [orchestrator, collector]


# ---------------------------------------------------------------------------
# Descrição principal
# ---------------------------------------------------------------------------

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg_gazebo, 'worlds', 'turtlebot3_world.world')

    # Gz server (headless -s) + client (-g) separados
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Bridge do clock (compartilhado — um único)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models'),
    )

    return LaunchDescription([
        set_gz_resources,
        gz_server,
        gz_client,
        clock_bridge,
        OpaqueFunction(function=launch_setup),
    ])
