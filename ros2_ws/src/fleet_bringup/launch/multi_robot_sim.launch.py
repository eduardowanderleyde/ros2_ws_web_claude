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
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace, SetParameter, SetRemap
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
    # In Gazebo Harmonic, plugin and sensor <topic> tags do NOT get scoped to
    # the model namespace automatically. Every output topic must be explicitly
    # set to an absolute path with the robot_id prefix.
    content = content.replace(
        '<odom_topic>odom</odom_topic>',
        f'<odom_topic>/{robot_id}/odom</odom_topic>',
    )
    content = content.replace(
        '<tf_topic>/tf</tf_topic>',
        f'<tf_topic>/{robot_id}/tf</tf_topic>',
    )
    # Sensor topics (lidar, imu) — use replace_all=False since there's one each
    content = content.replace(
        '<topic>scan</topic>',
        f'<topic>/{robot_id}/scan</topic>',
    )
    # Reduz taxa do LiDAR de 10 Hz para 5 Hz — alivia carga do SLAM com 2 robôs
    content = content.replace(
        f'<update_rate>10</update_rate>\n        <topic>/{robot_id}/scan</topic>',
        f'<update_rate>5</update_rate>\n        <topic>/{robot_id}/scan</topic>',
    )
    content = content.replace(
        '<topic>imu</topic>',
        f'<topic>/{robot_id}/imu</topic>',
    )
    # JointStatePublisher topic
    content = content.replace(
        '<topic>joint_states</topic>',
        f'<topic>/{robot_id}/joint_states</topic>',
    )
    # Sensor gz_frame_id must match the TF frames published by RSP (with frame_prefix)
    content = content.replace(
        '<gz_frame_id>base_scan</gz_frame_id>',
        f'<gz_frame_id>{robot_id}/base_scan</gz_frame_id>',
    )
    content = content.replace(
        '<gz_frame_id>camera_rgb_frame</gz_frame_id>',
        f'<gz_frame_id>{robot_id}/camera_rgb_frame</gz_frame_id>',
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
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
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

    # 2. Bridge misto Gz ↔ ROS — odom, cmd_vel, imu, scan
    #    tf e joint_states são isolados em processos dedicados abaixo para evitar
    #    TF_OLD_DATA causado por contenção de threads no bridge compartilhado.
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

    # 2b. Bridge dedicado para TF dinâmico (DiffDrive odom→base_footprint)
    #     Processo isolado elimina contenção de threads que causava TF_OLD_DATA.
    tf_bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'tf_bridge_{robot_id}',
            arguments=[
                f'{robot_id}/tf'
                f'@tf2_msgs/msg/TFMessage'
                f'[gz.msgs.Pose_V',
            ],
            output='screen',
        )],
    )

    # 2c. Bridge dedicado para joint_states (alimenta o robot_state_publisher)
    #     Processo isolado garante que base_link→wheel_* chegue sem regressão temporal.
    js_bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'js_bridge_{robot_id}',
            arguments=[
                f'{robot_id}/joint_states'
                f'@sensor_msgs/msg/JointState'
                f'[gz.msgs.Model',
            ],
            output='screen',
        )],
    )

    # 3. Robot State Publisher com frame_prefix para isolar TF
    #    Remapeia /tf e /tf_static para o namespace do robô para que o SLAM
    #    (em namespace tb1) receba os transforms estáticos das juntas.
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
            ('tf', f'/{robot_id}/tf'),
            ('tf_static', f'/{robot_id}/tf_static'),
        ],
        output='screen',
    )

    # 4. Nav2 sem SLAM (slam:=False) — inicia após bridge para já ter sensores disponíveis
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
                    'slam': 'False',
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

    # 5. SLAM Toolbox assíncrono — lançado separadamente para não depender do
    #    bringup_launch.py→slam_launch.py→online_sync_launch.py que força modo síncrono.
    #    GroupAction com PushROSNamespace isola scan/tf/map no namespace de cada robô.
    slam = TimerAction(
        period=12.0,
        actions=[GroupAction(actions=[
            SetParameter('use_sim_time', True),
            PushROSNamespace(robot_id),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/map', dst='map'),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                parameters=[ParameterFile(nav2_yaml, allow_substs=True)],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_saver'],
                    'use_sim_time': True,
                }],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': nav2_yaml,
                }.items(),
            ),
        ])],
    )

    return [spawn, bridge, tf_bridge, js_bridge, rsp, nav2, slam]


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

    headless = LaunchConfiguration('headless')

    # Gz server — sempre roda (física + sensores)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    # Gz client (GUI) — só sobe se headless:=false
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models'),
    )

    # Clock bridge dedicado — nó único, isolado, inicia logo após o Gz server
    # Nó único evita micro-jitter do clock por contenção de threads com outros tópicos
    clock_bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Se true, não abre a janela do Gazebo (economiza CPU/RAM)',
        ),
        set_gz_resources,
        gz_server,
        gz_client,
        clock_bridge,
        OpaqueFunction(function=launch_setup),
    ])
