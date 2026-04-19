# Fleet Repeatability — ROS 2 Jazzy

Framework para gravação e reprodução repetível de percursos com TurtleBot3, com coleta de sensores e análise de métricas.

---

## Pré-requisitos

- ROS 2 Jazzy instalado
- TurtleBot3 Waffle (`export TURTLEBOT3_MODEL=waffle`)
- Nav2 + SLAM Toolbox
- Python 3: `numpy`, `matplotlib`

**Build:**
```bash
cd ~/Documentos/ros2_ws/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Iniciar o sistema

Abrir 5 terminais em ordem:

```bash
# T1 — Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2 — Nav2 + SLAM  ← aguardar "Nav2 is ready!"
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

# T3 — Fleet (orchestrator + collector)
cd ~/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch fleet_orchestrator fleet.launch.py single_robot_sim:=true

# T4 — Backend da UI
cd ~/Documentos/ros2_ws/ros2_ws/fleet_ui/backend
bash run.sh

# T5 — Frontend da UI
cd ~/Documentos/ros2_ws/ros2_ws/fleet_ui/frontend
npm run dev
```

Acesse **http://localhost:5173**

---

## Usar a interface

1. Clicar **Conectar robô → Custom → Conectar** (aguardar luz verde)
2. Colar JSON de configuração na área esquerda
3. Clicar **Executar**
4. Acompanhar output ao vivo; ver métricas e resultado JSON ao final

**JSON de gravação (record):**
```json
{
  "command": "record",
  "robot": "default",
  "route": "percurso1",
  "collect": true,
  "topics": ["scan", "odom", "imu"],
  "initial_pose": [0, 0, 0],
  "points": [
    [0.5, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.5, 0.5, 0.0],
    [2.0, 0.5, 0.0]
  ]
}
```

**JSON de replay:**
```json
{
  "command": "replay",
  "robot": "default",
  "route": "percurso1",
  "collect": true,
  "topics": ["scan", "odom", "imu"],
  "initial_pose": [0, 0, 0],
  "return_to_start": [0, 0, 0]
}
```

---

## Analisar resultados

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

python3 scripts/analyze_runs.py \
  collections/default/baseline_bag \
  collections/default/replay_01_bag \
  collections/default/replay_02_bag \
  --labels baseline replay_01 replay_02 \
  --output-dir analysis_resultado/
```

Gera em `analysis_resultado/`:
- `summary.json` — RMSE, comprimento, erro de endpoint
- `trajectory_overlay.png` — trajetórias sobrepostas
- `trajectories_csv/` — coordenadas por run

---

## Estrutura do repositório

```
scripts/
  experiment_repeatability.py   # CLI de record e replay
  analyze_runs.py               # análise de repetibilidade
routes/default/                 # percursos gravados (.yaml)
collections/default/            # rosbags gravados (MCAP)
fleet_ui/                       # interface web (React + FastAPI)
src/
  fleet_orchestrator/           # navegação e gestão de rotas
  fleet_data_collector/         # gravação de rosbag por robot_id
  fleet_msgs/                   # interfaces ROS 2 (srv/msg)
```

---

## Tópicos de sensores suportados

| Short name | Tópico ROS 2 | Tipo |
|------------|-------------|------|
| `scan`     | `/scan`     | `sensor_msgs/LaserScan` |
| `odom`     | `/odom`     | `nav_msgs/Odometry` |
| `imu`      | `/imu`      | `sensor_msgs/Imu` |
