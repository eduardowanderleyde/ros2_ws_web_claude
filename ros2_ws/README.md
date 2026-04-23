# Fleet Repeatability — ROS 2 Jazzy

Framework para gravação e reprodução repetível de percursos com TurtleBot3, com coleta de sensores e análise de métricas.

---

## Pré-requisitos

- ROS 2 Jazzy instalado
- TurtleBot3 Waffle (`export TURTLEBOT3_MODEL=waffle`)
- Nav2 + SLAM Toolbox + `opennav_docking`
- Python 3: `numpy`, `matplotlib`

**Build:**
```bash
cd ~/Documentos/ros2_ws/ros2_ws
colcon build --merge-install
source install/setup.bash
```

---

## Iniciar o sistema (multi-robô)

Abre 3 terminais:

```bash
# T1 — Simulação completa (Gazebo + Nav2 + SLAM assíncrono para tb1 e tb2)
export TURTLEBOT3_MODEL=waffle
cd ~/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch fleet_bringup multi_robot_sim.launch.py

# Opcional: sem GUI do Gazebo (economiza CPU/RAM)
# ros2 launch fleet_bringup multi_robot_sim.launch.py headless:=true

# T2 — Backend da UI
cd ~/Documentos/ros2_ws/ros2_ws/fleet_ui/backend
bash run.sh

# T3 — Frontend da UI
cd ~/Documentos/ros2_ws/ros2_ws/fleet_ui/frontend
npm run dev
```

Acesse **http://localhost:5173**

### Sequência de inicialização do launch

| Tempo | O que sobe |
|-------|-----------|
| 0 s   | Gazebo server (físico + sensores) |
| 3 s   | Clock bridge dedicado (`/clock`) |
| 5 s   | Spawn dos modelos tb1 e tb2 |
| 6 s   | Bridges: odom / scan / imu / cmd_vel / tf / joint_states |
| 10 s  | Nav2 (sem SLAM interno — `slam:=False`) para cada robô |
| 12 s  | SLAM Toolbox assíncrono (`online_async_launch.py`) para cada robô |

---

## Enviar navegação via curl

**Gravar percurso:**
```bash
curl -s -X POST http://localhost:8000/record \
  -H "Content-Type: application/json" \
  -d '{"robot_id":"tb1","route_name":"percurso1","points":[{"x":0.5,"y":0.0,"yaw":0.0},{"x":1.0,"y":0.0,"yaw":0.0}]}'
```

**Reproduzir percurso:**
```bash
curl -s -X POST http://localhost:8000/replay \
  -H "Content-Type: application/json" \
  -d '{"robot_id":"tb1","route_name":"percurso1"}'
```

---

## Analisar resultados

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

python3 scripts/analyze_runs.py \
  collections/tb1/baseline_bag \
  collections/tb1/replay_01_bag \
  --labels baseline replay_01 \
  --output-dir analysis_resultado/
```

Gera em `analysis_resultado/`:
- `summary.json` — RMSE, comprimento, erro de endpoint
- `trajectory_overlay.png` — trajetórias sobrepostas
- `trajectories_csv/` — coordenadas por run

---

## Diagnóstico rápido

```bash
# SLAM assíncrono rodando?
ros2 node list | grep slam_toolbox

# scan_queue_size carregado (deve ser 1)
ros2 param get /tb1/slam_toolbox scan_queue_size
ros2 param get /tb2/slam_toolbox scan_queue_size

# Clock único
ros2 topic info /clock -v

# TF namespaceado
ros2 topic echo /tb1/tf_static --once
ros2 topic echo /tb1/tf --once

# Mapa sendo gerado
ros2 topic echo /tb1/map --once --no-arr 2>&1 | head -10
```

---

## Estrutura do repositório

```
scripts/
  experiment_repeatability.py   # CLI de record e replay
  analyze_runs.py               # análise de repetibilidade
routes/                         # percursos gravados por robot_id (.yaml)
collections/                    # rosbags gravados por robot_id (MCAP)
fleet_ui/                       # interface web (React + FastAPI)
src/
  fleet_bringup/                # launch files e params (Nav2 / bridge / SLAM)
  fleet_orchestrator/           # navegação e gestão de rotas
  fleet_data_collector/         # gravação de rosbag por robot_id
  fleet_msgs/                   # interfaces ROS 2 (srv/msg)
```

---

## Convenção de robot_id

| Modo | robot_id | Tópicos | Frames TF |
|------|----------|---------|-----------|
| Single | `""` | `/scan`, `/odom` | `map`, `base_link` |
| Multi | `"tb1"`, `"tb2"` | `/tb1/scan`, `/tb1/odom` | `tb1/map`, `tb1/base_link` |
