# Documentação Técnica — Fleet Repeatability Framework

## Visão geral

Sistema ROS 2 para experimentos de repetibilidade de navegação autônoma com TurtleBot3. Permite gravar percursos, reproduzi-los múltiplas vezes e medir a consistência das trajetórias via métricas como RMSE, comprimento de percurso e análise de sensores.

---

## Arquitetura

```
┌─────────────────────────────────────────────────────┐
│                     Fleet UI                        │
│  React (Vite) ←WebSocket/REST→ FastAPI Backend      │
│                        │                            │
│              POST /api/run_config                   │
│              GET  /api/job/{id}                     │
│              WS   /ws/status                        │
└────────────────────┬────────────────────────────────┘
                     │ subprocess + rclpy
┌────────────────────▼────────────────────────────────┐
│         experiment_repeatability.py                 │
│         (record | replay via serviços ROS 2)        │
└──────┬────────────────────────────┬─────────────────┘
       │ ROS 2 services             │ ROS 2 services
┌──────▼──────────┐      ┌──────────▼────────────────┐
│ fleet_orchestrator│    │  fleet_data_collector      │
│  - start_record  │    │   - enable_collection       │
│  - stop_record   │    │   - disable_collection      │
│  - play_route    │    │   - grava rosbag2 (MCAP)    │
│  - go_to_point   │    └───────────────────────────-─┘
│  - /fleet/status │
│  - Nav2 action   │
└─────────────────-┘
```

---

## Pacotes ROS 2

### `fleet_msgs`

Define todas as interfaces (srv/msg) usadas entre os nós:

| Interface | Tipo | Uso |
|-----------|------|-----|
| `StartRecord` | srv | Inicia gravação de rota |
| `StopRecord` | srv | Para gravação e salva YAML |
| `PlayRoute` | srv | Reproduz rota gravada via Nav2 |
| `GoToPoint` | srv | Envia robô a uma pose (x, y, yaw) |
| `Cancel` | srv | Cancela navegação em curso |
| `ListRobots` | srv | Lista robot_ids registrados |
| `ListRoutes` | srv | Lista rotas de um robot_id |
| `EnableCollection` | srv | Inicia gravação de rosbag2 |
| `DisableCollection` | srv | Para gravação e retorna path do bag |
| `CollectionStatus` | srv | Estado da coleta |
| `FleetStatus` | msg | Estado agregado publicado a 1 Hz |

---

### `fleet_orchestrator`

**Arquivo principal:** `src/fleet_orchestrator/fleet_orchestrator/main.py`

Nó que gerencia rotas e navega via Nav2.

**Serviços expostos:**
- `start_record` — define rota ativa, inicia registro de poses via TF
- `stop_record` — salva poses acumuladas em `routes/<robot_id>/<route>.yaml`
- `play_route` — lê YAML e envia waypoints para Nav2 (`NavigateThroughPoses`)
- `go_to_point` — envia goal direto (x, y, yaw) ao Nav2
- `cancel` — cancela action Nav2 em curso
- `list_robots`, `list_routes`

**Tópico publicado:**
- `/fleet/status` (`FleetStatus`) a 1 Hz — `nav_state`, `current_route`, `collection_on`, `last_error`

**Estados de navegação:**
```
idle → navigating → idle        (go_to_point / play_route)
idle → recording → recording    (start_record + go_to_point)
recording → idle                (stop_record)
* → failed                      (Nav2 rejeitou / TF ausente)
```

**Configuração single-robot (`config/single_robot_sim.yaml`):**
```yaml
robots: ['']           # robot_id vazio → sem namespace
use_shared_map_frame: true  # usa frame "map" sem prefixo
```

---

### `fleet_data_collector`

**Arquivo principal:** `src/fleet_data_collector/fleet_data_collector/main.py`

Nó que grava rosbag2 (formato MCAP) por robot_id.

**Tópicos suportados:**

```python
_TYPE_MAP = {
    "scan": ("sensor_msgs/msg/LaserScan", LaserScan),
    "odom": ("nav_msgs/msg/Odometry",     Odometry),
    "imu":  ("sensor_msgs/msg/Imu",       Imu),
}
```

Para `robot_id=""`, grava `/scan`, `/odom`, `/imu`.  
Para `robot_id="tb1"`, grava `/tb1/scan`, `/tb1/odom`, `/tb1/imu`.

**Bags salvos em:** `collections/<robot_id_ou_default>/<timestamp>_<uuid>/`

---

## Scripts

### `experiment_repeatability.py`

Script principal de experimentos. Dois subcomandos:

#### `record`

1. Publica `/initialpose` (AMCL/SLAM) na pose inicial
2. Chama `enable_collection` (inicia rosbag2)
3. Chama `start_record`
4. Para cada waypoint: chama `go_to_point`, aguarda `nav_state=recording`
5. Chama `stop_record` (salva YAML)
6. Chama `disable_collection` (fecha bag)
7. Lê bag e calcula `sensor_summary` + `bag_metrics`
8. Exporta JSON com metadados (`--export`)

**Parâmetros principais:**
```
--route NAME          nome da rota
--points "x,y,yaw;…"  waypoints separados por ";"
--topics scan odom imu
--initial-pose x,y,yaw
--single-robot        usa robot_id=""
--export path.json    exporta metadados
```

#### `replay`

1. (Opcional) `go_to_point` para retornar ao início (`--return-to-start`)
2. Publica `/initialpose`
3. Chama `enable_collection`
4. Chama `play_route` (Nav2 reproduz waypoints do YAML)
5. Aguarda `nav_state=idle`
6. Chama `disable_collection`
7. Exporta JSON com métricas do bag

---

### `analyze_runs.py`

Pós-processamento de múltiplos bags. Lê `/odom` (ou `/amcl_pose`) e calcula:

| Métrica | Descrição |
|---------|-----------|
| `num_poses` | Número de poses lidas |
| `duration_sec` | Duração total do bag |
| `path_length_m` | Comprimento acumulado da trajetória |
| `start_xy`, `end_xy` | Posição inicial e final |
| `pairwise_rmse_m` | RMSE entre todos os pares de runs |
| `rmse_vs_ref_m` | RMSE de cada run vs. referência (1º bag) |
| `mean_pointwise_distance_vs_ref_m` | Distância média ponto a ponto vs. ref |
| `final_endpoint_error_m` | Erro no ponto final vs. ref |
| `duration_ratio_vs_ref` | Razão de duração vs. ref |

**Saídas:** `summary.json`, `trajectory_overlay.png`, CSVs por run.

---

### `_bag_sensor_summary(bag_path)`

Lê metadados do bag (sem deserializar mensagens) e retorna:

```json
{
  "/scan": {"msgs": 271, "hz_est": 10.0},
  "/odom": {"msgs": 1354, "hz_est": 50.0},
  "/imu":  {"msgs": 5412, "hz_est": 200.0}
}
```

### `_bag_compute_metrics(bag_path)`

Deserializa mensagens e calcula:

| Métrica | Fonte | Significado |
|---------|-------|-------------|
| `duration_s` | metadata | Duração real da gravação |
| `odom_path_length_m` | `/odom` pose.pose.position | Comprimento integrado do percurso |
| `odom_avg_speed_ms` | path / duration | Velocidade média |
| `scan_avg_valid_points` | `/scan` ranges | Média de pontos LiDAR válidos por scan |
| `scan_min_valid_points` | `/scan` | Mínimo de pontos válidos (detecta oclusão) |
| `imu_accel_mean_ms2` | `/imu` linear_acceleration | Norma média da aceleração linear (≈9.8 em repouso) |
| `imu_accel_variance_ms2` | `/imu` | Variância da norma — mede suavidade do movimento |

**IMU variância:** valores baixos (< 0.01) indicam movimento suave e controlado. Valores altos indicam arranques bruscos ou trepidação.

---

## Backend da UI (`fleet_ui/backend/main.py`)

FastAPI com:

- `GET /api/status` — estado atual (`/fleet/status` + pose TF)
- `WS /ws/status` — streaming a 250 ms
- `POST /api/run_config` — inicia job assíncrono (record/replay)
- `GET /api/job/{id}` — polling de output e resultado
- `POST /api/go_to_point` — navega para (x,y,yaw)
- `GET /api/discover_robots` — varre subnet por SSH
- `POST /api/test_ssh` — verifica ROS 2 em host remoto

**Job system:** cada `run_config` lança um `subprocess.Popen` em thread separada com `PYTHONUNBUFFERED=1`. As linhas de output são filtradas (ruído TF) e acumuladas em `_jobs[job_id]["lines"]`. Ao final, lê o JSON de export e inclui em `result`.

**Pose do robô:** TF `map → base_footprint` via `tf2_ros.Buffer` em timer de 100 ms.

---

## Frontend da UI (`fleet_ui/frontend/src/App.jsx`)

React (Vite) com:

- **Barra de conexão** — "Conectar robô" (Custom/SSH) + "Procurar robô" (varredura SSH)
- **Coluna esquerda** — JSON config + botões de exemplo + Executar/Parar
- **Coluna direita** — output ao vivo com cores + cards de métricas + tabela de sensores + modal JSON
- **Header** — Nav state, Coleta ON/OFF, Pose x/y/yaw, botão Reiniciar

**Cores no output:**
- `[OK]` → verde
- `[FALHOU]` → vermelho
- `[TRACE]` → cinza
- `Resumo` → amarelo
- `===` → azul

---

## Fluxo completo de um experimento

```
1. record (UI ou CLI)
   ├── enable_collection → bag aberto
   ├── start_record
   ├── go_to_point wp1 … wpN → poses registradas via TF
   ├── stop_record → routes/default/percurso1.yaml
   └── disable_collection → bag fechado

2. replay × N (baseline + replays)
   ├── go_to_point(0,0,0) → retorno ao início
   ├── enable_collection → novo bag
   ├── play_route → Nav2 reproduz waypoints do YAML
   ├── aguardar idle
   └── disable_collection → bag fechado

3. analyze_runs.py bag_baseline bag_r1 bag_r2
   └── summary.json + PNG + CSV
```

---

## Parâmetros de qualidade da repetibilidade

| Indicador | Boa repetibilidade | Aceitável |
|-----------|-------------------|-----------|
| RMSE vs referência | < 0.05 m | < 0.15 m |
| Erro de endpoint | < 0.05 m | < 0.15 m |
| Razão de duração | 0.98 – 1.02 | 0.90 – 1.10 |
| IMU variância | < 0.005 | < 0.02 |
| Scan pts válidos | > 300 | > 200 |
