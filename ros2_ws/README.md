# ros2_ws — Workspace ROS 2 Jazzy (Linux)

Workspace ROS 2 Jazzy com **controle de robôs** e **coleta de dados** separados, prontos para UI.

## Estrutura (controle vs coleta)

- **Controle de robôs (rotas)** → `fleet_orchestrator`: record/save/play por `robot_id`, envio de goals Nav2.
- **Coleta de dados** → `fleet_data_collector`: ligar/desligar coleta de sensores (LiDAR, odom, imu) por `robot_id`.
- **Interfaces comuns** → `fleet_msgs`: serviços e mensagens usados por ambos (e pela UI depois).

Cada TurtleBot continua rodando no robô apenas o básico (drivers, Nav2, TF). Esta ferramenta roda no **PC central** e só consome/publica via ROS 2.

## Arquitetura: multi-mapa (fleet controller)

Cada robô tem **seu próprio SLAM e seu próprio mapa**. Não há mapa único compartilhado.

- **TF por robô:** `tb1/map`, `tb1/odom`, `tb1/base_link` — e idem para `tb2`, `tb3`.
- **Rotas não são compartilháveis:** uma rota em `routes/tb1/r1.yaml` é só do tb1 (frame `tb1/map`). Não serve para tb2.
- **Orchestrator** usa sempre frames e actions por robô:
  - `lookup_transform("tb1/map", "tb1/base_link")`
  - action `/tb1/navigate_through_poses`
- **Data collector** assina `/tb1/scan`, `/tb1/odom`, etc., e salva em `collections/tb1/<timestamp>/`.

Na UI, robôs são **instâncias independentes**: tabela por robô (map active, route, nav status, collection), sem nada global.

**Crítico:** cada robô deve rodar em namespace completo (`/tb1/...`). SLAM publica em `tbX/map`, Nav2 usa `tbX/map`. Conflito em `/tf` quebra tudo.

## Estrutura de pastas

```
ros2_ws/
  src/
    fleet_msgs/           # .srv comuns (UI + nós)
    fleet_orchestrator/   # record/save/play + Nav2 por robot_id
    fleet_data_collector/ # enable/disable coleta por robot_id (rosbag2)
    route_tool/           # legado: um robô, Trigger (mantido se quiser)
```

## Pacotes

| Pacote | Descrição |
|--------|-----------|
| **fleet_msgs** | `StartRecord`, `StopRecord`, `PlayRoute`, `GoToPoint`, `Cancel`, `ListRobots`, `ListRoutes`, `EnableCollection`, `DisableCollection`, `CollectionStatus` |
| **fleet_orchestrator** | Nó que expõe start_record, stop_record, play_route, **go_to_point** (ir para um ponto x,y), cancel, list_robots, list_routes (por `robot_id`) |
| **fleet_data_collector** | Nó que expõe enable_collection, disable_collection, collection_status (por `robot_id`); grava em **rosbag2** |
| **route_tool** | Nó legado (um robô, services Trigger) |

## Coleta: rosbag2 vs CSV

- **Implementado:** `output_mode = "rosbag2"`. Cada sessão vira uma pasta em `collections/<robot_id>/<timestamp>/` (mcap).
- **CSV:** não implementado; pode ser adicionado depois em `fleet_data_collector` tratando `output_mode == "csv"` e escrevendo CSVs por tópico/sessão.

## Build e run

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fleet_msgs fleet_orchestrator fleet_data_collector
source install/setup.bash
# Se o ros2 run não achar o pacote, use:
# export AMENT_PREFIX_PATH=$(pwd)/install/fleet_orchestrator:$(pwd)/install/fleet_data_collector:$AMENT_PREFIX_PATH
```

Terminal 1 – orquestrador:

```bash
ros2 run fleet_orchestrator fleet_orchestrator
```

Terminal 2 – coletor:

```bash
ros2 run fleet_data_collector sensor_collector
```

**Ou subir tudo com o launch (sem vários terminais):**

```bash
ros2 launch fleet_orchestrator fleet.launch.py
# Para sim 1 robô sem namespace (map, base_link, /navigate_through_poses):
# use config YAML ou ros2 run com -p robots:="['']" -p use_shared_map_frame:=true
```

### Modo single-robot (simulação sem namespace)

Se o simulador usa `map`, `base_link` e `/navigate_through_poses` (sem prefixo tb1/):

- **Orchestrator:** `robots:=['']`, `use_shared_map_frame:=true`. Rotas em `routes/default/`.
- **Collector:** `robots:=['']`. Tópicos `/scan`, `/odom`; bags em `collections/default/`.
- Nos services use `robot_id: ''` (string vazia). `list_robots` retorna `['']` ou exibe como "default".

**Config YAML pronta para sim:** `fleet_orchestrator/config/single_robot_sim.yaml` (robots: [''], use_shared_map_frame: true). Após o build, use com o launch: `ros2 launch fleet_orchestrator fleet.launch.py --params-file install/fleet_orchestrator/share/fleet_orchestrator/config/single_robot_sim.yaml` ou rode os nós com `--params-file <path>/single_robot_sim.yaml`.

**Script de teste:** `python3 scripts/test_fleet_cases.py --single-robot` usa `robot_id: ''` em todos os testes.

## Erros padronizados (error_code)

Os services retornam `success`, `message` e **`error_code`** (string; vazio se ok). A UI pode tratar por código:

- **Orchestrator:** `UNKNOWN_ROBOT`, `ALREADY_NAVIGATING`, `ALREADY_RECORDING`, `ROUTE_NOT_FOUND`, `NAV2_UNAVAILABLE`, `NAV2_REJECTED`, `TF_MISSING`, `ROLE_NOT_ALLOWED` (movimento só para papel **MUUT**), `CANCEL_FAILED`
- **Collector:** `UNKNOWN_ROBOT`, `ALREADY_COLLECTING`, `UNSUPPORTED_OUTPUT_MODE`, `NO_VALID_TOPICS`, `BAG_PATH_EXISTS`, `ENABLE_FAILED`, `DISABLE_FAILED`

O tópico `/fleet/status` inclui `last_error` (código ou vazio) e `nav_state=failed` quando há erro (ex.: TF ausente, Nav2 rejeitou).

## Tópico /fleet/status (para a UI)

O orchestrator publica **`/fleet/status`** (tipo `fleet_msgs/msg/FleetStatus`) a 1 Hz com estado agregado por robô:

- `robot_id`, **`role`** (MUUT | FUUT | SU), `nav_state` (idle | recording | navigating | failed), `current_route`
- `collection_on`, `collection_file`, `bytes_written`, `last_error`

A UI pode **assinar só esse tópico** e chamar services quando o usuário agir; evita polling em vários services.

## Exemplos de chamadas (API para UI)

Robots padrão: `tb1`, `tb2`, `tb3`. Troque por seus namespaces.

**Rotas (orchestrator):**

```bash
ros2 service call /start_record fleet_msgs/srv/StartRecord "{robot_id: 'tb1', route_name: 'r1'}"
ros2 service call /stop_record fleet_msgs/srv/StopRecord "{robot_id: 'tb1'}"
ros2 service call /play_route fleet_msgs/srv/PlayRoute "{robot_id: 'tb1', route_name: 'r1'}"
ros2 service call /go_to_point fleet_msgs/srv/GoToPoint "{robot_id: 'tb1', x: 1.0, y: 0.5, yaw: 0.0}"
ros2 service call /cancel fleet_msgs/srv/Cancel "{robot_id: 'tb1'}"
ros2 service call /list_robots fleet_msgs/srv/ListRobots "{}"
ros2 service call /list_routes fleet_msgs/srv/ListRoutes "{robot_id: 'tb1'}"
```

**Coleta (data collector):**

```bash
ros2 service call /enable_collection fleet_msgs/srv/EnableCollection "{robot_id: 'tb1', topics: ['scan', 'odom', 'imu'], output_mode: 'rosbag2'}"
ros2 service call /disable_collection fleet_msgs/srv/DisableCollection "{robot_id: 'tb1'}"
ros2 service call /collection_status fleet_msgs/srv/CollectionStatus "{robot_id: 'tb1'}"
```

## Parâmetros

- **fleet_orchestrator:** `robots`, `routes_dir`, `record_rate_hz`, `min_dist_m`, `min_yaw_deg`, `nav2_action_suffix`, `frame_map_suffix`, `frame_base_suffix`, **`use_shared_map_frame`** (bool, default false). Se o SLAM publica `map` sem prefixo, use `use_shared_map_frame:=true` para TF `map` → `tb1/base_link` (senão usa `tb1/map` → `tb1/base_link`).
- **fleet_data_collector:** `robots`, `collections_dir`. Tópicos conhecidos: `scan`, `odom`, `imu` (nomes relativos ao namespace do robô).

## Diagnóstico TF (quando record fica vazio)

Se o record sempre grava 0 pontos e o log diz `TF failed ... target_frame does not exist`, **não existe frame de mapa** no TF. Descubra quais frames existem com o stack do tb1 ligado (SLAM/Nav2 do robô rodando):

**1) TF básico (odom → base_link)**  
Se funcionar, o robô está publicando TF no namespace certo.

```bash
ros2 run tf2_ros tf2_echo odom tb1/base_link
```

**2) Snapshot do TF (nomes reais dos frames)**  
Procure por `map` em `header.frame_id` ou `child_frame_id`. Se não existir, SLAM/localização não está ativo ou usa outro nome.

```bash
ros2 topic echo /tf --once
```

**3) Nós de SLAM/localização**  
Confirme que algo de slam/amcl/map_server está rodando.

```bash
ros2 node list | egrep "slam|amcl|map|nav2"
```

**Interpretação:** Se existir `map -> odom` (ou `tb1/map -> tb1/odom`), use esse frame no orchestrator (`use_shared_map_frame` ou frames prefixados). Se só existir `odom -> tb1/base_link`, falta subir SLAM/localização para ter `map`.

## Validação antes da UI (obrigatória)

Você precisa comprovar **3 invariantes** no ambiente real. Se algum falhar, a UI vai parecer que “não funciona”.

### 1) TF por robô existe e é estável

Se falhar, **record grava lixo**.

```bash
ros2 run tf2_ros tf2_echo tb1/map tb1/base_link
ros2 run tf2_ros tf2_echo tb2/map tb2/base_link
```

(Opcional: conferir que os tópicos de mapa existem por namespace: `ros2 topic list | grep tb1/map`, `ros2 topic list | grep tb2/map`.)

### 2) Action do Nav2 existe por robô

Se falhar, **play_route não faz nada**.

```bash
ros2 action list | grep navigate_through_poses
```

Deve aparecer `/tb1/navigate_through_poses`, `/tb2/...`, etc. (conforme seus namespaces.)

### 3) Coletor assina e grava bag sem travar

enable → espera 5–10 s → disable → conferir bag.

```bash
ros2 service call /enable_collection fleet_msgs/srv/EnableCollection "{robot_id: 'tb1', topics: ['scan', 'odom'], output_mode: 'rosbag2'}"
# esperar 5–10 s
ros2 service call /disable_collection fleet_msgs/srv/DisableCollection "{robot_id: 'tb1'}"
ros2 bag info collections/tb1/<timestamp>   # usar o dir que o disable retornou
```

Se travar no enable/disable, a UI vai “ligar coleta” e nada acontece.

### Conferir que a API está exposta

```bash
ros2 service list | grep -E "start_record|stop_record|play_route|go_to_point|enable_collection|disable_collection"
```

Deve listar os services do orchestrator e do collector.

---

## Checklist final antes de começar a UI

- [ ] **TF:** `tf2_echo tbX/map tbX/base_link` ok para cada robô.
- [ ] **Actions:** `ros2 action list | grep navigate_through_poses` mostra `/tb1/...`, `/tb2/...`, etc.
- [ ] **Record/Play:** start_record → anda → stop_record → gera YAML; play_route executa no Nav2.
- [ ] **Coleta:** enable_collection → espera → disable_collection → `ros2 bag info` no diretório ok.
- [ ] **Services:** `ros2 service list` inclui os services do fleet (start_record, play_route, enable_collection, etc.).

Só depois disso: implementar UI (recomendado: Web UI com FastAPI + WebSocket).

---

## Ordem certa para validar (3 terminais)

O problema agora **não é mais implementação** e sim **execução/validação**: subir os nós na ordem certa antes do teste. Abaixo, a sequência para **TurtleBot 3 + Jazzy + fleet**, pronta para colar.

### Terminal 1 — Simulação (Gazebo + depois SLAM + Nav2)

**1.1 — Mundo e robô no Gazebo (TurtleBot 3):**

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

(Outros mundos: `turtlebot3_empty_world.launch.py`, `turtlebot3_house.launch.py` — ver [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).)

**1.2 — Em outros terminais, subir SLAM e Nav2** (simulação, SLAM e Nav2 costumam ser lançados em etapas separadas). Ex.: `slam_toolbox` ou Cartographer para SLAM; em seguida Nav2 (navigation2). Consultar a doc do TurtleBot3 para ROS 2 Jazzy ([Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)). Só depois disso rodar o fleet.

Se estiver usando outro stack/simulador, troque o launch acima pelo equivalente do seu ambiente; o importante é ter **map**, **base_link** e **/navigate_through_poses** antes do fleet.

### Terminal 2 — Fleet (modo single-robot)

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fleet_orchestrator fleet.launch.py use_shared_map_frame:=true
```

Para `robots: ['']` e demais parâmetros do arquivo `fleet_orchestrator/config/single_robot_sim.yaml`, passe-os nos nós com `ros2 run ... --ros-args --params-file $(ros2 pkg prefix fleet_orchestrator)/share/fleet_orchestrator/config/single_robot_sim.yaml` ou ajuste o `fleet.launch.py` para incluir esse YAML (o `ros2 launch` não aceita `--params-file` como flag global da mesma forma que `ros2 run`).

### Terminal 3 — Teste

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/test_fleet_cases.py --single-robot
```

**Objetivo:** services disponíveis, `/fleet/status` publicando, start_record gravando pontos, play_route e go_to_point executando, coleta gerando bag válido. Só depois disso começar a UI. Validação manual primeiro; não misturar problema do simulador com problema do fleet.

### Caminho 3 — Multi-robô (3× TurtleBot3 com namespaces `tb1`, `tb2`, `tb3`)

Use este fluxo para **validar papéis MUUT / FUUT / SU de verdade** com o mesmo hardware (TurtleBot3), **um namespace por robô**. A distinção é o **papel** (`roles.yaml`), não o modelo.

**O que você precisa no ROS 2 antes do fleet**

- Cada robô publica TF no seu namespace: `tbX/map`, `tbX/odom`, `tbX/base_link`.
- Nav2 por robô: action `/tbX/navigate_through_poses`.
- Tópicos de sensores: `/tbX/scan`, `/tbX/odom`, …

O pacote oficial do TurtleBot3 costuma trazer **um robô** por launch. Para **três robôs nomeados** no Gazebo, é comum usar um projeto de sim multi-robô (ex.: comunidade [`tb3_multi_robot`](https://github.com/arshadlab/tb3_multi_robot) / `turtlebot3_multi_robot`) ou um launch seu que:

- instancia 3 modelos com `namespace=tb1|tb2|tb3`,
- evita colisão de nomes em `/tf` e nos tópicos.

**Papéis (já no repositório)**

Arquivo: `src/fleet_orchestrator/config/roles.yaml` (instalado em `share/fleet_orchestrator/config/roles.yaml`).

| ID   | Papel | Movimento no fleet |
|------|-------|---------------------|
| tb1  | MUUT  | permitido (record, play, go_to_point) |
| tb2  | FUUT  | bloqueado no orchestrator (`ROLE_NOT_ALLOWED`); só coleta |
| tb3  | SU    | idem FUUT; papel lógico de infra/rede no experimento |

**FUUT/SU no Gazebo:** fisicamente o modelo pode se mover se você aplicar velocidade fora do fleet; o **fleet** só **não envia** goals Nav2 para esses IDs. No experimento, você deixa esses robôs parados (teleop zero / sem cmd_vel).

**Terminal 1 — Simulação + SLAM/Nav2 para os 3 namespaces**

Suba **o seu** launch multi-robô (3 namespaces) + stacks de localização/navegação até existirem, para cada `tbX`:

```bash
ros2 action list | grep navigate_through_poses
# esperado: /tb1/navigate_through_poses, /tb2/..., /tb3/...
```

**Terminal 2 — Fleet (multi-mapa: um mapa por robô)**

Use `use_shared_map_frame:=false` (padrão) para TF `tbX/map` → `tbX/base_link`:

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fleet_orchestrator fleet.launch.py use_shared_map_frame:=false
```

(O launch já define `robots: [tb1, tb2, tb3]` e o orchestrator carrega `config/roles.yaml`.)

**Validação rápida (TF + status)**

```bash
ros2 run tf2_ros tf2_echo tb1/map tb1/base_link
ros2 run tf2_ros tf2_echo tb2/map tb2/base_link
ros2 run tf2_ros tf2_echo tb3/map tb3/base_link
ros2 topic echo /fleet/status --once
```

**Terminal 3 — Teste automático por papéis (sem UI)**

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/test_roles_automatic.py --muut tb1 --fuut tb2 --su tb3
```

**Obs. DDS:** com vários participantes no mesmo host, às vezes é preciso ajustar CycloneDDS/Fast-DDS (limites de participantes); se um robô “some” do `ros2 node list`, verifique isso antes de culpar o fleet.

### Teste automático por papéis (MUUT/FUUT/SU), sem UI

Com `tb1=MUUT`, `tb2=FUUT`, `tb3=SU` (definidos em `src/fleet_orchestrator/config/roles.yaml`):

```bash
cd /home/eduardo/Documentos/ros2_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/test_roles_automatic.py --muut tb1 --fuut tb2 --su tb3
```

Esse script faz automaticamente:
- habilita coleta nos 3 robôs;
- valida que `FUUT`/`SU` não podem mover (`ROLE_NOT_ALLOWED`);
- no `MUUT`: `start_record` → sequência de `go_to_point` → `stop_record` → `play_route`;
- desabilita coleta nos 3 e verifica status final.

---

## O que falta para "terminar"

### Backend funcional (sem UI)

1. **Ambiente de teste real** — Subir 1 TurtleBot simulado com SLAM/Nav2 e validar TF, actions, record, go_to_point.
2. **Fleet para 1 robô simulado** — Já suportado: `robots:=['']`, frames `map`/`base_link`; rotas em `routes/default/`, coleta em `collections/default/`.
3. **Teste end-to-end passar** — Fluxo: start_record → stop_record → YAML → list_routes → play_route → go_to_point → enable/disable_collection → `ros2 bag info`.
4. **Launch de bringup** — Feito: `ros2 launch fleet_orchestrator fleet.launch.py`.

### Pronto para UI

5. **Tópico /fleet/status** — Feito. UI assina e evita polling.
6. **Padronizar erros** — Opcional: `error_code` nos .srv; UI trata por código.
7. **UX mínima** — Seletor de robô; botões record/play/cancel; go_to_point (x,y,yaw); enable/disable collection; lista de rotas; status ao vivo.


---

## Estado do projeto (backend)

O backend está **documentado e executável**. Inclui: orchestrator, data collector, services completos, `go_to_point`, record/play, error codes, `/fleet/status`, modo single-robot, arquitetura multi-robô, launch, config YAML e script de testes. A separação (robot stack → fleet controller → data collector → UI) está alinhada com sistemas de fleet robotics (Open-RMF, Isaac Mission Dispatch, etc.).

Falta: **(1)** validar o fluxo completo em simulação (os 3 terminais do README); **(2)** depois, criar a UI.

---

## Próximos passos (ordem recomendada)

1. **Validar em simulação** — Rodar os 3 terminais; conferir start_record → mover robô → stop_record → `routes/default/r1.yaml` → list_routes → play_route → go_to_point → enable/disable_collection → `ros2 bag info` → `ros2 topic echo /fleet/status`. Se tudo passar → backend validado.
2. **Demo em vídeo** — Gravar o sistema funcionando (prova de conceito / portfolio / mestrado).
3. **UI web** — Implementada em `fleet_ui/`: ver [Fleet UI](#fleet-ui-react--fastapi) abaixo.

---

## Fleet UI (React + FastAPI)

Interface web em `fleet_ui/`: mapa clicável (go_to_point), botões de rota e status ao vivo.

- **Backend**: FastAPI que assina `/fleet/status` (ROS 2) e expõe REST + WebSocket. Rode com o workspace sourceado:
  ```bash
  source install/setup.bash
  pip install -r fleet_ui/backend/requirements.txt   # opcional: venv
  python fleet_ui/backend/main.py
  ```
- **Frontend**: React (Vite) em `fleet_ui/frontend/`. Proxy para API em `localhost:8000`.
  ```bash
  cd fleet_ui/frontend && npm install && npm run dev
  ```
- Abra http://localhost:5173. Detalhes em `fleet_ui/README.md`.
