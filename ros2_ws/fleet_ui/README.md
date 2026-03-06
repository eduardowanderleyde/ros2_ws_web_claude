# Fleet UI (React + FastAPI)

Interface web para o fleet: mapa clicável (go_to_point), botões de rota e status ao vivo.

## Pré-requisitos

- ROS 2 Jazzy com workspace buildado (`colcon build --merge-install`)
- Fleet em execução: `ros2 launch fleet_orchestrator fleet.launch.py` (ou com `--params-file` para sim)

## Backend (FastAPI)

O backend precisa do ambiente ROS 2 (workspace sourceado) para assinar `/fleet/status` e chamar os services.

```bash
cd /caminho/para/ros2_ws/ros2_ws
source install/setup.bash

# Opcional: venv para FastAPI
python3 -m venv fleet_ui/backend/venv
source fleet_ui/backend/venv/bin/activate  # ou .\venv\Scripts\activate no Windows
pip install -r fleet_ui/backend/requirements.txt

# Sobe o backend (porta 8000)
python fleet_ui/backend/main.py
```

Ou com uvicorn direto (com o mesmo `source install/setup.bash` no mesmo terminal):

```bash
uvicorn fleet_ui.backend.main:app --host 0.0.0.0 --port 8000
```

(Se o backend não estiver no PYTHONPATH, rode de dentro de `fleet_ui/backend`: `uvicorn main:app --host 0.0.0.0 --port 8000`.)

## Frontend (React + Vite)

```bash
cd fleet_ui/frontend
npm install
npm run dev
```

Abre http://localhost:5173. O Vite faz proxy de `/api` e `/ws` para o backend em 8000.

## Uso

1. **Mapa**: clique para definir (x, y) em metros; depois "Ir para ponto" envia `go_to_point` para o robô selecionado.
2. **Robô**: seleção pelo dropdown (preenchido por `/api/status` ou `list_robots`).
3. **Rotas**: campo "Rota" + "Iniciar gravação" / "Parar gravação" / "Reproduzir rota".
4. **Coleta**: "Ligar coleta" / "Desligar coleta".
5. **Status**: painel à direita com estado de navegação, rota atual e coleta (atualizado por WebSocket ou polling).

## Variável de ambiente

- `FLEET_WS`: caminho absoluto do workspace ROS 2 (opcional; por padrão usa o diretório acima de `fleet_ui`).
