"""
Backend FastAPI: bridge para o fleet (ROS 2).
Rode com o workspace sourceado: source install/setup.bash && python main.py
"""
from __future__ import annotations

import json
import os
import subprocess
import threading
from contextlib import asynccontextmanager
from pathlib import Path

import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# Caminho do workspace (para ros2 service call). Use FLEET_WS ou sobe 2 níveis a partir de backend/
WORKSPACE = os.environ.get("FLEET_WS") or str(Path(__file__).resolve().parent.parent.parent)

# Status da frota (atualizado pelo subscriber ROS em thread)
_fleet_status: dict = {"robots": []}
_status_lock = threading.Lock()
_ws_clients: list[WebSocket] = []


def _ros_env():
    return {
        **os.environ,
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
    }


def _run_ros2_service(srv: str, srv_type: str, request_json: str, timeout: int = 10) -> tuple[bool, str]:
    cmd = f"source /opt/ros/jazzy/setup.bash 2>/dev/null; source {WORKSPACE}/install/setup.bash 2>/dev/null; ros2 service call {srv} {srv_type} '{request_json}'"
    try:
        r = subprocess.run(
            ["bash", "-c", cmd],
            env=_ros_env(),
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=WORKSPACE,
        )
        out = (r.stdout or "").strip() + (r.stderr or "").strip()
        if r.returncode != 0:
            return False, out or "ros2 service call failed"
        return True, out
    except subprocess.TimeoutExpired:
        return False, "timeout"
    except Exception as e:
        return False, str(e)






@asynccontextmanager
async def lifespan(app: FastAPI):
    # Inicia thread do subscriber ROS (se rclpy disponível)
    def run_ros():
        try:
            import rclpy
            from rclpy.node import Node
            from fleet_msgs.msg import FleetStatus

            rclpy.init()
            node = Node("fleet_ui_bridge")
            def cb(msg):
                with _status_lock:
                    _fleet_status["robots"] = [
                        {
                            "robot_id": r.robot_id,
                            "nav_state": r.nav_state,
                            "current_route": r.current_route,
                            "collection_on": r.collection_on,
                            "collection_file": r.collection_file,
                            "last_error": r.last_error,
                            "bytes_written": r.bytes_written,
                        }
                        for r in msg.robots
                    ]

            node.create_subscription(FleetStatus, "fleet/status", cb, 10)
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"[fleet_ui] ROS subscriber not started (source workspace?): {e}")

    t = threading.Thread(target=run_ros, daemon=True)
    t.start()
    yield
    # shutdown


app = FastAPI(title="Fleet UI API", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/status")
async def get_status():
    with _status_lock:
        return _fleet_status


@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.append(websocket)
    try:
        with _status_lock:
            await websocket.send_text(json.dumps(_fleet_status))
        while True:
            await asyncio.sleep(1.0)
            with _status_lock:
                await websocket.send_text(json.dumps(_fleet_status))
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in _ws_clients:
            _ws_clients.remove(websocket)


@app.post("/api/start_record")
async def start_record(robot_id: str = "", route_name: str = "r1"):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "start_record", "fleet_msgs/srv/StartRecord",
        json.dumps({"robot_id": robot_id, "route_name": route_name}),
    )
    return {"success": ok, "message": out}


@app.post("/api/stop_record")
async def stop_record(robot_id: str = ""):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "stop_record", "fleet_msgs/srv/StopRecord",
        json.dumps({"robot_id": robot_id}),
    )
    return {"success": ok, "message": out}


@app.post("/api/play_route")
async def play_route(robot_id: str = "", route_name: str = "r1"):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "play_route", "fleet_msgs/srv/PlayRoute",
        json.dumps({"robot_id": robot_id, "route_name": route_name}),
    )
    return {"success": ok, "message": out}


@app.post("/api/go_to_point")
async def go_to_point(robot_id: str = "", x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "go_to_point", "fleet_msgs/srv/GoToPoint",
        json.dumps({"robot_id": robot_id, "x": x, "y": y, "yaw": yaw}),
    )
    return {"success": ok, "message": out}


@app.post("/api/cancel")
async def cancel(robot_id: str = ""):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "cancel", "fleet_msgs/srv/Cancel",
        json.dumps({"robot_id": robot_id}),
    )
    return {"success": ok, "message": out}


@app.get("/api/list_robots")
async def list_robots():
    ok, out = _run_ros2_service("list_robots", "fleet_msgs/srv/ListRobots", "{}")
    if not ok:
        return JSONResponse(content={"robot_ids": []}, status_code=200)
    try:
        # Parse YAML-like output from ros2 service call
        import yaml
        data = yaml.safe_load(out) if out else {}
        robot_ids = data.get("robot_ids", []) or []
        return {"robot_ids": robot_ids}
    except Exception:
        return {"robot_ids": []}


@app.get("/api/list_routes")
async def list_routes(robot_id: str = ""):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "list_routes", "fleet_msgs/srv/ListRoutes",
        json.dumps({"robot_id": robot_id}),
    )
    if not ok:
        return {"route_names": []}
    try:
        import yaml
        data = yaml.safe_load(out) if out else {}
        return {"route_names": data.get("route_names", []) or []}
    except Exception:
        return {"route_names": []}


@app.post("/api/enable_collection")
async def enable_collection(robot_id: str = "", topics: str = "scan,odom", output_mode: str = "rosbag2"):
    robot_id = robot_id or ""
    topic_list = [t.strip() for t in topics.split(",") if t.strip()] or ["scan", "odom"]
    ok, out = _run_ros2_service(
        "enable_collection", "fleet_msgs/srv/EnableCollection",
        json.dumps({"robot_id": robot_id, "topics": topic_list, "output_mode": output_mode}),
    )
    return {"success": ok, "message": out}


@app.post("/api/disable_collection")
async def disable_collection(robot_id: str = ""):
    robot_id = robot_id or ""
    ok, out = _run_ros2_service(
        "disable_collection", "fleet_msgs/srv/DisableCollection",
        json.dumps({"robot_id": robot_id}),
    )
    return {"success": ok, "message": out}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
