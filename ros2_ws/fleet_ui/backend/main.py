"""
Backend FastAPI: bridge para o fleet (ROS 2).
Rode com o workspace sourceado: source install/setup.bash && python main.py
"""
from __future__ import annotations

import base64
import json
import math
import os
import struct
import subprocess
import threading
import zlib
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
_robot_pose: dict = {"x": 0.0, "y": 0.0, "yaw": 0.0, "valid": False}
_map_meta: dict = {}  # resolution, origin_x, origin_y, width, height, png_b64
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


def _encode_map_png(data: list, width: int, height: int) -> bytes:
    """Codifica OccupancyGrid como PNG grayscale (Y flipado para canvas)."""
    pixels = []
    for v in data:
        if v < 0:
            pixels.append(180)   # desconhecido: cinza
        elif v == 0:
            pixels.append(240)   # livre: branco
        else:
            pixels.append(30)    # ocupado: quase preto

    def make_row(row_idx: int) -> bytes:
        row = bytearray([0])  # filter type None
        row.extend(pixels[row_idx * width:(row_idx + 1) * width])
        return bytes(row)

    # Flipa Y: row 0 do PNG = maior y do mundo
    raw = b''.join(make_row(height - 1 - y) for y in range(height))
    compressed = zlib.compress(raw, 6)

    def png_chunk(tag: bytes, payload: bytes) -> bytes:
        body = tag + payload
        return struct.pack('>I', len(payload)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    png = b'\x89PNG\r\n\x1a\n'
    png += png_chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
    png += png_chunk(b'IDAT', compressed)
    png += png_chunk(b'IEND', b'')
    return png


@asynccontextmanager
async def lifespan(app: FastAPI):
    def run_ros():
        try:
            import rclpy
            from rclpy.node import Node
            from fleet_msgs.msg import FleetStatus
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid

            rclpy.init()
            node = Node("fleet_ui_bridge", parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)
            ])

            def fleet_cb(msg):
                with _status_lock:
                    _fleet_status["robots"] = [
                        {
                            "robot_id": r.robot_id,
                            "role": r.role,
                            "nav_state": r.nav_state,
                            "current_route": r.current_route,
                            "collection_on": r.collection_on,
                            "collection_file": r.collection_file,
                            "last_error": r.last_error,
                            "bytes_written": r.bytes_written,
                        }
                        for r in msg.robots
                    ]

            def amcl_cb(msg):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                with _status_lock:
                    _robot_pose.update({"x": p.x, "y": p.y, "yaw": yaw, "valid": True})

            # TF lookup: map → base_footprint (usado com SLAM Toolbox)
            import tf2_ros
            from rclpy.time import Time as RclpyTime
            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer, node)

            def tf_timer_cb():
                try:
                    t = tf_buffer.lookup_transform("map", "base_footprint", RclpyTime())
                    tr = t.transform.translation
                    q = t.transform.rotation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    with _status_lock:
                        _robot_pose.update({"x": tr.x, "y": tr.y, "yaw": yaw, "valid": True})
                except Exception:
                    pass

            node.create_timer(0.1, tf_timer_cb)

            def map_cb(msg):
                info = msg.info
                if info.width == 0 or info.height == 0:
                    return
                try:
                    png = _encode_map_png(list(msg.data), info.width, info.height)
                    b64 = base64.b64encode(png).decode()
                    with _status_lock:
                        _map_meta.update({
                            "resolution": info.resolution,
                            "origin_x": info.origin.position.x,
                            "origin_y": info.origin.position.y,
                            "width": info.width,
                            "height": info.height,
                            "png_b64": b64,
                        })
                except Exception as e:
                    node.get_logger().warning(f"map_cb error: {e}")

            # Verifica Nav2 periodicamente via ros2 action list (mais confiável que ActionClient)
            def nav2_check_cb():
                try:
                    env = {**os.environ}
                    r = subprocess.run(
                        ["bash", "-c", "source /opt/ros/jazzy/setup.bash 2>/dev/null; ros2 action list 2>/dev/null"],
                        capture_output=True, text=True, timeout=3, env=env,
                    )
                    ready = "/navigate_to_pose" in r.stdout
                except Exception:
                    ready = False
                with _status_lock:
                    _fleet_status["nav2_ready"] = ready

            node.create_timer(3.0, nav2_check_cb)

            node.create_subscription(FleetStatus, "fleet/status", fleet_cb, 10)
            node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", amcl_cb, 10)
            node.create_subscription(PoseWithCovarianceStamped, "pose", amcl_cb, 10)
            node.create_subscription(OccupancyGrid, "map", map_cb, 1)
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"[fleet_ui] ROS subscriber not started (source workspace?): {e}")

    t = threading.Thread(target=run_ros, daemon=True)
    t.start()
    yield


app = FastAPI(title="Fleet UI API", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/status")
async def get_status():
    with _status_lock:
        return {**_fleet_status, "pose": _robot_pose}


@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.append(websocket)
    try:
        with _status_lock:
            payload = {**_fleet_status, "pose": _robot_pose}
        await websocket.send_text(json.dumps(payload))
        while True:
            await asyncio.sleep(0.25)
            with _status_lock:
                payload = {**_fleet_status, "pose": _robot_pose}
            await websocket.send_text(json.dumps(payload))
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in _ws_clients:
            _ws_clients.remove(websocket)


import uuid
import shlex

_jobs: dict = {}   # job_id → {running, lines, result, error}


def _build_cmd(cfg: dict) -> list[str]:
    cmd = cfg.get("command", "record")
    robot = cfg.get("robot", "")
    # "default" é o robot único em single_robot_sim → usa --single-robot (robot_id="")
    single = not robot or robot == "default"
    route = cfg.get("route", "percurso1")
    collect = cfg.get("collect", True)
    topics = cfg.get("topics", ["scan", "odom", "imu", "pose"])
    ip = cfg.get("initial_pose")
    args = [
        "python3", str(Path(WORKSPACE) / "scripts" / "experiment_repeatability.py"),
        cmd, "--route", route,
    ]
    if single:
        args += ["--single-robot"]
    else:
        args += ["--robot", robot]
    if not collect:
        args += ["--skip-collection"]
    else:
        args += ["--topics"] + topics
    if ip:
        args += ["--initial-pose", f"{ip[0]},{ip[1]},{ip[2]}"]
    if cmd == "record":
        pts = cfg.get("points", [])
        if pts:
            pts_str = ";".join(f"{p[0]},{p[1]},{p[2] if len(p)>2 else 0}" for p in pts)
            args += [f"--points={pts_str}"]
    elif cmd == "replay":
        rts = cfg.get("return_to_start")
        if rts:
            args += ["--return-to-start", f"{rts[0]},{rts[1]},{rts[2]}"]
    # export result to temp file
    return args


@app.post("/api/run_config")
async def run_config(cfg: dict):
    job_id = str(uuid.uuid4())[:8]
    export_path = str(Path(WORKSPACE) / f"_job_{job_id}.json")
    try:
        cmd = _build_cmd(cfg)
        cmd += ["--export", export_path]
    except Exception as e:
        return JSONResponse({"success": False, "message": str(e)}, status_code=400)

    _jobs[job_id] = {"running": True, "lines": [], "result": None, "error": None, "exit_code": None}

    def _run():
        env = {**_ros_env(), "PYTHONUNBUFFERED": "1"}
        ros_setup = f"source /opt/ros/jazzy/setup.bash 2>/dev/null; source {WORKSPACE}/install/setup.bash 2>/dev/null; "
        shell_cmd = ros_setup + " ".join(shlex.quote(c) for c in cmd)
        try:
            proc = subprocess.Popen(
                ["bash", "-c", shell_cmd],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, cwd=WORKSPACE, env=env,
            )
            _NOISE = ("TF_OLD_DATA", "RTPS_TRANSPORT_SHM", "Possible reasons", "ros.org/tf")
            for line in proc.stdout:
                l = line.rstrip()
                if any(n in l for n in _NOISE):
                    continue
                _jobs[job_id]["lines"].append(l)
            proc.wait()
            _jobs[job_id]["exit_code"] = proc.returncode
            # load export json
            ep = Path(export_path)
            if ep.exists():
                import json as _json
                _jobs[job_id]["result"] = _json.loads(ep.read_text())
                ep.unlink(missing_ok=True)
        except Exception as ex:
            _jobs[job_id]["error"] = str(ex)
        finally:
            _jobs[job_id]["running"] = False

    threading.Thread(target=_run, daemon=True).start()
    return {"job_id": job_id}


@app.get("/api/job/{job_id}")
async def get_job(job_id: str):
    job = _jobs.get(job_id)
    if not job:
        return JSONResponse({"error": "job not found"}, status_code=404)
    return {
        "running": job["running"],
        "lines": job["lines"],
        "result": job["result"],
        "error": job["error"],
        "exit_code": job["exit_code"],
    }


@app.post("/api/save_route_waypoints")
async def save_route_waypoints(body: dict):
    """Salva lista de waypoints diretamente como yaml (sem precisar do record flow)."""
    import yaml
    robot_id = body.get("robot_id", "") or ""
    route_name = (body.get("route_name") or "").strip()
    waypoints = body.get("waypoints", [])
    if not route_name:
        return JSONResponse(content={"success": False, "message": "route_name vazio"}, status_code=400)
    if not waypoints:
        return JSONResponse(content={"success": False, "message": "waypoints vazio"}, status_code=400)
    folder = "default" if not robot_id else robot_id
    routes_dir = Path(WORKSPACE) / "routes" / folder
    routes_dir.mkdir(parents=True, exist_ok=True)
    path = routes_dir / f"{route_name}.yaml"
    data = {
        "route_name": route_name,
        "frame": "map",
        "poses": [{"x": float(w.get("x", 0)), "y": float(w.get("y", 0)), "yaw": float(w.get("yaw", 0))} for w in waypoints],
    }
    path.write_text(yaml.dump(data, default_flow_style=False))
    return {"success": True, "message": f"Salvo {len(waypoints)} waypoints em {path}"}


@app.get("/api/map")
async def get_map():
    with _status_lock:
        if not _map_meta:
            return JSONResponse(content={"available": False})
        return {"available": True, **_map_meta}


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


@app.get("/api/discover_robots")
async def discover_robots(subnet: str = ""):
    """
    Varre a subnet por hosts com porta 22 aberta e testa se têm ROS 2.
    subnet ex: '192.168.1' (varre .1–.254).
    Se omitido, detecta automaticamente a subnet local.
    """
    import ipaddress
    import socket
    import concurrent.futures

    # Detecta subnet local se não informada
    if not subnet:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            subnet = ".".join(local_ip.split(".")[:3])
            print(f"[discover] IP local: {local_ip}  subnet: {subnet}")
        except Exception as e:
            print(f"[discover] falhou auto-detecção: {e}")
            return {"found": [], "subnet_scanned": "", "error": "Não foi possível detectar subnet local. Informe manualmente (ex: 192.168.1)"}

    subnet = subnet.strip()

    # Valida formato
    try:
        parts = subnet.split(".")
        if len(parts) != 3 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
            return {"found": [], "subnet_scanned": subnet, "error": f"Subnet inválida: {subnet}. Use formato X.X.X (ex: 192.168.1)"}
    except Exception:
        return {"found": [], "subnet_scanned": subnet, "error": f"Subnet inválida: {subnet}"}

    targets = [f"{subnet}.{i}" for i in range(1, 255)]

    def _check(ip: str) -> dict | None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.3)
            result = sock.connect_ex((ip, 22))
            sock.close()
            if result != 0:
                return None
            # Tenta resolver hostname
            try:
                hostname = socket.gethostbyaddr(ip)[0]
            except Exception:
                hostname = ip
            # Heurística: robotics hostnames
            is_robot = any(kw in hostname.lower() for kw in ("tb", "turtle", "robot", "pi", "nano", "jetson", "ros"))
            return {"ip": ip, "hostname": hostname, "ssh": True, "likely_robot": is_robot}
        except Exception:
            return None

    found = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=60) as ex:
        for res in ex.map(_check, targets):
            if res:
                found.append(res)

    found.sort(key=lambda x: (not x["likely_robot"], x["ip"]))
    return {"found": found, "subnet_scanned": subnet}


@app.post("/api/test_ssh")
async def test_ssh(body: dict):
    """Testa SSH num host: verifica conexão e presença do ROS 2."""
    host = (body.get("host") or "").strip()
    user = (body.get("user") or "ubuntu").strip()
    port = int(body.get("port") or 22)
    if not host:
        return JSONResponse({"success": False, "message": "host vazio"}, status_code=400)
    try:
        cmd = (
            f"ssh -o ConnectTimeout=4 -o StrictHostKeyChecking=no -o BatchMode=yes "
            f"-p {port} {user}@{host} "
            f"'which ros2 && ros2 --version 2>/dev/null || echo NO_ROS2'"
        )
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=8)
        out = (r.stdout or "").strip()
        err = (r.stderr or "").strip()
        if r.returncode != 0:
            return {"success": False, "message": err or f"SSH falhou (exit {r.returncode})"}
        has_ros = "NO_ROS2" not in out and out
        return {
            "success": True,
            "has_ros2": has_ros,
            "ros2_version": out if has_ros else None,
            "message": f"ROS 2 encontrado: {out}" if has_ros else "SSH OK mas ROS 2 não encontrado",
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "message": "Timeout ao conectar via SSH"}
    except Exception as e:
        return {"success": False, "message": str(e)}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
