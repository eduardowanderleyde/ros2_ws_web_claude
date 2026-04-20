"""
Backend FastAPI: bridge para o fleet (ROS 2).
Rode com o workspace sourceado: source install/setup.bash && python main.py
"""
from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import shlex
import struct
import subprocess
import threading
import uuid
import zlib
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

WORKSPACE = os.environ.get("FLEET_WS") or str(Path(__file__).resolve().parent.parent.parent)

# ── Estado global (actualizado pelo thread ROS) ───────────────────────────────
_fleet_status: dict = {"robots": [], "nav2_ready": False}
_robot_poses: dict = {}   # robot_id → {"x", "y", "yaw", "valid"}
_map_meta: dict = {}
_status_lock = threading.Lock()
_ws_clients: list[WebSocket] = []


# ── Helpers ───────────────────────────────────────────────────────────────────

def _ros_env() -> dict:
    return {**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0")}


def _ros_setup_prefix() -> str:
    return (
        f"source /opt/ros/jazzy/setup.bash 2>/dev/null; "
        f"source {WORKSPACE}/install/setup.bash 2>/dev/null; "
    )


# P0-FIX-1: service calls async — não bloqueia o event loop
async def _call_service(srv: str, srv_type: str, request_json: str, timeout: int = 10) -> tuple[bool, str]:
    """Chama ros2 service call de forma assíncrona (não bloqueia outros requests)."""
    cmd = f"{_ros_setup_prefix()}ros2 service call {srv} {srv_type} '{request_json}'"
    try:
        proc = await asyncio.create_subprocess_exec(
            "bash", "-c", cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=_ros_env(),
            cwd=WORKSPACE,
        )
        try:
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)
        except asyncio.TimeoutError:
            proc.kill()
            await proc.communicate()
            return False, "timeout"
        out = (stdout.decode() if stdout else "").strip()
        err = (stderr.decode() if stderr else "").strip()
        combined = out + err
        if proc.returncode != 0:
            return False, combined or "ros2 service call failed"
        return True, combined
    except Exception as e:
        return False, str(e)


def _encode_map_png(data: list, width: int, height: int) -> bytes:
    pixels = []
    for v in data:
        if v < 0:   pixels.append(180)
        elif v == 0: pixels.append(240)
        else:        pixels.append(30)

    def make_row(row_idx: int) -> bytes:
        row = bytearray([0])
        row.extend(pixels[row_idx * width:(row_idx + 1) * width])
        return bytes(row)

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


# ── Thread ROS (subscriber + TF) ─────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    def run_ros():
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.time import Time as RclpyTime
            from fleet_msgs.msg import FleetStatus
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid
            import tf2_ros

            rclpy.init()
            node = Node("fleet_ui_bridge")

            # fleet/status ─────────────────────────────────────────────────────
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

            # pose topics (single-robot: /pose e /amcl_pose → robot_id "")
            def _pose_cb(robot_id: str):
                def _cb(msg):
                    p = msg.pose.pose.position
                    q = msg.pose.pose.orientation
                    yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                    )
                    with _status_lock:
                        _robot_poses[robot_id] = {"x": p.x, "y": p.y, "yaw": yaw, "valid": True}
                return _cb

            # P0-FIX-3: TF lookup por robot_id ────────────────────────────────
            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer, node)

            def tf_timer_cb():
                with _status_lock:
                    robots = [r["robot_id"] for r in _fleet_status.get("robots", [])]
                if not robots:
                    robots = [""]  # fallback single-robot

                for rid in robots:
                    # frames: single-robot → map/base_footprint
                    #         multi-robot  → tb1/map / tb1/base_footprint
                    map_frame  = "map"           if rid == "" else f"{rid}/map"
                    base_frame = "base_footprint" if rid == "" else f"{rid}/base_footprint"
                    try:
                        t = tf_buffer.lookup_transform(map_frame, base_frame, RclpyTime())
                        tr = t.transform.translation
                        q  = t.transform.rotation
                        yaw = math.atan2(
                            2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                        )
                        with _status_lock:
                            _robot_poses[rid] = {"x": tr.x, "y": tr.y, "yaw": yaw, "valid": True}
                    except Exception:
                        # marca como inválido se TF sumir
                        with _status_lock:
                            if rid in _robot_poses:
                                _robot_poses[rid]["valid"] = False

            node.create_timer(0.1, tf_timer_cb)

            # mapa ─────────────────────────────────────────────────────────────
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
                    node.get_logger().warning(f"map_cb: {e}")

            # Nav2 check ───────────────────────────────────────────────────────
            def nav2_check_cb():
                try:
                    r = subprocess.run(
                        ["bash", "-c", f"{_ros_setup_prefix()}ros2 action list 2>/dev/null"],
                        capture_output=True, text=True, timeout=3, env=_ros_env(),
                    )
                    ready = "/navigate_to_pose" in r.stdout
                except Exception:
                    ready = False
                with _status_lock:
                    _fleet_status["nav2_ready"] = ready

            node.create_timer(3.0, nav2_check_cb)

            node.create_subscription(FleetStatus, "fleet/status", fleet_cb, 10)
            node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", _pose_cb(""), 10)
            node.create_subscription(PoseWithCovarianceStamped, "pose", _pose_cb(""), 10)
            node.create_subscription(OccupancyGrid, "map", map_cb, 1)

            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"[fleet_ui] ROS thread erro: {e}")

    threading.Thread(target=run_ros, daemon=True).start()
    yield


app = FastAPI(title="Fleet UI API", lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


# ── Status ────────────────────────────────────────────────────────────────────

def _status_payload() -> dict:
    """Monta payload de status: inclui poses por robot_id + pose de compat (single-robot)."""
    with _status_lock:
        fs = dict(_fleet_status)
        poses = dict(_robot_poses)

    # P0-FIX-2: retorna poses por robot_id
    # "pose" mantém compat com frontend single-robot (robot_id="" ou primeiro da lista)
    robots = fs.get("robots", [])
    default_rid = robots[0]["robot_id"] if robots else ""
    compat_pose = poses.get(default_rid, poses.get("", {"x": 0.0, "y": 0.0, "yaw": 0.0, "valid": False}))

    return {**fs, "poses": poses, "pose": compat_pose}


@app.get("/api/status")
async def get_status():
    return _status_payload()


@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.append(websocket)
    try:
        await websocket.send_text(json.dumps(_status_payload()))
        while True:
            await asyncio.sleep(0.25)
            await websocket.send_text(json.dumps(_status_payload()))
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in _ws_clients:
            _ws_clients.remove(websocket)


# ── Jobs (experiment_repeatability.py subprocess) ────────────────────────────

_jobs: dict = {}


def _build_cmd(cfg: dict) -> list[str]:
    cmd   = cfg.get("command", "record")
    robot = cfg.get("robot", "")
    single = not robot or robot == "default"
    route  = cfg.get("route", "percurso1")
    collect = cfg.get("collect", True)
    topics  = cfg.get("topics", ["scan", "odom", "imu"])
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
            args += ["--points", ";".join(f"{p[0]},{p[1]},{p[2] if len(p) > 2 else 0}" for p in pts)]
    elif cmd == "replay":
        rts = cfg.get("return_to_start")
        if rts:
            args += ["--return-to-start", f"{rts[0]},{rts[1]},{rts[2]}"]
    return args


@app.post("/api/run_config")
async def run_config(cfg: dict):
    job_id = str(uuid.uuid4())[:8]
    export_path = str(Path(WORKSPACE) / f"_job_{job_id}.json")
    try:
        cmd = _build_cmd(cfg) + ["--export", export_path]
    except Exception as e:
        return JSONResponse({"success": False, "message": str(e)}, status_code=400)

    _jobs[job_id] = {"running": True, "lines": [], "result": None, "error": None, "exit_code": None}

    def _run():
        env = {**_ros_env(), "PYTHONUNBUFFERED": "1"}
        shell_cmd = _ros_setup_prefix() + " ".join(shlex.quote(c) for c in cmd)
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
            ep = Path(export_path)
            if ep.exists():
                _jobs[job_id]["result"] = json.loads(ep.read_text())
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
    return {k: job[k] for k in ("running", "lines", "result", "error", "exit_code")}


# ── Fleet services (agora todos async — sem bloqueio do event loop) ───────────

@app.post("/api/go_to_point")
async def go_to_point(robot_id: str = "", x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
    ok, out = await _call_service(
        "go_to_point", "fleet_msgs/srv/GoToPoint",
        json.dumps({"robot_id": robot_id or "", "x": x, "y": y, "yaw": yaw}),
    )
    return {"success": ok, "message": out}


@app.post("/api/start_record")
async def start_record(robot_id: str = "", route_name: str = "r1"):
    ok, out = await _call_service(
        "start_record", "fleet_msgs/srv/StartRecord",
        json.dumps({"robot_id": robot_id or "", "route_name": route_name}),
    )
    return {"success": ok, "message": out}


@app.post("/api/stop_record")
async def stop_record(robot_id: str = ""):
    ok, out = await _call_service(
        "stop_record", "fleet_msgs/srv/StopRecord",
        json.dumps({"robot_id": robot_id or ""}),
    )
    return {"success": ok, "message": out}


@app.post("/api/play_route")
async def play_route(robot_id: str = "", route_name: str = "r1"):
    ok, out = await _call_service(
        "play_route", "fleet_msgs/srv/PlayRoute",
        json.dumps({"robot_id": robot_id or "", "route_name": route_name}),
    )
    return {"success": ok, "message": out}


@app.post("/api/cancel")
async def cancel(robot_id: str = ""):
    ok, out = await _call_service(
        "cancel", "fleet_msgs/srv/Cancel",
        json.dumps({"robot_id": robot_id or ""}),
    )
    return {"success": ok, "message": out}


@app.post("/api/enable_collection")
async def enable_collection(robot_id: str = "", topics: str = "scan,odom", output_mode: str = "rosbag2"):
    topic_list = [t.strip() for t in topics.split(",") if t.strip()] or ["scan", "odom"]
    ok, out = await _call_service(
        "enable_collection", "fleet_msgs/srv/EnableCollection",
        json.dumps({"robot_id": robot_id or "", "topics": topic_list, "output_mode": output_mode}),
    )
    return {"success": ok, "message": out}


@app.post("/api/disable_collection")
async def disable_collection(robot_id: str = ""):
    ok, out = await _call_service(
        "disable_collection", "fleet_msgs/srv/DisableCollection",
        json.dumps({"robot_id": robot_id or ""}),
    )
    return {"success": ok, "message": out}


@app.get("/api/list_robots")
async def list_robots():
    ok, out = await _call_service("list_robots", "fleet_msgs/srv/ListRobots", "{}")
    if not ok:
        return {"robot_ids": []}
    try:
        import yaml
        data = yaml.safe_load(out) if out else {}
        return {"robot_ids": data.get("robot_ids", []) or []}
    except Exception:
        return {"robot_ids": []}


@app.get("/api/list_routes")
async def list_routes(robot_id: str = ""):
    ok, out = await _call_service(
        "list_routes", "fleet_msgs/srv/ListRoutes",
        json.dumps({"robot_id": robot_id or ""}),
    )
    if not ok:
        return {"route_names": []}
    try:
        import yaml
        data = yaml.safe_load(out) if out else {}
        return {"route_names": data.get("route_names", []) or []}
    except Exception:
        return {"route_names": []}


# ── Utilitários ───────────────────────────────────────────────────────────────

@app.post("/api/save_route_waypoints")
async def save_route_waypoints(body: dict):
    import yaml
    robot_id   = body.get("robot_id", "") or ""
    route_name = (body.get("route_name") or "").strip()
    waypoints  = body.get("waypoints", [])
    if not route_name:
        return JSONResponse({"success": False, "message": "route_name vazio"}, status_code=400)
    if not waypoints:
        return JSONResponse({"success": False, "message": "waypoints vazio"}, status_code=400)
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
            return JSONResponse({"available": False})
        return {"available": True, **_map_meta}


# ── Descoberta de robôs ───────────────────────────────────────────────────────

@app.get("/api/discover_robots")
async def discover_robots(subnet: str = ""):
    import socket
    import concurrent.futures

    if not subnet:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            subnet = ".".join(local_ip.split(".")[:3])
            print(f"[discover] IP local: {local_ip}  subnet: {subnet}")
        except Exception as e:
            print(f"[discover] auto-detecção falhou: {e}")
            return {"found": [], "subnet_scanned": "", "error": "Não foi possível detectar subnet. Informe manualmente (ex: 192.168.1)"}

    subnet = subnet.strip()
    parts = subnet.split(".")
    if len(parts) != 3 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
        return {"found": [], "subnet_scanned": subnet, "error": f"Subnet inválida: {subnet}. Use X.X.X (ex: 192.168.1)"}

    targets = [f"{subnet}.{i}" for i in range(1, 255)]

    def _check(ip: str):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.3)
            if sock.connect_ex((ip, 22)) != 0:
                sock.close()
                return None
            sock.close()
            try:
                hostname = socket.gethostbyaddr(ip)[0]
            except Exception:
                hostname = ip
            is_robot = any(kw in hostname.lower() for kw in ("tb", "turtle", "robot", "pi", "nano", "jetson", "ros"))
            return {"ip": ip, "hostname": hostname, "ssh": True, "likely_robot": is_robot}
        except Exception:
            return None

    loop = asyncio.get_event_loop()
    with concurrent.futures.ThreadPoolExecutor(max_workers=60) as ex:
        results = await loop.run_in_executor(None, lambda: list(ex.map(_check, targets)))

    found = sorted([r for r in results if r], key=lambda x: (not x["likely_robot"], x["ip"]))
    return {"found": found, "subnet_scanned": subnet}


@app.post("/api/test_ssh")
async def test_ssh(body: dict):
    host = (body.get("host") or "").strip()
    user = (body.get("user") or "ubuntu").strip()
    port = int(body.get("port") or 22)
    if not host:
        return JSONResponse({"success": False, "message": "host vazio"}, status_code=400)
    cmd = (
        f"ssh -o ConnectTimeout=4 -o StrictHostKeyChecking=no -o BatchMode=yes "
        f"-p {port} {user}@{host} "
        f"'which ros2 && ros2 --version 2>/dev/null || echo NO_ROS2'"
    )
    try:
        proc = await asyncio.create_subprocess_exec(
            "bash", "-c", cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=8)
        except asyncio.TimeoutError:
            proc.kill()
            return {"success": False, "message": "Timeout SSH"}
        out = (stdout.decode() if stdout else "").strip()
        err = (stderr.decode() if stderr else "").strip()
        if proc.returncode != 0:
            return {"success": False, "message": err or f"SSH falhou (exit {proc.returncode})"}
        has_ros = "NO_ROS2" not in out and bool(out)
        return {
            "success": True,
            "has_ros2": has_ros,
            "ros2_version": out if has_ros else None,
            "message": f"ROS 2 encontrado: {out}" if has_ros else "SSH OK mas ROS 2 não encontrado",
        }
    except Exception as e:
        return {"success": False, "message": str(e)}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
