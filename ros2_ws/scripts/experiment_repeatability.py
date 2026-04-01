#!/usr/bin/env python3
"""
Experimento reproduzível (sem UI): gravar percurso por sequência de go_to_point com coleta,
depois reproduzir a mesma rota com nova coleta — para comparar bags.

Fase A — gravar (coleta + record + waypoints + salvar YAML):
  source install/setup.bash
  python3 scripts/experiment_repeatability.py record \\
    --robot tb1 --route percurso1_tb1 \\
    --points "0.5,0,0;1.0,0,0;1.5,0.5,0;2.0,0.5,0"

Modo **single-robot** (sim sem namespace, `robot_id` vazio → rotas em `routes/default/`):
  python3 scripts/experiment_repeatability.py record --single-robot --route percurso1_tb1 \\
    --points "0.5,0,0;1.0,0,0;1.5,0.5,0;2.0,0.5,0"
  # ou: --robot ""

Fase B — reproduzir (coleta + play_route):
  python3 scripts/experiment_repeatability.py replay --robot tb1 --route percurso1_tb1

Requisitos: sim + Nav2 + fleet (orchestrator + collector), TF map ok, papel MUUT para movimento.

Pontos: triples x,y,yaw_rad separados por ';' (yaw pode ser 0).

Opcional: --export run_summary.json (metadados; inclui rosbag_path quando o disable_collection devolve o caminho do bag).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry

from fleet_msgs.msg import FleetStatus
from fleet_msgs.srv import (
    DisableCollection,
    EnableCollection,
    GoToPoint,
    PlayRoute,
    StartRecord,
    StopRecord,
)


def _norm_status_id(robot_id: str) -> str:
    """Alinha com /fleet/status (robot_id vazio -> 'default')."""
    return "default" if not robot_id.strip() else robot_id.strip()


def _expected_route_yaml_path(robot_id: str, route: str) -> str:
    key = "default" if not robot_id.strip() else robot_id.strip()
    return f"routes/{key}/{route}.yaml"


def _parse_points(s: str) -> List[Tuple[float, float, float]]:
    out: List[Tuple[float, float, float]] = []
    for part in s.split(";"):
        part = part.strip()
        if not part:
            continue
        bits = [float(x.strip()) for x in part.split(",")]
        if len(bits) != 3:
            raise ValueError(f"Ponto inválido (esperado x,y,yaw): {part!r}")
        out.append((bits[0], bits[1], bits[2]))
    if not out:
        raise ValueError("Nenhum ponto em --points")
    return out


class FleetExperimentNode(Node):
    def __init__(self, *, use_sim_time: bool = True) -> None:
        super().__init__(
            "experiment_repeatability",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time),
            ],
        )
        self.last_status: Optional[FleetStatus] = None
        self.last_odom: Optional[Odometry] = None
        self.create_subscription(FleetStatus, "fleet/status", self._cb, 10)
        self.create_subscription(Odometry, "odom", self._cb_odom, 30)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _cb(self, msg: FleetStatus) -> None:
        self.last_status = msg

    def _cb_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def call_srv(self, srv_type, name: str, request, timeout_sec: float = 15.0):
        cli = self.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=min(10.0, timeout_sec)):
            return None, False, f"service {name} indisponível"
        fut = cli.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if not fut.done():
            return None, False, f"timeout {name}"
        try:
            return fut.result(), True, ""
        except Exception as e:
            return None, False, str(e)

    def wait_nav_state(self, robot_id: str, expected: str, timeout_sec: float) -> bool:
        want = _norm_status_id(robot_id)
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.15)
            if self.last_status is None:
                continue
            for rs in self.last_status.robots:
                rid = rs.robot_id if rs.robot_id else "default"
                if rid == want and rs.nav_state == expected:
                    return True
            time.sleep(0.05)
        return False

    def wait_tf_available(
        self, global_frame: str, base_frame: str, timeout_sec: float
    ) -> bool:
        """Espera até existir TF global_frame -> base_frame (transform mais recente)."""
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            try:
                self.tf_buffer.lookup_transform(
                    global_frame,
                    base_frame,
                    Time(),  # latest transform
                    timeout=Duration(seconds=0.2),
                )
                return True
            except TransformException:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_motion_detected(self, min_dist_m: float, timeout_sec: float) -> Tuple[bool, float]:
        """
        Aguarda deslocamento no /odom e retorna (ok, path_length_m_estimate).
        path_length_m_estimate é a distância acumulada entre amostras de odom.
        """
        t0 = time.time()
        prev_xy: Optional[Tuple[float, float]] = None
        path_acc = 0.0

        # Tenta capturar uma amostra inicial rapidamente.
        while self.last_odom is None and (time.time() - t0) < min(2.0, timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            prev_xy = (float(p.x), float(p.y))

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_odom is None:
                continue
            p = self.last_odom.pose.pose.position
            cur_xy = (float(p.x), float(p.y))
            if prev_xy is None:
                prev_xy = cur_xy
                continue
            step = math.hypot(cur_xy[0] - prev_xy[0], cur_xy[1] - prev_xy[1])
            if step > 0.0:
                path_acc += step
                prev_xy = cur_xy
            if path_acc >= min_dist_m:
                return True, path_acc
        return False, path_acc


def _print(ok: bool, msg: str, detail: str = "") -> None:
    tag = "OK" if ok else "FALHOU"
    extra = f" — {detail}" if detail else ""
    print(f"[{tag}] {msg}{extra}")


def _rosbag_path_from_disable_message(msg: Optional[str]) -> Optional[str]:
    """Extrai o caminho do bag da resposta do disable_collection (texto '... Bag: <path>')."""
    if not msg or "Bag:" not in msg:
        return None
    tail = msg.split("Bag:", 1)[1].strip()
    return tail if tail else None


def _write_export(path: str, payload: Dict[str, Any]) -> None:
    payload = {
        **payload,
        "finished_at": datetime.now(timezone.utc).isoformat(),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"\n[OK] Resumo exportado: {path}")


def cmd_record(args: argparse.Namespace) -> int:
    try:
        points = _parse_points(args.points)
    except ValueError as e:
        print(f"Erro: {e}", file=sys.stderr)
        return 2

    rclpy.init()
    node = FleetExperimentNode(use_sim_time=not args.no_use_sim_time)
    fails = 0
    rid = args.robot
    disable_msg: Optional[str] = None
    run_error_code = ""
    path_length_m_estimate = 0.0

    try:
        print("\n=== Fase A: gravar percurso (coleta + record + waypoints) ===")
        print(f"robot={rid!r} route={args.route} pontos={len(points)}")
        print(f"topics={args.topics} skip_collection={args.skip_collection} export={args.export!r}")

        if not args.skip_collection:
            req = EnableCollection.Request()
            req.robot_id = rid
            req.topics = args.topics
            req.output_mode = "rosbag2"
            print("[TRACE] Chamando service enable_collection ...")
            resp, ok, err = node.call_srv(EnableCollection, "enable_collection", req)
            good = ok and resp is not None and resp.success
            _print(good, "enable_collection", (resp.message if resp else err))
            fails += int(not good)
            if not good:
                code = 1
                if args.export:
                    _write_export(
                        args.export,
                        {
                            "command": "record",
                            "robot_id": rid,
                            "route": args.route,
                            "expected_route_yaml": _expected_route_yaml_path(rid, args.route),
                            "points": [list(p) for p in points],
                            "topics": args.topics,
                            "skip_collection": args.skip_collection,
                            "success": False,
                            "failures_reported": fails,
                            "disable_collection_message": disable_msg,
                            "rosbag_path": _rosbag_path_from_disable_message(disable_msg),
                        },
                    )
                return code
            time.sleep(1.0)

        req_sr = StartRecord.Request()
        req_sr.robot_id = rid
        req_sr.route_name = args.route
        print("[TRACE] Chamando service start_record ...")
        resp, ok, err = node.call_srv(StartRecord, "start_record", req_sr)
        good = ok and resp is not None and resp.success
        _print(good, "start_record", (resp.message if resp else err))
        fails += int(not good)
        if not good:
            code = 1
            if args.export:
                _write_export(
                    args.export,
                    {
                        "command": "record",
                        "robot_id": rid,
                        "route": args.route,
                        "expected_route_yaml": _expected_route_yaml_path(rid, args.route),
                        "points": [list(p) for p in points],
                        "topics": args.topics,
                        "skip_collection": args.skip_collection,
                        "success": False,
                        "failures_reported": fails,
                        "disable_collection_message": disable_msg,
                        "rosbag_path": _rosbag_path_from_disable_message(disable_msg),
                    },
                )
            return code

        # Evita falhas transitórias de TF (TF_ERROR 202) antes do 1º goal.
        print("[TRACE] Esperando TF map->base_link ...")
        tf_ok = node.wait_tf_available("map", "base_link", timeout_sec=20.0)
        _print(tf_ok, "TF map->base_link disponível")

        try:
            for i, (x, y, yaw) in enumerate(points, start=1):
                print(f"[TRACE] go_to_point wp{i}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
                req_g = GoToPoint.Request()
                req_g.robot_id = rid
                req_g.x = x
                req_g.y = y
                req_g.yaw = yaw
                resp_g, ok_g, err_g = node.call_srv(GoToPoint, "go_to_point", req_g, timeout_sec=20.0)
                good = ok_g and resp_g is not None and resp_g.success
                _print(good, f"go_to_point wp{i} ({x:.2f}, {y:.2f}, yaw={yaw:.2f})", (resp_g.message if resp_g else err_g))
                fails += int(not good)
                if not good:
                    break
                nav_started = node.wait_nav_state(rid, "navigating", timeout_sec=12.0)
                _print(nav_started, f"wp{i} estado navigating")
                fails += int(not nav_started)
                if args.require_motion and nav_started:
                    moved, dist_m = node.wait_motion_detected(
                        min_dist_m=args.motion_min_dist,
                        timeout_sec=args.motion_timeout,
                    )
                    path_length_m_estimate = max(path_length_m_estimate, dist_m)
                    detail = (
                        f"dist_est={dist_m:.3f}m (mín={args.motion_min_dist:.3f}m em {args.motion_timeout:.1f}s)"
                    )
                    _print(moved, f"wp{i} movimento detectado", detail)
                    if not moved:
                        fails += 1
                        run_error_code = "NO_MOTION_DETECTED"
                        break
                # Durante record, o fleet mostra `recording` enquanto a coleta está ativa.
                # Depois que o Nav2 termina, `is_navigating` vira False, mas `is_recording`
                # permanece True até o stop_record. Por isso "idle" pode nunca acontecer aqui.
                nav_done = node.wait_nav_state(rid, "recording", timeout_sec=args.wait_goal)
                _print(nav_done, f"wp{i} voltou a recording (nav finalizado)")
                fails += int(not nav_done)
        finally:
            req_st = StopRecord.Request()
            req_st.robot_id = rid
            print("[TRACE] Chamando service stop_record ...")
            resp, ok, err = node.call_srv(StopRecord, "stop_record", req_st)
            good = ok and resp is not None and resp.success and "Route length: 0" not in (resp.message or "")
            _print(good, "stop_record", (resp.message if resp else err))
            fails += int(not good)

        if not args.skip_collection:
            req_d = DisableCollection.Request()
            req_d.robot_id = rid
            resp, ok, err = node.call_srv(DisableCollection, "disable_collection", req_d)
            good = ok and resp is not None and resp.success
            _print(good, "disable_collection", (resp.message if resp else err))
            fails += int(not good)
            if resp is not None:
                disable_msg = resp.message

        print(f"\nResumo: {'sem falhas' if fails == 0 else f'{fails} falha(s)'}")
        code = 0 if fails == 0 else 1
        if args.export:
            _write_export(
                args.export,
                {
                    "command": "record",
                    "robot_id": rid,
                    "route": args.route,
                    "expected_route_yaml": _expected_route_yaml_path(rid, args.route),
                    "points": [list(p) for p in points],
                    "topics": args.topics,
                    "skip_collection": args.skip_collection,
                    "success": code == 0,
                    "failures_reported": fails,
                    "error_code": run_error_code,
                    "path_length_m_estimate": path_length_m_estimate,
                    "disable_collection_message": disable_msg,
                    "rosbag_path": _rosbag_path_from_disable_message(disable_msg),
                },
            )
        return code
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_replay(args: argparse.Namespace) -> int:
    rclpy.init()
    node = FleetExperimentNode(use_sim_time=not args.no_use_sim_time)
    fails = 0
    rid = args.robot
    disable_msg: Optional[str] = None
    run_error_code = ""
    path_length_m_estimate = 0.0

    try:
        print("\n=== Fase B: reproduzir percurso (coleta + play_route) ===")
        print(f"robot={rid!r} route={args.route}")
        print(f"topics={args.topics} skip_collection={args.skip_collection} export={args.export!r}")

        if not args.skip_collection:
            req = EnableCollection.Request()
            req.robot_id = rid
            req.topics = args.topics
            req.output_mode = "rosbag2"
            print("[TRACE] Chamando service enable_collection ...")
            resp, ok, err = node.call_srv(EnableCollection, "enable_collection", req)
            good = ok and resp is not None and resp.success
            _print(good, "enable_collection", (resp.message if resp else err))
            fails += int(not good)
            if not good:
                code = 1
                if args.export:
                    _write_export(
                        args.export,
                        {
                            "command": "replay",
                            "robot_id": rid,
                            "route": args.route,
                            "expected_route_yaml": _expected_route_yaml_path(rid, args.route),
                            "topics": args.topics,
                            "skip_collection": args.skip_collection,
                            "success": False,
                            "failures_reported": fails,
                            "disable_collection_message": disable_msg,
                            "rosbag_path": _rosbag_path_from_disable_message(disable_msg),
                        },
                    )
                return code
            time.sleep(1.0)

        print("[TRACE] Warmup após enable_collection (sleep 5s) ...")
        time.sleep(5.0)

        req_p = PlayRoute.Request()
        req_p.robot_id = rid
        req_p.route_name = args.route
        print("[TRACE] Chamando service play_route ...")
        resp, ok, err = node.call_srv(PlayRoute, "play_route", req_p, timeout_sec=20.0)
        good = ok and resp is not None and resp.success
        _print(good, "play_route", (resp.message if resp else err))
        fails += int(not good)
        if good:
            nav_started = node.wait_nav_state(rid, "navigating", timeout_sec=15.0)
            _print(nav_started, "navegação iniciou (navigating)")
            fails += int(not nav_started)
            if args.require_motion and nav_started:
                moved, dist_m = node.wait_motion_detected(
                    min_dist_m=args.motion_min_dist,
                    timeout_sec=args.motion_timeout,
                )
                path_length_m_estimate = max(path_length_m_estimate, dist_m)
                detail = (
                    f"dist_est={dist_m:.3f}m (mín={args.motion_min_dist:.3f}m em {args.motion_timeout:.1f}s)"
                )
                _print(moved, "movimento detectado", detail)
                if not moved:
                    fails += 1
                    run_error_code = "NO_MOTION_DETECTED"
            nav_done = node.wait_nav_state(rid, "idle", timeout_sec=args.wait_goal)
            _print(nav_done, "navegação terminou (idle)")
            fails += int(not nav_done)

        if not args.skip_collection:
            req_d = DisableCollection.Request()
            req_d.robot_id = rid
            print("[TRACE] Chamando service disable_collection ...")
            resp, ok, err = node.call_srv(DisableCollection, "disable_collection", req_d)
            good = ok and resp is not None and resp.success
            _print(good, "disable_collection", (resp.message if resp else err))
            fails += int(not good)
            if resp is not None:
                disable_msg = resp.message

        print(f"\nResumo: {'sem falhas' if fails == 0 else f'{fails} falha(s)'}")
        code = 0 if fails == 0 else 1
        if args.export:
            _write_export(
                args.export,
                {
                    "command": "replay",
                    "robot_id": rid,
                    "route": args.route,
                    "expected_route_yaml": _expected_route_yaml_path(rid, args.route),
                    "topics": args.topics,
                    "skip_collection": args.skip_collection,
                    "success": code == 0,
                    "failures_reported": fails,
                    "error_code": run_error_code,
                    "path_length_m_estimate": path_length_m_estimate,
                    "disable_collection_message": disable_msg,
                    "rosbag_path": _rosbag_path_from_disable_message(disable_msg),
                },
            )
        return code
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _apply_single_robot(args: argparse.Namespace) -> None:
    if getattr(args, "single_robot", False):
        args.robot = ""


def main() -> int:
    p = argparse.ArgumentParser(
        description="Experimento reproduzível: record (waypoints) e replay (play_route), com coleta."
    )
    sub = p.add_subparsers(dest="command", required=True)

    pr = sub.add_parser("record", help="Coleta + start_record + sequência go_to_point + stop_record")
    pr.add_argument("--robot", default="tb1", help="robot_id (use '' ou --single-robot para sim sem namespace)")
    pr.add_argument(
        "--single-robot",
        action="store_true",
        help="Força robot_id vazio (rotas/bags em routes/default, collections/default)",
    )
    pr.add_argument("--route", default="percurso1_tb1", help="nome da rota YAML (sem .yaml)")
    pr.add_argument(
        "--points",
        default="0.5,0,0;1.0,0,0;1.5,0.5,0;2.0,0.5,0",
        help="Pontos x,y,yaw separados por ';' (metros e rad)",
    )
    pr.add_argument(
        "--topics",
        nargs="+",
        default=["scan", "odom", "imu"],
        help="Tópicos relativos ao namespace do robô",
    )
    pr.add_argument("--wait-goal", type=float, default=120.0, help="Timeout por goal (s)")
    pr.add_argument(
        "--no-use-sim-time",
        action="store_true",
        help="Relógio de parede (sem /clock). Em Gazebo+Nav2 omita esta flag.",
    )
    pr.add_argument(
        "--require-motion",
        action="store_true",
        help="Falha cedo se não detectar deslocamento mínimo no /odom após iniciar navegação",
    )
    pr.add_argument("--motion-min-dist", type=float, default=0.10, help="Deslocamento mínimo (m)")
    pr.add_argument("--motion-timeout", type=float, default=10.0, help="Janela para detectar movimento (s)")
    pr.add_argument("--skip-collection", action="store_true", help="Só record/play sem rosbag")
    pr.add_argument(
        "--export",
        metavar="FILE.json",
        help="Salva metadados do run (rota, pontos, sucesso, rosbag_path quando disponível)",
    )
    pr.set_defaults(func=cmd_record)

    pb = sub.add_parser("replay", help="Coleta + play_route da rota salva")
    pb.add_argument("--robot", default="tb1")
    pb.add_argument("--single-robot", action="store_true", help="Força robot_id vazio")
    pb.add_argument("--route", default="percurso1_tb1")
    pb.add_argument("--topics", nargs="+", default=["scan", "odom", "imu"])
    pb.add_argument("--wait-goal", type=float, default=300.0, help="Timeout da rota completa (s)")
    pb.add_argument(
        "--no-use-sim-time",
        action="store_true",
        help="Relógio de parede (sem /clock). Em Gazebo+Nav2 omita esta flag.",
    )
    pb.add_argument(
        "--require-motion",
        action="store_true",
        help="Falha cedo se não detectar deslocamento mínimo no /odom após iniciar navegação",
    )
    pb.add_argument("--motion-min-dist", type=float, default=0.10, help="Deslocamento mínimo (m)")
    pb.add_argument("--motion-timeout", type=float, default=10.0, help="Janela para detectar movimento (s)")
    pb.add_argument("--skip-collection", action="store_true")
    pb.add_argument("--export", metavar="FILE.json", help="Salva metadados do run")
    pb.set_defaults(func=cmd_replay)

    args = p.parse_args()
    _apply_single_robot(args)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
