#!/usr/bin/env python3
"""Fleet orchestrator: record/save/play routes and send Nav2 goals per robot_id. UI-ready API."""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener, TransformException
from nav2_msgs.action import NavigateThroughPoses

from fleet_msgs.msg import FleetStatus, RobotState
from fleet_msgs.srv import (
    StartRecord,
    StopRecord,
    PlayRoute,
    ListRobots,
    ListRoutes,
    Cancel,
    GoToPoint,
    CollectionStatus,
)

import yaml


@dataclass
class XYYaw:
    x: float
    y: float
    yaw: float


# Códigos de erro para UI (response.error_code)
class ErrorCode:
    UNKNOWN_ROBOT = "UNKNOWN_ROBOT"
    ALREADY_NAVIGATING = "ALREADY_NAVIGATING"
    ALREADY_RECORDING = "ALREADY_RECORDING"
    ROUTE_NOT_FOUND = "ROUTE_NOT_FOUND"
    NAV2_UNAVAILABLE = "NAV2_UNAVAILABLE"
    NAV2_REJECTED = "NAV2_REJECTED"
    TF_MISSING = "TF_MISSING"
    ROLE_NOT_ALLOWED = "ROLE_NOT_ALLOWED"


@dataclass
class RobotRouteState:
    route: List[PoseStamped] = field(default_factory=list)
    is_recording: bool = False
    is_navigating: bool = False
    route_name: str = ""
    _last_saved: Optional[XYYaw] = None
    _goal_handle = None
    _last_error: str = ""  # para /fleet/status (TF_MISSING, NAV2_REJECTED, etc.)


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FleetOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_orchestrator")

        self.declare_parameter("robots", ["tb1", "tb2", "tb3"])
        self.declare_parameter("routes_dir", "routes")
        self.declare_parameter("record_rate_hz", 5.0)
        self.declare_parameter("min_dist_m", 0.10)
        self.declare_parameter("min_yaw_deg", 5.0)
        self.declare_parameter("nav2_action_suffix", "navigate_through_poses")
        self.declare_parameter("frame_map_suffix", "map")
        self.declare_parameter("frame_base_suffix", "base_link")
        # Se True: usa "map" (sem prefixo) para todos os robôs (SLAM publica só "map").
        # Se False: usa "tbX/map" por robô (multi-mapa).
        self.declare_parameter("use_shared_map_frame", False)
        # YAML opcional com roles (MUUT/FUUT/SU) por robô
        # Relativo à raiz do pacote fleet_orchestrator (onde está config/roles.yaml)
        self.declare_parameter("roles_config", "config/roles.yaml")

        self.robots: List[str] = self.get_parameter("robots").value
        self.routes_dir: str = self.get_parameter("routes_dir").value
        self.record_rate_hz: float = float(self.get_parameter("record_rate_hz").value)
        self.min_dist_m: float = float(self.get_parameter("min_dist_m").value)
        self.min_yaw_rad: float = math.radians(float(self.get_parameter("min_yaw_deg").value))
        self.nav2_action_suffix: str = self.get_parameter("nav2_action_suffix").value
        self.frame_map_suffix: str = self.get_parameter("frame_map_suffix").value
        self.frame_base_suffix: str = self.get_parameter("frame_base_suffix").value
        self.use_shared_map_frame: bool = self.get_parameter("use_shared_map_frame").value
        self.roles_config: str = self.get_parameter("roles_config").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._state: Dict[str, RobotRouteState] = {
            rid: RobotRouteState() for rid in self.robots
        }
        self._roles: Dict[str, str] = self._load_roles()
        self._nav2_clients: Dict[str, ActionClient] = {}
        self._last_feedback_log_ns: Dict[str, int] = {}
        self._collection_status_client = self.create_client(CollectionStatus, "collection_status")

        self._status_pub = self.create_publisher(FleetStatus, "fleet/status", 10)
        self._status_timer = self.create_timer(1.0, self._publish_fleet_status)

        record_period = 1.0 / max(self.record_rate_hz, 0.1)
        self._record_timer = self.create_timer(record_period, self._record_timer_callback)

        self._start_record_srv = self.create_service(
            StartRecord, "start_record", self._handle_start_record
        )
        self._stop_record_srv = self.create_service(
            StopRecord, "stop_record", self._handle_stop_record
        )
        self._play_route_srv = self.create_service(
            PlayRoute, "play_route", self._handle_play_route
        )
        self._list_robots_srv = self.create_service(
            ListRobots, "list_robots", self._handle_list_robots
        )
        self._list_routes_srv = self.create_service(
            ListRoutes, "list_routes", self._handle_list_routes
        )
        self._cancel_srv = self.create_service(Cancel, "cancel", self._handle_cancel)
        self._go_to_point_srv = self.create_service(GoToPoint, "go_to_point", self._handle_go_to_point)

        self.get_logger().info(
            f"fleet_orchestrator ready. Robots: {self.robots}. Roles: {self._roles}. "
            "API: start_record, stop_record, play_route, go_to_point, cancel, list_robots, list_routes."
        )

    def _role(self, robot_id: str) -> str:
        return self._roles.get(robot_id, "MUUT")

    def _is_mobile_role(self, robot_id: str) -> bool:
        return self._role(robot_id) == "MUUT"

    def _load_roles(self) -> Dict[str, str]:
        """Carrega roles por robô a partir de um YAML opcional."""
        path = self.roles_config
        roles: Dict[str, str] = {}
        if not path:
            return roles
        try:
            if not os.path.isabs(path):
                # Desenvolvimento: .../src/fleet_orchestrator/config/roles.yaml
                base = os.path.dirname(os.path.dirname(__file__))
                candidate = os.path.abspath(os.path.join(base, path))
                if os.path.exists(candidate):
                    path = candidate
                else:
                    # Instalado: .../share/fleet_orchestrator/config/roles.yaml
                    try:
                        from ament_index_python.packages import get_package_share_directory

                        share = get_package_share_directory("fleet_orchestrator")
                        alt = os.path.join(share, path)
                        if os.path.exists(alt):
                            path = alt
                        else:
                            path = candidate
                    except Exception:
                        path = candidate
            if not os.path.exists(path):
                self.get_logger().warn(f"roles_config não encontrado: {path}")
                return roles
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            raw = data.get("roles", {}) or {}
            for rid in self.robots:
                r = rid or "default"
                roles[rid] = str(raw.get(r, "MUUT"))
            return roles
        except Exception as e:
            self.get_logger().warn(f"Falha ao carregar roles_config ({path}): {e}")
            return {}

    def _global_frame(self, robot_id: str) -> str:
        if not robot_id:
            return "map"
        if self.use_shared_map_frame:
            return "map"
        return f"{robot_id}/{self.frame_map_suffix}"

    def _base_frame(self, robot_id: str) -> str:
        if not robot_id:
            return "base_link"
        return f"{robot_id}/{self.frame_base_suffix}"

    def _nav2_action_name(self, robot_id: str) -> str:
        if not robot_id:
            return f"/{self.nav2_action_suffix}"
        return f"/{robot_id}/{self.nav2_action_suffix}"

    def _robot_routes_dir(self, robot_id: str) -> str:
        # Sim sem namespace: robot_id vazio -> routes/default
        key = robot_id if robot_id else "default"
        return os.path.join(self.routes_dir, key)

    def _publish_fleet_status(self) -> None:
        msg = FleetStatus()
        for rid in self.robots:
            rs = RobotState()
            rs.robot_id = rid if rid else "default"
            rs.role = self._roles.get(rid, "MUUT")
            state = self._state[rid]
            if state.is_recording:
                rs.nav_state = "recording"
                rs.current_route = state.route_name or ""
            elif state.is_navigating:
                rs.nav_state = "navigating"
                rs.current_route = state.route_name or ""
            elif state._last_error:
                rs.nav_state = "failed"
                rs.current_route = ""
            else:
                rs.nav_state = "idle"
                rs.current_route = ""
            rs.last_error = state._last_error or ""
            rs.collection_on = False
            rs.collection_file = ""
            rs.bytes_written = 0
            if self._collection_status_client.service_is_ready():
                req = CollectionStatus.Request()
                req.robot_id = rid
                future = self._collection_status_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
                try:
                    if future.done() and future.result() is not None:
                        r = future.result()
                        rs.collection_on = r.is_collecting
                        rs.collection_file = r.current_file
                        rs.bytes_written = r.bytes_written
                except Exception:
                    pass
            msg.robots.append(rs)
        self._status_pub.publish(msg)

    def _get_nav2_client(self, robot_id: str) -> ActionClient:
        if robot_id not in self._nav2_clients:
            name = self._nav2_action_name(robot_id)
            self._nav2_clients[robot_id] = ActionClient(
                self, NavigateThroughPoses, name
            )
        return self._nav2_clients[robot_id]

    def _handle_start_record(
        self, request: StartRecord.Request, response: StartRecord.Response
    ) -> StartRecord.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}. Known: {self.robots}."
            response.error_code = ErrorCode.UNKNOWN_ROBOT
            return response
        if not self._is_mobile_role(robot_id):
            role = self._role(robot_id)
            response.success = False
            response.message = f"Role {role} is fixed/support. start_record allowed only for MUUT."
            response.error_code = ErrorCode.ROLE_NOT_ALLOWED
            return response

        state = self._state[robot_id]
        if state.is_navigating:
            response.success = False
            response.message = "Cannot start recording while robot is navigating."
            response.error_code = ErrorCode.ALREADY_NAVIGATING
            return response

        state.route.clear()
        state._last_saved = None
        state._last_error = ""
        state.is_recording = True
        state.route_name = request.route_name.strip() or "route1"

        self.get_logger().info(f"[{robot_id}] Recording started. route_name={state.route_name}")
        response.success = True
        response.message = f"Recording started for {robot_id}."
        response.error_code = ""
        return response

    def _handle_stop_record(
        self, request: StopRecord.Request, response: StopRecord.Response
    ) -> StopRecord.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}."
            response.error_code = ErrorCode.UNKNOWN_ROBOT
            return response
        if not self._is_mobile_role(robot_id):
            role = self._role(robot_id)
            response.success = False
            response.message = f"Role {role} is fixed/support. stop_record allowed only for MUUT."
            response.error_code = ErrorCode.ROLE_NOT_ALLOWED
            return response

        state = self._state[robot_id]
        state.is_recording = False

        count = len(state.route)
        saved_path = None
        if count > 0:
            saved_path = self._save_route_yaml(robot_id)

        msg = f"Recording stopped for {robot_id}. Route length: {count}."
        if saved_path:
            msg += f" Saved: {saved_path}"
        self.get_logger().info(msg)

        response.success = True
        response.message = msg
        response.error_code = ""
        return response

    def _handle_play_route(
        self, request: PlayRoute.Request, response: PlayRoute.Response
    ) -> PlayRoute.Response:
        robot_id = request.robot_id.strip()
        route_name = (request.route_name or "").strip() or "route1"

        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}."
            response.error_code = ErrorCode.UNKNOWN_ROBOT
            return response
        if not self._is_mobile_role(robot_id):
            role = self._role(robot_id)
            response.success = False
            response.message = f"Role {role} is fixed/support. play_route allowed only for MUUT."
            response.error_code = ErrorCode.ROLE_NOT_ALLOWED
            return response

        state = self._state[robot_id]
        if state.is_recording:
            response.success = False
            response.message = "Stop recording before playing."
            response.error_code = ErrorCode.ALREADY_RECORDING
            return response
        if state.is_navigating:
            response.success = False
            response.message = "Robot is already navigating."
            response.error_code = ErrorCode.ALREADY_NAVIGATING
            return response

        if not state.route or state.route_name != route_name:
            if not self._load_route_yaml(robot_id, route_name):
                response.success = False
                response.message = f"Could not load route '{route_name}' for {robot_id}."
                response.error_code = ErrorCode.ROUTE_NOT_FOUND
                return response
            state.route_name = route_name

        client = self._get_nav2_client(robot_id)
        if not client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = f"Nav2 action server not available for {robot_id}."
            response.error_code = ErrorCode.NAV2_UNAVAILABLE
            state._last_error = ErrorCode.NAV2_UNAVAILABLE
            return response

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = state.route

        state._last_error = ""
        state.is_navigating = True
        self.get_logger().info(f"[{robot_id}] Sending route to Nav2: {len(state.route)} poses.")

        send_future = client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._nav2_feedback_cb(robot_id, fb)
        )
        send_future.add_done_callback(
            lambda f: self._nav2_goal_response_cb(robot_id, f)
        )

        response.success = True
        response.message = f"Nav2 navigation started for {robot_id}."
        response.error_code = ""
        return response

    def _handle_go_to_point(
        self, request: GoToPoint.Request, response: GoToPoint.Response
    ) -> GoToPoint.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}. Known: {self.robots}."
            response.error_code = ErrorCode.UNKNOWN_ROBOT
            return response
        if not self._is_mobile_role(robot_id):
            role = self._role(robot_id)
            response.success = False
            response.message = f"Role {role} is fixed/support. go_to_point allowed only for MUUT."
            response.error_code = ErrorCode.ROLE_NOT_ALLOWED
            return response

        state = self._state[robot_id]
        if state.is_recording:
            response.success = False
            response.message = "Stop recording before sending goal."
            response.error_code = ErrorCode.ALREADY_RECORDING
            return response
        if state.is_navigating:
            response.success = False
            response.message = "Robot is already navigating. Cancel first or wait."
            response.error_code = ErrorCode.ALREADY_NAVIGATING
            return response

        client = self._get_nav2_client(robot_id)
        if not client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = f"Nav2 action server not available for {robot_id}."
            response.error_code = ErrorCode.NAV2_UNAVAILABLE
            state._last_error = ErrorCode.NAV2_UNAVAILABLE
            return response

        frame_id = self._global_frame(robot_id)
        yaw = float(request.yaw) if request.yaw != 0.0 else 0.0

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(request.x)
        pose.pose.position.y = float(request.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quat(yaw)

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [pose]

        state._last_error = ""
        state.is_navigating = True
        self.get_logger().info(f"[{robot_id}] GoToPoint ({request.x:.2f}, {request.y:.2f}) -> Nav2.")

        send_future = client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._nav2_feedback_cb(robot_id, fb)
        )
        send_future.add_done_callback(
            lambda f: self._nav2_goal_response_cb(robot_id, f)
        )

        response.success = True
        response.message = f"Goal sent for {robot_id}. Navigating to ({request.x:.2f}, {request.y:.2f})."
        response.error_code = ""
        return response

    def _handle_list_robots(
        self, request: ListRobots.Request, response: ListRobots.Response
    ) -> ListRobots.Response:
        response.robot_ids = list(self.robots)
        return response

    def _handle_list_routes(
        self, request: ListRoutes.Request, response: ListRoutes.Response
    ) -> ListRoutes.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            return response

        base = self._robot_routes_dir(robot_id)
        if not os.path.isdir(base):
            return response

        names = []
        for f in os.listdir(base):
            if f.endswith(".yaml"):
                names.append(f[:-5])
        response.route_names = sorted(names)
        return response

    def _handle_cancel(
        self, request: Cancel.Request, response: Cancel.Response
    ) -> Cancel.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}."
            response.error_code = ErrorCode.UNKNOWN_ROBOT
            return response

        state = self._state[robot_id]
        if not state.is_navigating or state._goal_handle is None:
            response.success = True
            response.message = f"No active navigation for {robot_id}."
            response.error_code = ""
            return response

        try:
            state._goal_handle.cancel_goal_async()
            state.is_navigating = False
            state._goal_handle = None
            state._last_error = ""
            self.get_logger().info(f"[{robot_id}] Navigation cancelled.")
            response.success = True
            response.message = f"Cancelled navigation for {robot_id}."
            response.error_code = ""
        except Exception as ex:
            self.get_logger().error(f"Cancel failed: {ex}")
            response.success = False
            response.message = str(ex)
            response.error_code = "CANCEL_FAILED"
        return response

    def _record_timer_callback(self) -> None:
        for robot_id, state in self._state.items():
            if not state.is_recording:
                continue
            pose = self._get_current_pose(robot_id)
            if pose is None:
                state._last_error = ErrorCode.TF_MISSING
                continue
            state._last_error = ""
            if self._should_save_pose(robot_id, pose):
                state.route.append(pose)
                state._last_saved = self._pose_to_xyyaw(pose)

    def _nav2_goal_response_cb(self, robot_id: str, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[{robot_id}] Nav2 rejected the goal.")
            self._state[robot_id].is_navigating = False
            self._state[robot_id]._last_error = ErrorCode.NAV2_REJECTED
            return

        self._state[robot_id]._goal_handle = goal_handle
        self.get_logger().info(f"[{robot_id}] Nav2 goal accepted. Navigating...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._nav2_result_cb(robot_id, f))

    def _nav2_result_cb(self, robot_id: str, future) -> None:
        try:
            result = future.result().result
            status = future.result().status
        except Exception as ex:
            self.get_logger().error(f"[{robot_id}] Nav2 result error: {ex}")
            self._state[robot_id].is_navigating = False
            return

        self.get_logger().info(f"[{robot_id}] Nav2 finished. status={status}")
        self._state[robot_id].is_navigating = False
        self._state[robot_id]._goal_handle = None
        self._state[robot_id]._last_error = ""

    def _nav2_feedback_cb(self, robot_id: str, feedback_msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_feedback_log_ns.get(robot_id, 0)) < int(2e9):
            return
        self._last_feedback_log_ns[robot_id] = now_ns
        try:
            fb = feedback_msg.feedback
            self.get_logger().info(f"[{robot_id}] Nav2 feedback: distance_remaining={fb.distance_remaining:.2f}")
        except Exception:
            self.get_logger().info(f"[{robot_id}] Nav2 feedback received.")

    def _get_current_pose(self, robot_id: str) -> Optional[PoseStamped]:
        global_frame = self._global_frame(robot_id)
        base_frame = self._base_frame(robot_id)
        try:
            transform = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().warn(f"[{robot_id}] TF failed {global_frame}->{base_frame}: {ex}")
            return None

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _pose_to_xyyaw(self, pose: PoseStamped) -> XYYaw:
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        yaw = quat_to_yaw(pose.pose.orientation)
        return XYYaw(x=x, y=y, yaw=yaw)

    def _should_save_pose(self, robot_id: str, pose: PoseStamped) -> bool:
        state = self._state[robot_id]
        curr = self._pose_to_xyyaw(pose)
        if state._last_saved is None:
            return True
        dx = curr.x - state._last_saved.x
        dy = curr.y - state._last_saved.y
        dist = math.hypot(dx, dy)
        dyaw = wrap_angle(curr.yaw - state._last_saved.yaw)
        return (dist >= self.min_dist_m) or (abs(dyaw) >= self.min_yaw_rad)

    def _yaw_to_quat(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _save_route_yaml(self, robot_id: str) -> Optional[str]:
        state = self._state[robot_id]
        base = self._robot_routes_dir(robot_id)
        os.makedirs(base, exist_ok=True)
        path = os.path.join(base, f"{state.route_name}.yaml")
        global_frame = self._global_frame(robot_id)

        data = {
            "route_name": state.route_name,
            "frame": global_frame,
            "poses": [],
        }
        for p in state.route:
            yaw = quat_to_yaw(p.pose.orientation)
            data["poses"].append({
                "x": float(p.pose.position.x),
                "y": float(p.pose.position.y),
                "yaw": float(yaw),
            })

        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return path

    def _load_route_yaml(self, robot_id: str, route_name: str) -> bool:
        base = self._robot_routes_dir(robot_id)
        path = os.path.join(base, f"{route_name}.yaml")
        if not os.path.exists(path):
            self.get_logger().error(f"Route file not found: {path}")
            return False

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        global_frame = data.get("frame", self._global_frame(robot_id))
        poses = data.get("poses", [])
        if not poses:
            self.get_logger().error(f"No poses in route file: {path}")
            return False

        now_msg = self.get_clock().now().to_msg()
        loaded: List[PoseStamped] = []
        for item in poses:
            p = PoseStamped()
            p.header.frame_id = global_frame
            p.header.stamp = now_msg
            p.pose.position.x = float(item["x"])
            p.pose.position.y = float(item["y"])
            p.pose.position.z = 0.0
            p.pose.orientation = self._yaw_to_quat(float(item.get("yaw", 0.0)))
            loaded.append(p)

        self._state[robot_id].route = loaded
        self.get_logger().info(f"[{robot_id}] Loaded route from YAML: {path} ({len(loaded)} poses)")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
