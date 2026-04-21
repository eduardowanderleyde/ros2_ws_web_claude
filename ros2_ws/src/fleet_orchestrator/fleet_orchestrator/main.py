#!/usr/bin/env python3
"""Orquestrador: record/play rotas, go_to_point, Nav2 NavigateThroughPoses, /fleet/status."""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from fleet_msgs.msg import FleetStatus, RobotState
from fleet_msgs.srv import (
    Cancel,
    GoToPoint,
    ListRobots,
    ListRoutes,
    PlayRoute,
    StartRecord,
    StopRecord,
)


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


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


@dataclass
class XYYaw:
    x: float
    y: float
    yaw: float


@dataclass
class RobotRuntime:
    nav_state: str = "idle"  # idle | recording | navigating | failed
    current_route: str = ""
    last_error: str = ""
    is_recording: bool = False
    is_navigating: bool = False
    route_poses: List[PoseStamped] = field(default_factory=list)
    last_saved: Optional[XYYaw] = None
    goal_handle = None
    pending_goal_pose: Optional[PoseStamped] = None  # pose atual em navegação


class FleetOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_orchestrator")
        self.declare_parameter("robots", ["tb1", "tb2", "tb3"])
        self.declare_parameter("routes_dir", "routes")
        self.declare_parameter("use_shared_map_frame", False)
        self.declare_parameter("record_rate_hz", 5.0)
        self.declare_parameter("min_dist_m", 0.10)
        self.declare_parameter("min_yaw_deg", 5.0)

        self._robots: List[str] = list(self.get_parameter("robots").value)
        self._routes_dir: str = str(self.get_parameter("routes_dir").value)
        self._use_shared: bool = bool(self.get_parameter("use_shared_map_frame").value)
        self._record_hz: float = float(self.get_parameter("record_rate_hz").value)
        self._min_dist: float = float(self.get_parameter("min_dist_m").value)
        self._min_yaw_rad: float = math.radians(float(self.get_parameter("min_yaw_deg").value))

        self._roles = self._load_roles()
        self._state: Dict[str, RobotRuntime] = {rid: RobotRuntime() for rid in self._robots}

        self._cb_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._nav_clients: Dict[str, ActionClient] = {}
        self._wp_clients: Dict[str, ActionClient] = {}
        for rid in self._robots:
            self._nav_clients[rid] = ActionClient(
                self, NavigateToPose, "/navigate_to_pose" if rid == "" else f"/{rid}/navigate_to_pose",
                callback_group=self._cb_group
            )
            self._wp_clients[rid] = ActionClient(
                self, FollowWaypoints, "/follow_waypoints" if rid == "" else f"/{rid}/follow_waypoints",
                callback_group=self._cb_group
            )

        self.create_service(StartRecord, "start_record", self._cb_start_record)
        self.create_service(StopRecord, "stop_record", self._cb_stop_record)
        self.create_service(PlayRoute, "play_route", self._cb_play_route)
        self.create_service(GoToPoint, "go_to_point", self._cb_go_to_point)
        self.create_service(Cancel, "cancel", self._cb_cancel)
        self.create_service(ListRobots, "list_robots", self._cb_list_robots)
        self.create_service(ListRoutes, "list_routes", self._cb_list_routes)

        self._pub_status = self.create_publisher(FleetStatus, "fleet/status", 10)
        self.create_timer(1.0, self._publish_status)

        period = 1.0 / max(self._record_hz, 0.1)
        self.create_timer(period, self._record_tick)

        self.get_logger().info(f"fleet_orchestrator ready robots={self._robots} shared_map={self._use_shared}")

    def _load_roles(self) -> Dict[str, str]:
        path = os.path.join(get_package_share_directory("fleet_orchestrator"), "config", "roles.yaml")
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return dict(data.get("roles", {}))
        except Exception as e:
            self.get_logger().warn(f"roles.yaml não carregado: {e}")
            return {}

    def _role(self, robot_id: str) -> str:
        if robot_id == "":
            return "MUUT"
        return self._roles.get(robot_id, "MUUT")

    def _motion_allowed(self, robot_id: str) -> bool:
        return self._role(robot_id) == "MUUT"

    def _known_robot(self, robot_id: str) -> bool:
        if robot_id not in self._state:
            if robot_id == "":
                self._state[""] = RobotRuntime()
                self._nav_clients[""] = ActionClient(
                    self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb_group
                )
                self._wp_clients[""] = ActionClient(
                    self, FollowWaypoints, "/follow_waypoints", callback_group=self._cb_group
                )
            else:
                return False
        return True

    def _subdir(self, robot_id: str) -> str:
        return "default" if robot_id == "" else robot_id

    def _robot_route_dir(self, robot_id: str) -> str:
        return os.path.join(self._routes_dir, self._subdir(robot_id))

    def _map_base(self, robot_id: str) -> tuple[str, str]:
        if robot_id == "":
            return "map", "base_link"
        return f"{robot_id}/map", f"{robot_id}/base_link"

    def _action_name(self, robot_id: str) -> str:
        if robot_id == "":
            return "/navigate_through_poses"
        return f"/{robot_id}/navigate_through_poses"

    def _get_pose(self, robot_id: str) -> Optional[PoseStamped]:
        map_f, base_f = self._map_base(robot_id)
        try:
            transform = self.tf_buffer.lookup_transform(
                map_f, base_f, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF {map_f}->{base_f}: {ex}")
            return None
        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = map_f
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _pose_xyyaw(self, pose: PoseStamped) -> XYYaw:
        return XYYaw(
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            quat_to_yaw(pose.pose.orientation),
        )

    def _should_save(self, st: RobotRuntime, pose: PoseStamped) -> bool:
        curr = self._pose_xyyaw(pose)
        if st.last_saved is None:
            return True
        dist = math.hypot(curr.x - st.last_saved.x, curr.y - st.last_saved.y)
        dyaw = wrap_angle(curr.yaw - st.last_saved.yaw)
        return dist >= self._min_dist or abs(dyaw) >= self._min_yaw_rad

    def _record_tick(self) -> None:
        for rid, st in self._state.items():
            if not st.is_recording:
                continue
            pose = self._get_pose(rid)
            if pose is None:
                continue
            if self._should_save(st, pose):
                st.route_poses.append(pose)
                st.last_saved = self._pose_xyyaw(pose)

    def _publish_status(self) -> None:
        msg = FleetStatus()
        for rid, st in self._state.items():
            rs = RobotState()
            rs.robot_id = rid
            rs.role = self._role(rid)
            rs.nav_state = st.nav_state
            rs.current_route = st.current_route
            rs.collection_on = False
            rs.collection_file = ""
            rs.last_error = st.last_error
            rs.bytes_written = 0
            msg.robots.append(rs)
        self._pub_status.publish(msg)

    def _save_route_yaml(self, robot_id: str, route_name: str, poses: List[PoseStamped]) -> str:
        map_f, _ = self._map_base(robot_id)
        out_dir = self._robot_route_dir(robot_id)
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"{route_name}.yaml")
        data = {"route_name": route_name, "frame": map_f, "poses": []}
        for p in poses:
            data["poses"].append(
                {
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "yaw": float(quat_to_yaw(p.pose.orientation)),
                }
            )
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return path

    def _load_route_yaml(self, robot_id: str, route_name: str) -> Optional[List[PoseStamped]]:
        path = os.path.join(self._robot_route_dir(robot_id), f"{route_name}.yaml")
        if not os.path.isfile(path):
            return None
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        map_f, _ = self._map_base(robot_id)
        frame = data.get("frame", map_f)
        poses_data = data.get("poses", [])
        if not poses_data:
            return None
        now_msg = self.get_clock().now().to_msg()
        out: List[PoseStamped] = []
        for item in poses_data:
            p = PoseStamped()
            p.header.frame_id = frame
            p.header.stamp = now_msg
            p.pose.position.x = float(item["x"])
            p.pose.position.y = float(item["y"])
            p.pose.position.z = 0.0
            p.pose.orientation = yaw_to_quat(float(item.get("yaw", 0.0)))
            out.append(p)
        return out

    def _send_nav_poses(self, robot_id: str, poses: List[PoseStamped], route_label: str) -> tuple[bool, str, str]:
        st = self._state[robot_id]
        if len(poses) == 1:
            client = self._nav_clients[robot_id]
            if not client.wait_for_server(timeout_sec=20.0):
                st.nav_state = "failed"
                st.last_error = "NAV2_UNAVAILABLE"
                return False, "Nav2 navigate_to_pose not available", "NAV2_UNAVAILABLE"
            goal = NavigateToPose.Goal()
            goal.pose = poses[0]
        else:
            client = self._wp_clients[robot_id]
            if not client.wait_for_server(timeout_sec=20.0):
                st.nav_state = "failed"
                st.last_error = "NAV2_UNAVAILABLE"
                return False, "Nav2 follow_waypoints not available", "NAV2_UNAVAILABLE"
            goal = FollowWaypoints.Goal()
            goal.poses = poses

        st.is_navigating = True
        st.nav_state = "navigating"
        st.last_error = ""
        if route_label:
            st.current_route = route_label

        st.pending_goal_pose = poses[0] if poses else None
        send_future = client.send_goal_async(goal, feedback_callback=self._make_feedback_cb(robot_id))
        send_future.add_done_callback(self._make_goal_response_cb(robot_id))
        return True, "Navigation started", ""

    def _make_feedback_cb(self, robot_id: str):
        def _cb(_fb) -> None:
            pass

        return _cb

    def _make_goal_response_cb(self, robot_id: str):
        def _cb(fut) -> None:
            st = self._state[robot_id]
            try:
                gh = fut.result()
            except Exception as ex:
                st.is_navigating = False
                st.nav_state = "recording" if st.is_recording else "failed"
                st.last_error = "NAV2_REJECTED"
                if not st.is_recording:
                    st.current_route = ""
                self.get_logger().error(f"Nav2 goal error {robot_id}: {ex}")
                return
            if not gh.accepted:
                st.is_navigating = False
                st.nav_state = "recording" if st.is_recording else "failed"
                st.last_error = "NAV2_REJECTED"
                if not st.is_recording:
                    st.current_route = ""
                self.get_logger().error(f"Nav2 rejected goal {robot_id}")
                return
            st.goal_handle = gh
            rh = gh.get_result_async()
            rh.add_done_callback(self._make_result_cb(robot_id))
        return _cb

    def _make_result_cb(self, robot_id: str):
        def _cb(fut) -> None:
            st = self._state[robot_id]
            st.is_navigating = False
            st.goal_handle = None
            # Don't clear current_route while recording — stop_record needs it
            if not st.is_recording:
                st.current_route = ""
            try:
                res = fut.result()
                status = res.status
            except Exception as ex:
                st.nav_state = "recording" if st.is_recording else "failed"
                st.last_error = "NAV2_FAILED"
                self.get_logger().error(f"Nav2 result {robot_id}: {ex}")
                return
            if status == GoalStatus.STATUS_SUCCEEDED:
                # Durante recording: salva waypoint atingido diretamente (não depende de TF)
                if st.is_recording and st.pending_goal_pose is not None:
                    st.route_poses.append(st.pending_goal_pose)
                    self.get_logger().info(
                        f"[record] pose salva #{len(st.route_poses)}: "
                        f"x={st.pending_goal_pose.pose.position.x:.3f} "
                        f"y={st.pending_goal_pose.pose.position.y:.3f}"
                    )
                st.pending_goal_pose = None
                st.nav_state = "recording" if st.is_recording else "idle"
                st.last_error = ""
            else:
                st.pending_goal_pose = None
                st.nav_state = "recording" if st.is_recording else "failed"
                st.last_error = "NAV2_ABORTED"
        return _cb

    def _cb_list_robots(self, req: ListRobots.Request, resp: ListRobots.Response) -> ListRobots.Response:
        resp.robot_ids = list(self._robots)
        return resp

    def _cb_list_routes(self, req: ListRoutes.Request, resp: ListRoutes.Response) -> ListRoutes.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.route_names = []
            return resp
        d = self._robot_route_dir(rid)
        if not os.path.isdir(d):
            resp.route_names = []
            return resp
        names = []
        for f in os.listdir(d):
            if f.endswith(".yaml"):
                names.append(f[:-5])
        resp.route_names = sorted(names)
        return resp

    def _cb_start_record(self, req: StartRecord.Request, resp: StartRecord.Response) -> StartRecord.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.success = False
            resp.message = f"Unknown robot {rid!r}"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        st = self._state[rid]
        if st.is_navigating:
            resp.success = False
            resp.message = "Cannot start recording while navigating"
            resp.error_code = "ALREADY_NAVIGATING"
            return resp
        st.route_poses.clear()
        st.last_saved = None
        st.is_recording = True
        st.nav_state = "recording"
        st.current_route = req.route_name
        st.last_error = ""
        resp.success = True
        resp.message = "Recording started"
        resp.error_code = ""
        return resp

    def _cb_stop_record(self, req: StopRecord.Request, resp: StopRecord.Response) -> StopRecord.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.success = False
            resp.message = f"Unknown robot {rid!r}"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        st = self._state[rid]
        route_name = st.current_route
        st.is_recording = False
        n = len(st.route_poses)
        if n == 0:
            st.nav_state = "idle"
            st.current_route = ""
            resp.success = True
            resp.message = "Route length: 0"
            resp.error_code = ""
            return resp
        path = self._save_route_yaml(rid, route_name, st.route_poses)
        st.route_poses.clear()
        st.last_saved = None
        st.nav_state = "idle"
        st.current_route = ""
        resp.success = True
        resp.message = f"Saved {n} poses to {path}"
        resp.error_code = ""
        return resp

    def _cb_play_route(self, req: PlayRoute.Request, resp: PlayRoute.Response) -> PlayRoute.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.success = False
            resp.message = "UNKNOWN_ROBOT"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        if not self._motion_allowed(rid):
            resp.success = False
            resp.message = "ROLE_NOT_ALLOWED"
            resp.error_code = "ROLE_NOT_ALLOWED"
            return resp
        st = self._state[rid]
        if st.is_recording:
            resp.success = False
            resp.message = "Stop recording before play"
            resp.error_code = "ALREADY_RECORDING"
            return resp
        if st.is_navigating:
            resp.success = False
            resp.message = "Already navigating"
            resp.error_code = "ALREADY_NAVIGATING"
            return resp
        poses = self._load_route_yaml(rid, req.route_name)
        if not poses:
            resp.success = False
            resp.message = f"Route not found: {req.route_name}"
            resp.error_code = "ROUTE_NOT_FOUND"
            return resp
        ok, msg, code = self._send_nav_poses(rid, poses, req.route_name)
        resp.success = ok
        resp.message = msg
        resp.error_code = code
        return resp

    def _cb_go_to_point(self, req: GoToPoint.Request, resp: GoToPoint.Response) -> GoToPoint.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.success = False
            resp.message = "UNKNOWN_ROBOT"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        if not self._motion_allowed(rid):
            resp.success = False
            resp.message = "ROLE_NOT_ALLOWED"
            resp.error_code = "ROLE_NOT_ALLOWED"
            return resp
        st = self._state[rid]
        # Durante start_record o robô precisa navegar para amostrar a rota (TF).
        if st.is_navigating:
            resp.success = False
            resp.message = "Already navigating"
            resp.error_code = "ALREADY_NAVIGATING"
            return resp
        map_f, _ = self._map_base(rid)
        p = PoseStamped()
        p.header.frame_id = map_f
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(req.x)
        p.pose.position.y = float(req.y)
        p.pose.position.z = 0.0
        p.pose.orientation = yaw_to_quat(float(req.yaw))
        ok, msg, code = self._send_nav_poses(rid, [p], "")
        resp.success = ok
        resp.message = msg
        resp.error_code = code
        return resp

    def _cb_cancel(self, req: Cancel.Request, resp: Cancel.Response) -> Cancel.Response:
        rid = req.robot_id
        if not self._known_robot(rid):
            resp.success = False
            resp.message = "UNKNOWN_ROBOT"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        st = self._state[rid]
        if st.goal_handle is None:
            resp.success = False
            resp.message = "No active goal"
            resp.error_code = "CANCEL_FAILED"
            return resp
        cancel_future = st.goal_handle.cancel_goal_async()
        st.goal_handle = None
        st.is_navigating = False
        st.nav_state = "idle"
        st.current_route = ""
        cancel_future.add_done_callback(lambda _f: None)
        resp.success = True
        resp.message = "Cancel requested"
        resp.error_code = ""
        return resp


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FleetOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
