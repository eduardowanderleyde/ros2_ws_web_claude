#!/usr/bin/env python3
"""Coleta rosbag2 por robot_id (enable/disable)."""
from __future__ import annotations

import os
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional, Type

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.subscription import Subscription

import rosbag2_py
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata

from fleet_msgs.srv import CollectionStatus, DisableCollection, EnableCollection

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


@dataclass
class RobotSession:
    is_collecting: bool = False
    bag_uri: str = ""
    writer: Optional[SequentialWriter] = None
    bytes_written: int = 0
    subscriptions: List[Subscription] = field(default_factory=list)


class SensorCollector(Node):
    # Tópicos com prefixo de robot_id
    _TYPE_MAP = {
        "scan": ("sensor_msgs/msg/LaserScan", LaserScan),
        "odom": ("nav_msgs/msg/Odometry", Odometry),
        "imu":  ("sensor_msgs/msg/Imu", Imu),
        # Localização: inclua apenas um destes conforme o modo usado
        "amcl_pose": ("geometry_msgs/msg/PoseWithCovarianceStamped", PoseWithCovarianceStamped),  # AMCL (mapa fixo) ← padrão recomendado
        "pose":      ("geometry_msgs/msg/PoseWithCovarianceStamped", PoseWithCovarianceStamped),  # SLAM Toolbox (live)
    }
    # Tópicos globais (sem prefixo de robot_id, nome fixo)
    _GLOBAL_TOPICS = {
        "tf":        ("/tf",        "tf2_msgs/msg/TFMessage", TFMessage),
        "tf_static": ("/tf_static", "tf2_msgs/msg/TFMessage", TFMessage),
    }

    def __init__(self) -> None:
        super().__init__("sensor_collector")
        self.declare_parameter("robots", ["tb1", "tb2", "tb3"])
        self.declare_parameter("collections_dir", "collections")

        self._robots: List[str] = list(self.get_parameter("robots").value)
        self._collections_dir: str = str(self.get_parameter("collections_dir").value)
        self._sessions: Dict[str, RobotSession] = {rid: RobotSession() for rid in self._robots}

        self.create_service(EnableCollection, "enable_collection", self._cb_enable)
        self.create_service(DisableCollection, "disable_collection", self._cb_disable)
        self.create_service(CollectionStatus, "collection_status", self._cb_status)

        self.get_logger().info(f"sensor_collector ready robots={self._robots} dir={self._collections_dir}")

    def _topic_name(self, robot_id: str, short: str) -> str:
        if robot_id == "":
            return f"/{short}"
        return f"/{robot_id}/{short}"

    def _subdir(self, robot_id: str) -> str:
        return "default" if robot_id == "" else robot_id

    def _known(self, robot_id: str) -> bool:
        if robot_id not in self._sessions:
            if robot_id == "":
                self._sessions[""] = RobotSession()
            else:
                return False
        return True

    def _qos_sensor(self) -> QoSProfile:
        """BEST_EFFORT para sensores (scan, odom, imu) — compatível com Gazebo."""
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _qos_reliable(self) -> QoSProfile:
        """RELIABLE + VOLATILE — compatível com SLAM Toolbox /pose."""
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _qos_transient(self) -> QoSProfile:
        """RELIABLE + TRANSIENT_LOCAL — obrigatório para /amcl_pose (AMCL publica latched)."""
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    # Tópicos com QoS RELIABLE + VOLATILE (SLAM Toolbox)
    _RELIABLE_TOPICS = {"pose"}
    # Tópicos com QoS RELIABLE + TRANSIENT_LOCAL (AMCL publica latched)
    _TRANSIENT_LOCAL_TOPICS = {"amcl_pose"}

    def _qos_tf(self) -> QoSProfile:
        return QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _make_writer(self, bag_dir: str) -> SequentialWriter:
        storage = StorageOptions(uri=bag_dir, storage_id="mcap")
        converter = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        w = SequentialWriter()
        w.open(storage, converter)
        return w

    def _cb_enable(self, req: EnableCollection.Request, resp: EnableCollection.Response) -> EnableCollection.Response:
        rid = req.robot_id
        if not self._known(rid):
            resp.success = False
            resp.message = "UNKNOWN_ROBOT"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        sess = self._sessions[rid]
        if sess.is_collecting:
            resp.success = False
            resp.message = "Already collecting"
            resp.error_code = "ALREADY_COLLECTING"
            return resp
        if req.output_mode != "rosbag2":
            resp.success = False
            resp.message = f"Unsupported output_mode {req.output_mode!r}"
            resp.error_code = "UNSUPPORTED_OUTPUT_MODE"
            return resp

        # resolved: (topic_name, type_str, msg_cls, is_global)
        resolved: List[tuple[str, str, Type, bool]] = []
        for short in req.topics:
            s = short.strip().lstrip("/")
            if s in self._GLOBAL_TOPICS:
                tname, type_str, cls = self._GLOBAL_TOPICS[s]
                resolved.append((tname, type_str, cls, True))
            elif s in self._TYPE_MAP:
                tname = self._topic_name(rid, s)
                type_str, cls = self._TYPE_MAP[s]
                resolved.append((tname, type_str, cls, False))
            else:
                resp.success = False
                resp.message = f"Unknown topic short name: {short!r}"
                resp.error_code = "NO_VALID_TOPICS"
                return resp

        if not resolved:
            resp.success = False
            resp.message = "No topics"
            resp.error_code = "NO_VALID_TOPICS"
            return resp

        ts = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%S_%f")
        # Sufixo único (colisão de nome no disco).
        bag_name = f"{ts}_{uuid.uuid4().hex[:8]}"
        parent = os.path.join(self._collections_dir, self._subdir(rid))
        os.makedirs(parent, exist_ok=True)
        bag_dir = os.path.join(parent, bag_name)
        # Não criar bag_dir aqui: rosbag2 SequentialWriter.open() exige que o URI do bag
        # ainda não exista ("can't overwrite existing bag" se a pasta já foi criada vazia).

        try:
            writer = self._make_writer(bag_dir)
        except Exception as e:
            resp.success = False
            resp.message = str(e)
            resp.error_code = "ENABLE_FAILED"
            return resp

        sess.writer = writer
        sess.bag_uri = bag_dir
        sess.bytes_written = 0
        sess.is_collecting = True

        topic_id = 0
        for tname, type_str, msg_cls, is_global in resolved:
            meta = TopicMetadata(topic_id, tname, type_str, "cdr", [])
            topic_id += 1
            writer.create_topic(meta)

            def make_cb(full_name: str, cls: Type, global_topic: bool):
                def _write(msg) -> None:
                    if sess.writer is None:
                        return
                    try:
                        data = serialize_message(msg)
                        # TFMessage: usa stamp do primeiro transform se disponível
                        if global_topic and hasattr(msg, "transforms") and msg.transforms:
                            stamp_ns = Time.from_msg(msg.transforms[0].header.stamp).nanoseconds
                            if stamp_ns == 0:
                                stamp_ns = self.get_clock().now().nanoseconds
                        elif hasattr(msg, "header") and msg.header.stamp.sec != 0:
                            stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
                        else:
                            stamp_ns = self.get_clock().now().nanoseconds
                        sess.writer.write(full_name, data, stamp_ns)
                        sess.bytes_written += len(data)
                    except Exception as ex:
                        self.get_logger().warn(f"write {full_name}: {ex}")

                return _write

            short_name = tname.lstrip("/").split("/")[-1]
            if is_global:
                qos = self._qos_tf()
            elif short_name in self._TRANSIENT_LOCAL_TOPICS:
                qos = self._qos_transient()
            elif short_name in self._RELIABLE_TOPICS:
                qos = self._qos_reliable()
            else:
                qos = self._qos_sensor()
            sub = self.create_subscription(
                msg_cls,
                tname,
                make_cb(tname, msg_cls, is_global),
                qos,
            )
            sess.subscriptions.append(sub)

        resp.success = True
        resp.message = f"Collection enabled. Bag: {bag_dir}"
        resp.error_code = ""
        return resp

    def _cb_disable(self, req: DisableCollection.Request, resp: DisableCollection.Response) -> DisableCollection.Response:
        rid = req.robot_id
        if not self._known(rid):
            resp.success = False
            resp.message = "UNKNOWN_ROBOT"
            resp.error_code = "UNKNOWN_ROBOT"
            return resp
        sess = self._sessions[rid]
        if not sess.is_collecting:
            resp.success = True
            resp.message = "Collection was off"
            resp.error_code = ""
            return resp

        path = sess.bag_uri
        for sub in sess.subscriptions:
            self.destroy_subscription(sub)
        sess.subscriptions.clear()
        if sess.writer is not None:
            try:
                sess.writer.close()
            except Exception:
                pass
            sess.writer = None
        sess.is_collecting = False
        sess.bytes_written = 0

        resp.success = True
        resp.message = f"Collection disabled for {rid or 'default'}. Bag: {path}"
        resp.error_code = ""
        return resp

    def _cb_status(self, req: CollectionStatus.Request, resp: CollectionStatus.Response) -> CollectionStatus.Response:
        rid = req.robot_id
        if not self._known(rid):
            resp.is_collecting = False
            resp.current_file = ""
            resp.bytes_written = 0
            resp.message = "UNKNOWN_ROBOT"
            return resp
        sess = self._sessions[rid]
        resp.is_collecting = sess.is_collecting
        resp.current_file = sess.bag_uri if sess.is_collecting else ""
        resp.bytes_written = sess.bytes_written
        resp.message = "ok"
        return resp


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SensorCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
