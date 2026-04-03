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

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan


@dataclass
class RobotSession:
    is_collecting: bool = False
    bag_uri: str = ""
    writer: Optional[SequentialWriter] = None
    bytes_written: int = 0
    subscriptions: List[Subscription] = field(default_factory=list)


class SensorCollector(Node):
    _TYPE_MAP = {
        "scan": ("sensor_msgs/msg/LaserScan", LaserScan),
        "odom": ("nav_msgs/msg/Odometry", Odometry),
        "imu": ("sensor_msgs/msg/Imu", Imu),
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
        return robot_id in self._sessions

    def _qos_sensor(self) -> QoSProfile:
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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

        resolved: List[tuple[str, str, Type]] = []
        for short in req.topics:
            s = short.strip()
            if s.startswith("/"):
                s = s.lstrip("/")
            if s not in self._TYPE_MAP:
                resp.success = False
                resp.message = f"Unknown topic short name: {short!r}"
                resp.error_code = "NO_VALID_TOPICS"
                return resp
            tname = self._topic_name(rid, s)
            type_str, _cls = self._TYPE_MAP[s]
            resolved.append((tname, type_str, _cls))

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
        for tname, type_str, msg_cls in resolved:
            meta = TopicMetadata(topic_id, tname, type_str, "cdr", [])
            topic_id += 1
            writer.create_topic(meta)

            def make_cb(full_name: str, cls: Type):
                def _write(msg) -> None:
                    if sess.writer is None:
                        return
                    try:
                        data = serialize_message(msg)
                        stamp_ns = self.get_clock().now().nanoseconds
                        if hasattr(msg, "header") and msg.header.stamp.sec != 0:
                            stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
                        sess.writer.write(full_name, data, stamp_ns)
                        sess.bytes_written += len(data)
                    except Exception as ex:
                        self.get_logger().warn(f"write {full_name}: {ex}")

                return _write

            sub = self.create_subscription(
                msg_cls,
                tname,
                make_cb(tname, msg_cls),
                self._qos_sensor(),
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
