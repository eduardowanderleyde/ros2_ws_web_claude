#!/usr/bin/env python3
"""Sensor data collector: enable/disable per robot_id, write to rosbag2. UI-ready API."""
from __future__ import annotations

import os
import threading
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple, Type

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import rosbag2_py

from fleet_msgs.srv import (
    EnableCollection,
    DisableCollection,
    CollectionStatus,
)


# Topic short name (relative to robot namespace) -> (Python msg type, rosbag2 type string)
def _subscription_qos(short_name: str):
    """QoS por tópico.

    Nav2 AMCL (Jazzy): ``create_publisher(..., QoS(KeepLast(1)).transient_local().reliable())``.
    Subscritor **volatile** pode não receber nada no Fast DDS; usar **transient_local**
    igual ao publisher. ``depth`` > 1 para não perder rajadas entre callbacks.
    """
    if short_name == "amcl_pose":
        return QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
    return 10


KNOWN_TOPICS: Dict[str, Tuple[Type[Any], str]] = {
    "scan": (LaserScan, "sensor_msgs/msg/LaserScan"),
    "odom": (Odometry, "nav_msgs/msg/Odometry"),
    "imu": (Imu, "sensor_msgs/msg/Imu"),
    "amcl_pose": (
        PoseWithCovarianceStamped,
        "geometry_msgs/msg/PoseWithCovarianceStamped",
    ),
}


@dataclass
class RobotCollectionState:
    enabled: bool = False
    writer: Optional[Any] = None  # rosbag2_py.SequentialWriter
    write_lock: threading.Lock = field(default_factory=threading.Lock)
    subscriptions: List[Any] = field(default_factory=list)
    current_bag_uri: str = ""
    topics: List[str] = field(default_factory=list)
    output_mode: str = "rosbag2"


def _bag_bytes(bag_uri: str) -> int:
    if not bag_uri or not os.path.isdir(bag_uri):
        return 0
    total = 0
    for _root, _dirs, files in os.walk(bag_uri):
        for f in files:
            total += os.path.getsize(os.path.join(_root, f))
    return total


class SensorCollector(Node):
    def __init__(self) -> None:
        super().__init__("sensor_collector")
        self._collection_cbg = ReentrantCallbackGroup()

        self.declare_parameter("robots", ["tb1", "tb2", "tb3"])
        self.declare_parameter("collections_dir", "collections")

        self.robots: List[str] = self.get_parameter("robots").value
        self.collections_dir: str = self.get_parameter("collections_dir").value

        self._state: Dict[str, RobotCollectionState] = {
            rid: RobotCollectionState() for rid in self.robots
        }

        self._enable_srv = self.create_service(
            EnableCollection, "enable_collection", self._handle_enable
        )
        self._disable_srv = self.create_service(
            DisableCollection, "disable_collection", self._handle_disable
        )
        self._status_srv = self.create_service(
            CollectionStatus, "collection_status", self._handle_status
        )

        self.get_logger().info(
            f"sensor_collector ready. Robots: {self.robots}. "
            "API: enable_collection, disable_collection, collection_status."
        )

    def _full_topic(self, robot_id: str, short_name: str) -> str:
        if short_name.startswith("/"):
            return short_name
        # Sim sem namespace: robot_id vazio -> /scan, /odom, etc.
        if not robot_id:
            return f"/{short_name}"
        return f"/{robot_id}/{short_name}"

    def _handle_enable(
        self, request: EnableCollection.Request, response: EnableCollection.Response
    ) -> EnableCollection.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}. Known: {self.robots}."
            response.error_code = "UNKNOWN_ROBOT"
            return response

        state = self._state[robot_id]
        if state.enabled:
            response.success = False
            response.message = f"Collection already enabled for {robot_id}. Disable first."
            response.error_code = "ALREADY_COLLECTING"
            return response

        topics = [t.strip() for t in request.topics if t.strip()]
        if not topics:
            topics = list(KNOWN_TOPICS.keys())

        output_mode = (request.output_mode or "rosbag2").strip().lower()
        if output_mode != "rosbag2":
            response.success = False
            response.message = f"Only output_mode 'rosbag2' is supported for now. Got: {output_mode}"
            response.error_code = "UNSUPPORTED_OUTPUT_MODE"
            return response

        key = robot_id if robot_id else "default"
        base = os.path.join(self.collections_dir, key)
        os.makedirs(base, exist_ok=True)
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_uri = os.path.join(base, stamp)

        if os.path.exists(bag_uri):
            response.success = False
            response.message = f"Bag path already exists: {bag_uri}"
            response.error_code = "BAG_PATH_EXISTS"
            return response

        try:
            writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id="mcap")
            converter_options = rosbag2_py.ConverterOptions("", "")
            self.get_logger().info(
                f"[{robot_id}] enable_collection: opening rosbag writer storage_id=mcap uri={bag_uri}"
            )
            writer.open(storage_options, converter_options)

            topic_id = 0
            topic_meta: List[Tuple[str, Type[Any], str]] = []
            for short in topics:
                if short not in KNOWN_TOPICS:
                    self.get_logger().warn(f"Unknown topic '{short}', skipping. Known: {list(KNOWN_TOPICS.keys())}")
                    continue
                msg_type, type_str = KNOWN_TOPICS[short]
                full_name = self._full_topic(robot_id, short)
                info = rosbag2_py.TopicMetadata(
                    id=topic_id,
                    name=full_name,
                    type=type_str,
                    serialization_format="cdr",
                )
                writer.create_topic(info)
                topic_meta.append((full_name, msg_type, type_str))
                topic_id += 1

            if not topic_meta:
                response.success = False
                response.message = "No valid topics to record."
                response.error_code = "NO_VALID_TOPICS"
                return response

            subs = []
            for (full_name, msg_type, _), short in zip(
                topic_meta,
                [t for t in topics if t in KNOWN_TOPICS],
            ):
                cb = self._make_write_cb(robot_id, full_name, writer)
                qos = _subscription_qos(short)
                sub = self.create_subscription(
                    msg_type,
                    full_name,
                    cb,
                    qos,
                    callback_group=self._collection_cbg,
                )
                subs.append(sub)

            state.enabled = True
            state.writer = writer
            state.subscriptions = subs
            state.current_bag_uri = bag_uri
            state.topics = [t[0] for t in topic_meta]
            state.output_mode = output_mode

            self.get_logger().info(
                f"[{robot_id}] Collection enabled. Bag: {bag_uri}, topics: {state.topics}"
            )
            response.success = True
            response.message = f"Collection enabled for {robot_id}. Bag: {bag_uri}"
            response.error_code = ""
        except Exception as e:
            self.get_logger().error(f"Enable collection failed: {e}")
            self.get_logger().error(traceback.format_exc())
            # Remove partial bag directory to keep subsequent runs clean.
            try:
                if os.path.isdir(bag_uri):
                    import shutil

                    shutil.rmtree(bag_uri)
            except Exception:
                pass
            response.success = False
            response.message = str(e)
            response.error_code = "ENABLE_FAILED"
        return response

    def _make_write_cb(self, robot_id: str, topic_name: str, writer):
        def cb(msg):
            state = self._state.get(robot_id)
            if not state or not state.enabled or state.writer is None:
                return
            try:
                with state.write_lock:
                    state.writer.write(
                        topic_name,
                        serialize_message(msg),
                        self.get_clock().now().nanoseconds,
                    )
            except Exception as ex:
                self.get_logger().error(f"Write failed [{topic_name}]: {ex}")

        return cb

    def _handle_disable(
        self, request: DisableCollection.Request, response: DisableCollection.Response
    ) -> DisableCollection.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.success = False
            response.message = f"Unknown robot_id: {robot_id}."
            response.error_code = "UNKNOWN_ROBOT"
            return response

        state = self._state[robot_id]
        if not state.enabled:
            response.success = True
            response.message = f"Collection was not enabled for {robot_id}."
            response.error_code = ""
            return response

        try:
            for sub in state.subscriptions:
                self.destroy_subscription(sub)
            state.subscriptions.clear()
            if state.writer is not None:
                state.writer.close()
                state.writer = None
            bag_uri = state.current_bag_uri
            state.enabled = False
            state.current_bag_uri = ""
            state.topics = []

            self.get_logger().info(f"[{robot_id}] Collection disabled. Bag saved: {bag_uri}")
            response.success = True
            response.message = f"Collection disabled for {robot_id}. Bag: {bag_uri}"
            response.error_code = ""
        except Exception as e:
            self.get_logger().error(f"Disable collection failed: {e}")
            response.success = False
            response.message = str(e)
            response.error_code = "DISABLE_FAILED"
        return response

    def _handle_status(
        self, request: CollectionStatus.Request, response: CollectionStatus.Response
    ) -> CollectionStatus.Response:
        robot_id = request.robot_id.strip()
        if robot_id not in self._state:
            response.is_collecting = False
            response.current_file = ""
            response.bytes_written = 0
            response.message = f"Unknown robot_id: {robot_id}."
            return response

        state = self._state[robot_id]
        response.is_collecting = state.enabled
        response.current_file = state.current_bag_uri
        response.bytes_written = _bag_bytes(state.current_bag_uri) if state.current_bag_uri else 0
        response.message = "OK"
        return response


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = SensorCollector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (KeyboardInterrupt).")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
