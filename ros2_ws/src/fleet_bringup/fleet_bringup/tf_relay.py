#!/usr/bin/env python3
"""Relay /robot/tf e /robot/tf_static para o /tf e /tf_static globais.

Uso:
  ros2 run fleet_bringup tf_relay --ros-args -p robot_id:=tb1
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robot_id', '')
        robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # /tf usa BEST_EFFORT (padrão ROS 2 TF e ros_gz_bridge)
        tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # /tf_static usa RELIABLE + TRANSIENT_LOCAL (latched)
        tf_static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pub_tf = self.create_publisher(TFMessage, '/tf', tf_qos)
        self._pub_tf_static = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)

        self.create_subscription(TFMessage, f'/{robot_id}/tf', self._cb_tf, tf_qos)
        self.create_subscription(TFMessage, f'/{robot_id}/tf_static', self._cb_tf_static, tf_static_qos)

        self.get_logger().info(f'TF relay: /{robot_id}/tf → /tf')

    def _cb_tf(self, msg):
        self._pub_tf.publish(msg)

    def _cb_tf_static(self, msg):
        self._pub_tf_static.publish(msg)


def main():
    rclpy.init()
    node = TFRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
