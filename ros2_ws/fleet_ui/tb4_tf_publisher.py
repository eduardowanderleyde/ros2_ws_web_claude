#!/usr/bin/env python3
"""
Publica TF odom→base_link a partir do /sim_ground_truth_pose do TB4.
Publica directamente no /tf sem depender do relógio de simulação.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class TB4TFPublisher(Node):
    def __init__(self):
        super().__init__('tb4_tf_publisher')
        # Publica directamente em /tf (sem TransformBroadcaster)
        self._tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self._cb, qos)
        self.get_logger().info('TB4 TF publisher pronto')

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp   # usa o stamp da mensagem directamente
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        tf_msg = TFMessage(transforms=[t])
        self._tf_pub.publish(tf_msg)


def main():
    rclpy.init()
    node = TB4TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
