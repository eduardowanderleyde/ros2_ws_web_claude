#!/usr/bin/env python3
"""
Converte /cmd_vel (TwistStamped, publicado pelo Nav2 TB4) →
/cmd_vel_unstamped (Twist, esperado pelo bridge Gazebo Harmonic).

O nav2_minimal_tb4_sim usa bridge ROS→GZ com Twist (sem stamp),
mas o Nav2 com config TB4 (enable_stamped_cmd_vel=true) publica TwistStamped.
Este nó resolve o mismatch de tipos.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__(
            'tb4_cmd_vel_relay',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel_stamped', self._cb, qos)
        self.get_logger().info('cmd_vel relay: /cmd_vel_stamped → /cmd_vel (Twist)')

    def _cb(self, msg: TwistStamped):
        t = Twist()
        t.linear  = msg.twist.linear
        t.angular = msg.twist.angular
        self._pub.publish(t)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
