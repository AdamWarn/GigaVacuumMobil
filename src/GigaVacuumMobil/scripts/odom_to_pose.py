#!/usr/bin/env python3
"""Bridge nav_msgs/Odometry into PoseWithCovarianceStamped for exploration."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class OdomToPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_pose_bridge')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_pose')

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(PoseWithCovarianceStamped, self._output_topic, qos)
        self._subscription = self.create_subscription(
            Odometry,
            self._input_topic,
            self._odom_callback,
            qos,
        )

        self.get_logger().info(
            'Odom-to-pose bridge running (input=%s output=%s)' % (self._input_topic, self._output_topic)
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose
        self._publisher.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
