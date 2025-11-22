#!/usr/bin/env python3
"""Re-stamp LaserScan messages so TF consumers see monotonic timestamps."""

from __future__ import annotations

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RosTime
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan


class ScanTimestampSync(Node):
    """Ensures LaserScan timestamps stay aligned with the current ROS clock."""

    def __init__(self) -> None:
        super().__init__('scan_timestamp_sync')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_synced')
        self.declare_parameter('max_skew_sec', 5.0)
        self.declare_parameter('min_increment_ns', 1000)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._max_skew = float(self.get_parameter('max_skew_sec').value)
        self._min_increment_ns = max(1, int(self.get_parameter('min_increment_ns').value))

        self._publisher = self.create_publisher(LaserScan, self._output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            LaserScan,
            self._input_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self._fixed_msgs = 0
        self._last_log = self.get_clock().now()
        self._status_period = Duration(seconds=10.0)
        self._last_pub_ns: int | None = None

        self.get_logger().info(
            'Scan timestamp sync running (input=%s output=%s max_skew=%.2fs)'
            % (self._input_topic, self._output_topic, self._max_skew)
        )

    # ------------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan) -> None:
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        skew = self._time_difference(now_msg, msg.header.stamp)

        target_ns = self._stamp_to_ns(msg.header.stamp)

        if skew < -self._max_skew or skew > self._max_skew:
            target_ns = now_time.nanoseconds
            self._fixed_msgs += 1
            if now_time - self._last_log >= self._status_period:
                self.get_logger().warn(
                    'LaserScan skew %.2fs exceeded %.2fs; rewrote %d messages.'
                    % (skew, self._max_skew, self._fixed_msgs)
                )
                self._last_log = now_time
        else:
            target_ns = max(target_ns, now_time.nanoseconds)
        if self._last_pub_ns is not None and target_ns <= self._last_pub_ns:
            target_ns = self._last_pub_ns + self._min_increment_ns

        msg.header.stamp = RosTime(nanoseconds=target_ns).to_msg()
        self._last_pub_ns = target_ns
        self._publisher.publish(msg)

    @staticmethod
    def _time_difference(a: Time, b: Time) -> float:
        return float(a.sec - b.sec) + float(a.nanosec - b.nanosec) / 1e9

    @staticmethod
    def _stamp_to_ns(stamp: Time) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanTimestampSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
