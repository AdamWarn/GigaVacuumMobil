#!/usr/bin/env python3
"""Pre-flight monitor that verifies mapping topics before exploration begins."""

from __future__ import annotations

from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


# Attempt to import whichever Cartographer trajectory message is available.
DEFAULT_TRAJECTORY_TOPIC: Optional[str]
try:  # Newer installations sometimes omit TrajectoryNodeList.
    from cartographer_ros_msgs.msg import TrajectoryNodeList as _TrajectoryMsg  # type: ignore

    TRAJECTORY_MSG = _TrajectoryMsg
    DEFAULT_TRAJECTORY_TOPIC = '/trajectory_node_list'
except ImportError:  # pragma: no cover - depends on distro packaging
    try:
        from cartographer_ros_msgs.msg import SubmapList as _TrajectoryMsg  # type: ignore

        TRAJECTORY_MSG = _TrajectoryMsg
        DEFAULT_TRAJECTORY_TOPIC = '/submap_list'
    except ImportError:  # pragma: no cover - depends on distro packaging
        try:
            from cartographer_ros_msgs.msg import TrajectoryStates as _TrajectoryMsg  # type: ignore

            TRAJECTORY_MSG = _TrajectoryMsg
            DEFAULT_TRAJECTORY_TOPIC = '/trajectory_states'
        except ImportError:  # pragma: no cover - depends on distro packaging
            TRAJECTORY_MSG = None
            DEFAULT_TRAJECTORY_TOPIC = None


class RateTracker:
    """Keeps a short history of event timestamps to estimate frequency."""

    def __init__(self, window_sec: float) -> None:
        self._window = max(0.1, float(window_sec))
        self._samples: Deque[float] = deque()

    def tick(self, now_sec: float) -> None:
        self._samples.append(now_sec)
        while self._samples and now_sec - self._samples[0] > self._window:
            self._samples.popleft()

    def rate_hz(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        duration = self._samples[-1] - self._samples[0]
        if duration <= 0.0:
            return 0.0
        return (len(self._samples) - 1) / duration


class MappingHealthMonitor(Node):
    """Watches core mapping topics and publishes a latched readiness flag."""

    def __init__(self) -> None:
        super().__init__('mapping_health_monitor')

        self.declare_parameter('scan_topic', '/scan_synced')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('stable_map_topic', '/stable_map')
        self.declare_parameter('tf_topic', '/tf')
        traj_topic_default = DEFAULT_TRAJECTORY_TOPIC or ''
        self.declare_parameter('trajectory_topic', traj_topic_default)
        self.declare_parameter('scan_rate_window', 5.0)
        self.declare_parameter('scan_rate_threshold', 8.0)
        self.declare_parameter('odom_max_age', 0.2)
        self.declare_parameter('map_timeout', 1.5)
        self.declare_parameter('tf_timeout', 1.0)
        self.declare_parameter('trajectory_timeout', 1.5)
        self.declare_parameter('timer_period', 0.5)
        self.declare_parameter('ready_topic', '/mapping_health/ready')
        self.declare_parameter('status_topic', '/mapping_health/status')

        self._scan_topic = self.get_parameter('scan_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._map_topic = self.get_parameter('map_topic').value
        self._stable_map_topic = self.get_parameter('stable_map_topic').value
        self._tf_topic = self.get_parameter('tf_topic').value
        self._trajectory_topic = str(self.get_parameter('trajectory_topic').value or '')
        self._scan_rate_threshold = float(self.get_parameter('scan_rate_threshold').value)
        self._odom_max_age = float(self.get_parameter('odom_max_age').value)
        self._map_timeout = float(self.get_parameter('map_timeout').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout').value)
        self._trajectory_timeout = float(self.get_parameter('trajectory_timeout').value)
        self._ready_topic = self.get_parameter('ready_topic').value
        self._status_topic = self.get_parameter('status_topic').value

        window = float(self.get_parameter('scan_rate_window').value)
        timer_period = float(self.get_parameter('timer_period').value)

        self._scan_rate = RateTracker(window)
        self._last_odom_stamp = None
        self._last_map_stamp = None
        self._last_stable_map_stamp = None
        self._tf_pairs: Dict[Tuple[str, str], float] = {}
        self._trajectory_enabled = bool(TRAJECTORY_MSG and self._trajectory_topic)
        self._last_trajectory_stamp = None
        self._last_ready = False

        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(Bool, self._ready_topic, ready_qos)
        self._status_pub = self.create_publisher(String, self._status_topic, ready_qos)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        map_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(LaserScan, self._scan_topic, self._scan_cb, sensor_qos)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, map_qos)
        self.create_subscription(OccupancyGrid, self._stable_map_topic, self._stable_map_cb, map_qos)
        self.create_subscription(TFMessage, self._tf_topic, self._tf_cb, sensor_qos)
        if self._trajectory_enabled:
            self.create_subscription(TRAJECTORY_MSG, self._trajectory_topic, self._trajectory_cb, 10)
        else:
            self.get_logger().warn(
                'No Cartographer trajectory topic detected; readiness will ignore trajectory activity.',
            )

        self._timer = self.create_timer(timer_period, self._on_timer)
        self.get_logger().info('Mapping health monitor online.')

    # ------------------------------------------------------------------
    def _scan_cb(self, _msg: LaserScan) -> None:
        self._scan_rate.tick(self._now_sec())

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom_stamp = self._time_to_float(msg.header.stamp)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._last_map_stamp = self._time_to_float(msg.header.stamp)

    def _stable_map_cb(self, msg: OccupancyGrid) -> None:
        self._last_stable_map_stamp = self._time_to_float(msg.header.stamp)

    def _tf_cb(self, msg: TFMessage) -> None:
        now = self._now_sec()
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self._tf_pairs[(parent, child)] = now

    def _trajectory_cb(self, msg: Any) -> None:
        self._last_trajectory_stamp = self._time_to_float(msg.header.stamp)

    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        now = self._now_sec()
        reasons = []

        scan_ok = self._scan_rate.rate_hz() >= self._scan_rate_threshold
        if not scan_ok:
            reasons.append('scan rate low')

        odom_ok = self._stamp_is_recent(self._last_odom_stamp, self._odom_max_age, now)
        if not odom_ok:
            reasons.append('odom stale')

        map_ok = self._stamp_is_recent(self._last_map_stamp, self._map_timeout, now)
        if not map_ok:
            reasons.append('map missing')

        stable_ok = self._stamp_is_recent(self._last_stable_map_stamp, self._map_timeout, now)
        if not stable_ok:
            reasons.append('stable_map missing')

        tf_ok = (
            self._stamp_is_recent(self._tf_pairs.get(('map', 'odom')), self._tf_timeout, now)
            and self._stamp_is_recent(self._tf_pairs.get(('odom', 'base_link')), self._tf_timeout, now)
        )
        if not tf_ok:
            reasons.append('tf chain incomplete')

        traj_ok = True
        if self._trajectory_enabled:
            traj_ok = self._stamp_is_recent(self._last_trajectory_stamp, self._trajectory_timeout, now)
            if not traj_ok:
                reasons.append('cartographer idle')

        ready = scan_ok and odom_ok and map_ok and stable_ok and tf_ok and traj_ok
        if ready != self._last_ready:
            self._publish_ready(ready, reasons)
            self._last_ready = ready

    # ------------------------------------------------------------------
    def _publish_ready(self, ready: bool, reasons) -> None:
        msg = Bool()
        msg.data = ready
        self._ready_pub.publish(msg)

        status = String()
        if ready:
            status.data = 'mapping stack healthy'
        else:
            status.data = 'not ready: ' + ', '.join(reasons) if reasons else 'not ready'
        self._status_pub.publish(status)

        log = status.data
        if ready:
            self.get_logger().info(log)
        else:
            self.get_logger().warn(log)

    # ------------------------------------------------------------------
    def _stamp_is_recent(self, stamp_sec, limit, now) -> bool:
        if stamp_sec is None:
            return False
        return (now - stamp_sec) <= limit

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    @staticmethod
    def _time_to_float(stamp: Time) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
