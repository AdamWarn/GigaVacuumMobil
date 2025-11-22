#!/usr/bin/env python3
"""Gate the coverage planner until the map is sealed and drive exploration goals meanwhile."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@dataclass
class MapMetrics:
    unknown_ratio: float
    edge_occupancy: Dict[str, float]

    @property
    def min_edge_fraction(self) -> float:
        return min(self.edge_occupancy.values()) if self.edge_occupancy else 0.0


class ExplorationSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('exploration_supervisor', automatically_declare_parameters_from_overrides=True)

        self._declare_if_needed('map_topic', '/stable_map')
        self._declare_if_needed('pose_topic', '/amcl_pose')
        self._declare_if_needed('navigate_action', 'navigate_to_pose')
        self._declare_if_needed('coverage_start_service', '/coverage_executor/start_coverage')
        self._declare_if_needed('coverage_enable_topic', '/coverage_enable')
        self._declare_if_needed('map_ready_topic', '/map_ready')
        self._declare_if_needed('edge_band_width', 0.3)
        self._declare_if_needed('min_edge_fraction', 0.7)
        self._declare_if_needed('max_unknown_ratio', 0.18)
        self._declare_if_needed('fallback_unknown_ratio', 0.3)
        self._declare_if_needed('frontier_clearance', 0.25)
        self._declare_if_needed('frontier_retry_limit', 5)
        self._declare_if_needed('goal_timeout', 120.0)
        self._declare_if_needed('analysis_period', 3.0)
        self._declare_if_needed('occupied_threshold', 0.6)
        self._declare_if_needed('free_threshold', 0.3)
        self._declare_if_needed('max_goal_distance', 10.0)
        self._declare_if_needed('behavior_tree', '')

        self._map_topic = self.get_parameter('map_topic').value
        self._pose_topic = self.get_parameter('pose_topic').value
        self._coverage_start_service = self.get_parameter('coverage_start_service').value
        self._coverage_enable_topic = self.get_parameter('coverage_enable_topic').value
        self._map_ready_topic = self.get_parameter('map_ready_topic').value
        self._edge_band_width = float(self.get_parameter('edge_band_width').value)
        self._min_edge_fraction = float(self.get_parameter('min_edge_fraction').value)
        self._max_unknown_ratio = float(self.get_parameter('max_unknown_ratio').value)
        self._fallback_unknown_ratio = float(self.get_parameter('fallback_unknown_ratio').value)
        self._frontier_clearance = float(self.get_parameter('frontier_clearance').value)
        self._frontier_retry_limit = max(1, int(self.get_parameter('frontier_retry_limit').value))
        self._goal_timeout = max(30.0, float(self.get_parameter('goal_timeout').value))
        self._analysis_period = max(1.0, float(self.get_parameter('analysis_period').value))
        self._occupied_value = int(round(float(self.get_parameter('occupied_threshold').value) * 100.0))
        self._free_value = int(round(float(self.get_parameter('free_threshold').value) * 100.0))
        self._max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self._behavior_tree = self.get_parameter('behavior_tree').value or ''

        qos_transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        self._map_sub = self.create_subscription(OccupancyGrid, self._map_topic, self._map_callback, qos_sensor)
        self._pose_sub = self.create_subscription(PoseWithCovarianceStamped, self._pose_topic, self._pose_callback, qos_sensor)
        self._map_ready_pub = self.create_publisher(Bool, self._map_ready_topic, qos_transient)
        self._coverage_enable_pub = None
        if self._coverage_enable_topic:
            self._coverage_enable_pub = self.create_publisher(Bool, self._coverage_enable_topic, qos_transient)

        self._nav_client = ActionClient(self, NavigateToPose, self.get_parameter('navigate_action').value)
        self._coverage_client = self.create_client(Trigger, self._coverage_start_service)

        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_pose: Optional[PoseWithCovarianceStamped] = None
        self._latest_metrics: Optional[MapMetrics] = None
        self._state = 'initializing'
        self._current_goal_handle = None
        self._goal_deadline_ns: Optional[int] = None
        self._frontier_failures = 0
        self._coverage_requested = False
        self._last_ready_flag = False

        self._analysis_timer = self.create_timer(self._analysis_period, self._on_timer)
        self.get_logger().info('Exploration supervisor ready: monitoring map completeness.')

    def _declare_if_needed(self, name: str, value) -> None:
        if not self.has_parameter(name):
            self.declare_parameter(name, value)

    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._latest_metrics = self._analyze_map(msg)

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = msg

    def _on_timer(self) -> None:
        if self._latest_map is None or self._latest_pose is None:
            return

        metrics = self._latest_metrics
        if metrics is None:
            return

        ready = self._map_is_ready(metrics)
        if ready and not self._last_ready_flag:
            self.get_logger().info('Map sealed (unknown_ratio=%.3f, min_edge=%.2f). Triggering cleaning.', metrics.unknown_ratio, metrics.min_edge_fraction)
        self._last_ready_flag = ready

        if ready:
            self._publish_map_ready(True)
            self._transition_to_cleaning()
            return

        self._publish_map_ready(False)
        self._state = 'exploring'
        if not self._nav_client.server_is_ready():
            self._nav_client.wait_for_server(timeout_sec=0.0)
            return

        if self._current_goal_handle is None:
            goal = self._select_frontier_goal(self._latest_map)
            if goal is None:
                self._frontier_failures += 1
                if self._frontier_failures >= self._frontier_retry_limit and metrics.unknown_ratio <= self._fallback_unknown_ratio:
                    self.get_logger().warn('No more safe frontiers but map mostly closed; accepting current map.')
                    self._last_ready_flag = True
                else:
                    self.get_logger().debug('No frontier goal available yet (%d/%d).', self._frontier_failures, self._frontier_retry_limit)
                return
            self._frontier_failures = 0
            self._dispatch_goal(goal)
        else:
            self._check_goal_timeout()

    # ------------------------------------------------------------------
    def _map_is_ready(self, metrics: MapMetrics) -> bool:
        edge_ok = all(frac >= self._min_edge_fraction for frac in metrics.edge_occupancy.values())
        unknown_ok = metrics.unknown_ratio <= self._max_unknown_ratio
        if edge_ok and unknown_ok:
            return True
        if self._frontier_failures >= self._frontier_retry_limit and metrics.unknown_ratio <= self._fallback_unknown_ratio:
            return True
        return False

    def _publish_map_ready(self, ready: bool) -> None:
        msg = Bool()
        msg.data = ready
        self._map_ready_pub.publish(msg)
        if self._coverage_enable_pub is not None and ready:
            self._coverage_enable_pub.publish(msg)

    def _transition_to_cleaning(self) -> None:
        if self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.get_logger().info('Canceled outstanding exploration goal.'))
            self._current_goal_handle = None
        self._goal_deadline_ns = None
        self._state = 'cleaning'
        if self._coverage_requested:
            return
        self._coverage_requested = True
        if not self._coverage_client.service_is_ready():
            self._coverage_client.wait_for_service(timeout_sec=0.0)
        request = Trigger.Request()
        future = self._coverage_client.call_async(request)
        future.add_done_callback(self._handle_coverage_response)

    def _handle_coverage_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime diagnostics
            self.get_logger().error(f'Coverage start service failed: {exc}')
            self._coverage_requested = False
            return
        if response.success:
            self.get_logger().info('Coverage executor acknowledged start request.')
        else:
            self.get_logger().warn(f'Coverage executor rejected start request: {response.message}')
            self._coverage_requested = False

    # ------------------------------------------------------------------
    def _select_frontier_goal(self, msg: OccupancyGrid) -> Optional[PoseStamped]:
        grid = np.asarray(msg.data, dtype=np.int16)
        width = msg.info.width
        height = msg.info.height
        if grid.size != width * height:
            self.get_logger().warn('Occupancy grid size mismatch; skipping frontier search.')
            return None
        grid = grid.reshape((height, width))
        free_mask = np.logical_and(grid >= 0, grid <= self._free_value)
        unknown_mask = grid < 0
        if not np.any(unknown_mask):
            return None

        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_dilated = cv2.dilate(unknown_mask.astype(np.uint8), kernel, iterations=1)
        frontier_mask = np.logical_and(free_mask, unknown_dilated.astype(bool))
        if not np.any(frontier_mask):
            return None

        dist_pixels = cv2.distanceTransform(free_mask.astype(np.uint8), cv2.DIST_L2, 3)
        dist_meters = dist_pixels * msg.info.resolution
        clearance_mask = dist_meters >= self._frontier_clearance
        candidate_mask = np.logical_and(frontier_mask, clearance_mask)
        if not np.any(candidate_mask):
            candidate_mask = frontier_mask
        coords = np.argwhere(candidate_mask)
        if coords.size == 0:
            return None

        robot_xy = None
        if self._latest_pose is not None:
            robot_xy = (
                self._latest_pose.pose.pose.position.x,
                self._latest_pose.pose.pose.position.y,
            )

        world_points: Dict[int, Tuple[float, float]] = {}
        distances: Dict[int, float] = {}
        for idx, (row, col) in enumerate(coords):
            wx, wy = self._cell_to_world(msg, row, col)
            world_points[idx] = (wx, wy)
            if robot_xy is not None:
                distances[idx] = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            else:
                distances[idx] = 0.0

        if not distances:
            return None

        best_idx = max(distances, key=distances.get)
        if self._max_goal_distance > 0.0 and distances[best_idx] > self._max_goal_distance:
            self.get_logger().debug('Skipping frontier %.2fm away (over max distance).', distances[best_idx])
            return None

        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id or 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = world_points[best_idx]
        goal.pose.position.z = 0.0
        yaw = 0.0
        if robot_xy is not None:
            yaw = math.atan2(goal.pose.position.y - robot_xy[1], goal.pose.position.x - robot_xy[0])
        goal.pose.orientation = self._yaw_to_quaternion(yaw)
        return goal

    def _dispatch_goal(self, goal: PoseStamped) -> None:
        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = goal
        if self._behavior_tree:
            navigate_goal.behavior_tree = self._behavior_tree
        send_future = self._nav_client.send_goal_async(navigate_goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)
        deadline_time = self.get_clock().now() + Duration(seconds=self._goal_timeout)
        self._goal_deadline_ns = int(deadline_time.nanoseconds)
        self.get_logger().info('Exploration goal dispatched at (%.2f, %.2f).', goal.pose.position.x, goal.pose.position.y)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected exploration goal.')
            self._current_goal_handle = None
            self._goal_deadline_ns = None
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        self._current_goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Exploration goal reached.')
        else:
            self.get_logger().warn(f'Exploration goal finished with status={result.status}.')
        self._goal_deadline_ns = None

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'Exploration feedback: distance_remaining=%.2f recovery_count=%d',
            getattr(feedback, 'distance_remaining', float('nan')),
            getattr(feedback, 'number_of_recoveries', -1),
        )

    def _check_goal_timeout(self) -> None:
        if self._current_goal_handle is None or self._goal_deadline_ns is None:
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns > self._goal_deadline_ns:
            self.get_logger().warn('Exploration goal exceeded timeout; canceling.')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.get_logger().info('Exploration goal canceled after timeout.'))
            self._current_goal_handle = None
            self._goal_deadline_ns = None

    # ------------------------------------------------------------------
    def _analyze_map(self, msg: OccupancyGrid) -> Optional[MapMetrics]:
        grid = np.asarray(msg.data, dtype=np.int16)
        width = msg.info.width
        height = msg.info.height
        if grid.size != width * height:
            return None
        grid = grid.reshape((height, width))
        occupied_mask = grid >= self._occupied_value
        band_px = max(1, int(round(self._edge_band_width / msg.info.resolution)))
        edge_slices = {
            'left': occupied_mask[:, :band_px],
            'right': occupied_mask[:, width - band_px:],
            'top': occupied_mask[:band_px, :],
            'bottom': occupied_mask[height - band_px:, :],
        }
        edge_occupancy = {}
        for name, data in edge_slices.items():
            if data.size == 0:
                edge_occupancy[name] = 0.0
            else:
                edge_occupancy[name] = float(np.mean(data))
        unknown_ratio = float(np.mean(grid < 0))
        return MapMetrics(unknown_ratio=unknown_ratio, edge_occupancy=edge_occupancy)

    @staticmethod
    def _cell_to_world(msg: OccupancyGrid, row: int, col: int) -> Tuple[float, float]:
        res = msg.info.resolution
        origin = msg.info.origin.position
        x = origin.x + (col + 0.5) * res
        y = origin.y + (row + 0.5) * res
        return (x, y)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        half = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual stop
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
