"""ROS 2 node that exposes the coverage planner as a service + latched topic."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from nav_msgs.msg import Path, OccupancyGrid

from .coverage_planner_core import CoveragePlannerCore, PlannerSettings


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner', automatically_declare_parameters_from_overrides=True)

        self._declare_if_needed('map_file', '')
        self._declare_if_needed('row_spacing', 0.5)
        self._declare_if_needed('safety_margin', 0.15)
        self._declare_if_needed('inflation_size', 5)
        self._declare_if_needed('boundary_laps', 1)
        self._declare_if_needed('orientation_mode', 'heading')
        self._declare_if_needed('plan_frame', 'map')
        self._declare_if_needed('min_passage_width', 0.35)
        self._declare_if_needed('map_topic', '')
        self._declare_if_needed('auto_replan_on_map_update', True)
        self._declare_if_needed('map_update_min_interval', 5.0)

        self._planner: Optional[CoveragePlannerCore] = None
        self._latest_plan: Optional[Path] = None
        self._latest_map_metadata = None
        self._latest_map_image = None
        self._map_sub = None
        self._map_topic_name = ''
        self._auto_replan_on_map_update = True
        self._map_update_min_interval = 5.0
        self._last_topic_replan_time = None

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_pub = self.create_publisher(Path, 'coverage_plan', qos)
        self._replan_srv = self.create_service(Trigger, 'replan_coverage', self._handle_replan)
        self.add_on_set_parameters_callback(self._on_params_changed)

        self._refresh_runtime_config()
        self._configure_map_subscription()

        self._refresh_planner()
        self._generate_and_publish_plan()

    def _declare_if_needed(self, name, value):
        if not self.has_parameter(name):
            self.declare_parameter(name, value)

    def _refresh_runtime_config(self):
        self._map_topic_name = self.get_parameter('map_topic').value or ''
        self._auto_replan_on_map_update = bool(self.get_parameter('auto_replan_on_map_update').value)
        self._map_update_min_interval = float(self.get_parameter('map_update_min_interval').value)

    def _configure_map_subscription(self):
        if self._map_topic_name:
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            if self._map_sub is None or self._map_sub.topic_name != self._map_topic_name:
                if self._map_sub is not None:
                    self.destroy_subscription(self._map_sub)
                self._map_sub = self.create_subscription(
                    OccupancyGrid,
                    self._map_topic_name,
                    self._map_topic_callback,
                    qos,
                )
                self.get_logger().info(f'Subscribed to live map updates on {self._map_topic_name}.')
        elif self._map_sub is not None:
            self.destroy_subscription(self._map_sub)
            self._map_sub = None

    def _map_topic_callback(self, msg: OccupancyGrid):
        metadata = {
            'resolution': msg.info.resolution,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
            'free_thresh': 0.25,
        }
        image = self._convert_occupancy_to_image(msg)
        self._latest_map_metadata = metadata
        self._latest_map_image = image

        if self._planner is not None:
            self._planner.set_map_data(metadata, image)

        if not self._auto_replan_on_map_update:
            return

        now = self.get_clock().now()
        if self._last_topic_replan_time is not None:
            elapsed = now - self._last_topic_replan_time
            if elapsed.nanoseconds < int(self._map_update_min_interval * 1e9):
                return

        self._last_topic_replan_time = now
        self.get_logger().info('Map update received; regenerating coverage plan.')
        self._generate_and_publish_plan()

    @staticmethod
    def _convert_occupancy_to_image(msg: OccupancyGrid) -> np.ndarray:
        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != msg.info.height * msg.info.width:
            raise ValueError('OccupancyGrid data size mismatch')
        grid = data.reshape((msg.info.height, msg.info.width))
        grid = np.flipud(grid)
        clamped = np.clip(grid, 0, 100)
        image = (255 - (clamped * 255 // 100)).astype(np.uint8)
        image[grid < 0] = 205
        return image

    # ------------------------------------------------------------------
    def _refresh_planner(self):
        settings = PlannerSettings(
            map_file=self.get_parameter('map_file').value,
            row_spacing=float(self.get_parameter('row_spacing').value),
            safety_margin=float(self.get_parameter('safety_margin').value),
            inflation_size=int(self.get_parameter('inflation_size').value),
            boundary_laps=int(self.get_parameter('boundary_laps').value),
            orientation_mode=self.get_parameter('orientation_mode').value,
            min_passage_width=float(self.get_parameter('min_passage_width').value),
        )

        if self._planner is None:
            self._planner = CoveragePlannerCore(settings)
        else:
            self._planner.settings = settings

        if self._latest_map_metadata is not None and self._latest_map_image is not None:
            self._planner.set_map_data(self._latest_map_metadata, self._latest_map_image)
        elif not settings.map_file:
            self.get_logger().warn('Waiting for live map data before planning (map_file unset).')

    def _generate_and_publish_plan(self):
        if self._planner is None:
            self.get_logger().warn('Planner not configured yet.')
            return

        if not self._planner.settings.map_file and not self._planner.has_map():
            self.get_logger().warn('Live map data has not arrived yet; coverage plan deferred.')
            return

        try:
            waypoints = self._planner.generate_waypoints_world()
        except Exception as exc:  # pragma: no cover - runtime diagnostics
            self.get_logger().error(f'Failed to build coverage plan: {exc}')
            return

        frame = self.get_parameter('plan_frame').value
        path_msg = self._planner.build_path_message(
            waypoints,
            frame_id=frame,
            stamp=self.get_clock().now().to_msg()
        )
        self._latest_plan = path_msg
        self._plan_pub.publish(path_msg)
        self.get_logger().info(f'Published coverage plan with {len(path_msg.poses)} poses.')

    def _handle_replan(self, _request, response):
        self._generate_and_publish_plan()
        response.success = self._latest_plan is not None
        response.message = 'Plan regenerated.' if response.success else 'Planner failed to produce a plan.'
        return response

    def _on_params_changed(self, params):
        relevant = {'map_file', 'row_spacing', 'safety_margin', 'inflation_size', 'boundary_laps', 'orientation_mode'}
        runtime = {'map_topic', 'auto_replan_on_map_update', 'map_update_min_interval'}
        if any(p.name in relevant for p in params):
            self._refresh_planner()
            self._generate_and_publish_plan()
        if any(p.name in runtime for p in params):
            self._refresh_runtime_config()
            self._configure_map_subscription()
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
