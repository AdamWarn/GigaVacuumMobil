"""Node that listens to live occupancy grids and periodically saves them to disk."""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import yaml


class MapUpdaterNode(Node):
    def __init__(self) -> None:
        super().__init__('map_updater_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('save_directory', str(Path.cwd() / 'saved_maps'))
        self.declare_parameter('base_map_name', 'map_snapshot')
        self.declare_parameter('symlink_name', 'current_map.yaml')
        self.declare_parameter('save_period', 60.0)
        self.declare_parameter('save_on_update', False)
        self.declare_parameter('keep_history', True)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('filtered_map_topic', '/stable_map')
        self.declare_parameter('publish_filtered_map', True)
        self.declare_parameter('wall_persistence_observations', 5)
        self.declare_parameter('obstacle_dilation_radius', 0.05)
        self.declare_parameter('workspace_bounds', '')
        self.declare_parameter('wall_gap_fill_enabled', True)
        self.declare_parameter('wall_gap_fill_max_width', 0.5)
        self.declare_parameter('free_decay_step', 1)

        self._map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self._save_dir = Path(self.get_parameter('save_directory').get_parameter_value().string_value).expanduser()
        self._base_name = self.get_parameter('base_map_name').get_parameter_value().string_value
        self._symlink_name = self.get_parameter('symlink_name').get_parameter_value().string_value
        self._save_period = float(self.get_parameter('save_period').value)
        self._save_on_update = bool(self.get_parameter('save_on_update').value)
        self._keep_history = bool(self.get_parameter('keep_history').value)
        self._occupied_thresh = float(self.get_parameter('occupied_thresh').value)
        self._free_thresh = float(self.get_parameter('free_thresh').value)
        self._filtered_map_topic = self.get_parameter('filtered_map_topic').get_parameter_value().string_value
        self._publish_filtered_map = bool(self.get_parameter('publish_filtered_map').value)
        self._wall_persistence = max(1, int(self.get_parameter('wall_persistence_observations').value))
        self._obstacle_dilation_radius = float(self.get_parameter('obstacle_dilation_radius').value)
        bounds_text = self.get_parameter('workspace_bounds').get_parameter_value().string_value
        self._workspace_bounds = self._parse_workspace_bounds(bounds_text)
        self._wall_gap_fill_enabled = bool(self.get_parameter('wall_gap_fill_enabled').value)
        self._wall_gap_fill_max_width = float(self.get_parameter('wall_gap_fill_max_width').value)
        self._free_decay_step = max(1, int(self.get_parameter('free_decay_step').value))

        self._latest_msg: Optional[OccupancyGrid] = None
        self._latest_filtered_grid: Optional[np.ndarray] = None
        self._latest_info = None
        self._last_save_time = None
        self._confidence_grid: Optional[np.ndarray] = None
        self._filtered_pub = None

        self._save_dir.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._sub = self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, qos)

        if self._publish_filtered_map and self._filtered_map_topic:
            self._filtered_pub = self.create_publisher(OccupancyGrid, self._filtered_map_topic, qos)

        if self._save_period > 0.0:
            self._timer = self.create_timer(self._save_period, self._save_latest_map)
        else:
            self._timer = None

        self.get_logger().info(
            f"Listening for maps on {self._map_topic} and saving to {self._save_dir} every {self._save_period}s"
        )

    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_msg = msg
        filtered_grid = self._process_map(msg)
        self._latest_filtered_grid = filtered_grid
        self._latest_info = msg.info

        if self._publish_filtered_map and self._filtered_pub is not None:
            filtered_msg = OccupancyGrid()
            filtered_msg.header = msg.header
            filtered_msg.info = msg.info
            filtered_msg.data = filtered_grid.astype(np.int8).flatten().tolist()
            self._filtered_pub.publish(filtered_msg)

        if self._save_on_update:
            self._save_latest_map()

    def _save_latest_map(self) -> None:
        if self._latest_msg is None or self._latest_filtered_grid is None:
            self.get_logger().debug('No filtered map data yet; skipping save.')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self._keep_history:
            stem = f"{self._base_name}_{timestamp}"
        else:
            stem = self._base_name

        yaml_path = self._save_dir / f"{stem}.yaml"
        pgm_path = self._save_dir / f"{stem}.pgm"

        pgm_bytes = self._occupancy_to_pgm(self._latest_filtered_grid, self._latest_info)
        with open(pgm_path, 'wb') as pgm_file:
            pgm_file.write(pgm_bytes)

        yaml_payload = self._build_yaml(self._latest_info, pgm_path.name)
        with open(yaml_path, 'w', encoding='utf-8') as yaml_file:
            yaml.safe_dump(yaml_payload, yaml_file, default_flow_style=False)

        if self._symlink_name:
            symlink_path = self._save_dir / self._symlink_name
            try:
                if symlink_path.is_symlink() or symlink_path.exists():
                    symlink_path.unlink()
                symlink_target = yaml_path.relative_to(symlink_path.parent)
            except ValueError:
                symlink_target = yaml_path
            symlink_path.symlink_to(symlink_target)

        self.get_logger().info(f'Saved updated map to {yaml_path}')

        if not self._keep_history and stem != self._base_name:
            # If history disabled we overwrite base file names only once.
            pass

    # ------------------------------------------------------------------
    def _occupancy_to_pgm(self, grid: np.ndarray, info) -> bytes:
        height, width = grid.shape
        flipped = np.flipud(grid)
        image = np.full_like(grid, 205, dtype=np.uint8)
        valid = flipped >= 0
        clipped = np.clip(flipped[valid], 0, 100)
        image[valid] = (100 - clipped) * 255 // 100
        header = f"P5\n{width} {height}\n255\n".encode('ascii')
        return header + image.astype(np.uint8).tobytes()

    def _build_yaml(self, info, image_name: str) -> dict:
        origin = info.origin
        yaw = self._quat_to_yaw(origin.orientation)
        return {
            'image': image_name,
            'mode': 'trinary',
            'resolution': float(info.resolution),
            'origin': [float(origin.position.x), float(origin.position.y), float(yaw)],
            'negate': 0,
            'occupied_thresh': self._occupied_thresh,
            'free_thresh': self._free_thresh,
        }

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    def _process_map(self, msg: OccupancyGrid) -> np.ndarray:
        width = msg.info.width
        height = msg.info.height
        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != width * height:
            raise ValueError('OccupancyGrid data does not match metadata size')
        if width == 0 or height == 0:
            self.get_logger().warn('Received occupancy grid with zero width/height; skipping processing.')
            return np.empty((0, 0), dtype=np.int16)
        grid = data.reshape((height, width))

        self._ensure_confidence_grid(grid.shape)

        occupied_value = int(round(self._occupied_thresh * 100))
        free_value = int(round(self._free_thresh * 100))

        occupied_mask = grid >= occupied_value
        free_mask = (grid >= 0) & (grid <= free_value)

        self._confidence_grid[occupied_mask] = self._wall_persistence

        decay_mask = np.logical_and(free_mask, self._confidence_grid > 0)
        if np.any(decay_mask):
            self._confidence_grid[decay_mask] = np.maximum(
                0,
                self._confidence_grid[decay_mask] - self._free_decay_step,
            )

        filtered = grid.copy()
        persistence_mask = self._confidence_grid > 0
        filtered[persistence_mask] = 100

        if self._obstacle_dilation_radius > 0.0:
            radius_px = max(1, int(round(self._obstacle_dilation_radius / msg.info.resolution)))
            if radius_px <= 0:
                return grid
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius_px * 2 + 1, radius_px * 2 + 1))
            occ_mask = (filtered >= occupied_value).astype(np.uint8)
            dilated = cv2.dilate(occ_mask, kernel, iterations=1)
            filtered[dilated > 0] = 100

        if self._wall_gap_fill_enabled and self._wall_gap_fill_max_width > 0.0:
            filtered = self._fill_wall_gaps(filtered, msg.info)

        if self._workspace_bounds is not None:
            filtered = self._apply_workspace_bounds(filtered, msg.info)

        return filtered

    def _ensure_confidence_grid(self, shape):
        if self._confidence_grid is None or self._confidence_grid.shape != shape:
            self._confidence_grid = np.zeros(shape, dtype=np.int16)

    def _parse_workspace_bounds(self, bounds_text: str) -> Optional[tuple]:
        if not bounds_text:
            return None
        parts = [p.strip() for p in bounds_text.split(',') if p.strip()]
        if len(parts) != 4:
            self.get_logger().warn('workspace_bounds must have four comma-separated values (min_x,min_y,max_x,max_y); ignoring.')
            return None
        try:
            min_x, min_y, max_x, max_y = map(float, parts)
        except ValueError:
            self.get_logger().warn('workspace_bounds values must be numeric; ignoring.')
            return None
        if min_x >= max_x or min_y >= max_y:
            self.get_logger().warn('workspace_bounds must satisfy min < max for both axes; ignoring.')
            return None
        self.get_logger().info(f'Clipping workspace to [{min_x}, {min_y}] -> [{max_x}, {max_y}] (meters).')
        return (min_x, min_y, max_x, max_y)

    def _apply_workspace_bounds(self, grid: np.ndarray, info) -> np.ndarray:
        min_x, min_y, max_x, max_y = self._workspace_bounds
        res = info.resolution
        width = info.width
        height = info.height
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        x_coords = origin_x + (np.arange(width, dtype=np.float32) + 0.5) * res
        y_coords = origin_y + (height - np.arange(height, dtype=np.float32) - 0.5) * res
        valid_x = (x_coords >= min_x) & (x_coords <= max_x)
        valid_y = (y_coords >= min_y) & (y_coords <= max_y)
        mask = np.outer(valid_y, valid_x)
        grid[~mask] = 100
        return grid

    def _fill_wall_gaps(self, grid: np.ndarray, info) -> np.ndarray:
        gap_px = max(1, int(round(self._wall_gap_fill_max_width / info.resolution)))
        occ_mask = (grid >= 90).astype(np.uint8)
        diag_kernel = np.eye(gap_px, dtype=np.uint8)
        anti_diag_kernel = np.flipud(diag_kernel)
        kernels = [
            cv2.getStructuringElement(cv2.MORPH_RECT, (gap_px, 1)),  # horizontal
            cv2.getStructuringElement(cv2.MORPH_RECT, (1, gap_px)),  # vertical
            diag_kernel,
            anti_diag_kernel,
        ]
        filled = occ_mask.copy()
        for kernel in kernels:
            closed = cv2.morphologyEx(filled, cv2.MORPH_CLOSE, kernel, iterations=1)
            filled = np.maximum(filled, closed)
        grid[filled > 0] = 100
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()