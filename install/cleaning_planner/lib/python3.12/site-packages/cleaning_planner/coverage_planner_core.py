"""Core coverage-planning utilities shared by CLI and ROS nodes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple
import os
import yaml
import numpy as np
import cv2
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


@dataclass
class PlannerSettings:
    map_file: str
    row_spacing: float = 0.5
    inflation_size: int = 5
    boundary_laps: int = 1
    orientation_mode: str = 'heading'  # 'identity' | 'heading'
    safety_margin: float = 0.0
    min_passage_width: float = 0.35


class CoveragePlannerCore:
    """Pure-Python planner that turns an occupancy map into a coverage path."""

    def __init__(self, settings: PlannerSettings):
        self.settings = settings
        self._map_metadata = None
        self._map_image = None
        self._free_space = None

    @property
    def resolution(self) -> float:
        return self._map_metadata['resolution']

    @property
    def origin(self) -> Sequence[float]:
        return self._map_metadata['origin']

    def load_map(self) -> None:
        if not os.path.exists(self.settings.map_file):
            raise FileNotFoundError(f"Map YAML not found: {self.settings.map_file}")

        with open(self.settings.map_file, 'r', encoding='utf-8') as f:
            self._map_metadata = yaml.safe_load(f)

        image_path = os.path.join(
            os.path.dirname(self.settings.map_file),
            self._map_metadata['image']
        )
        self._map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if self._map_image is None:
            raise FileNotFoundError(f"Map image not found: {image_path}")
        self._free_space = None

    def has_map(self) -> bool:
        return self._map_metadata is not None and self._map_image is not None

    def set_map_data(self, metadata: Dict, image: np.ndarray) -> None:
        self._map_metadata = dict(metadata)
        self._map_image = image.copy()
        self._free_space = None

    def generate_waypoints_world(self) -> List[Tuple[float, float]]:
        if self._map_metadata is None or self._map_image is None:
            self.load_map()

        self._free_space = self._process_map_image()
        if self._free_space is None:
            raise RuntimeError('Could not find free space in map.')

        pixel_path = self._create_coverage_path(self._free_space)
        if not pixel_path:
            raise RuntimeError('Planner produced an empty path.')

        return [self._pixel_to_world(px, py) for px, py in pixel_path]

    def build_path_message(self, waypoints: Sequence[Tuple[float, float]], frame_id: str, stamp=None) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        if stamp is not None:
            path_msg.header.stamp = stamp

        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp if stamp is not None else path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = self._orientation_from_sequence(waypoints, i)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)
        return path_msg

    def write_yaml(self, waypoints: Sequence[Tuple[float, float]], output_file: str) -> None:
        lines = ["waypoints:"]
        for i, (x, y) in enumerate(waypoints):
            qx, qy, qz, qw = self._orientation_from_sequence(waypoints, i)
            lines.append(f"  waypoint{i}:")
            lines.append("    pose:")
            lines.append(f"      - {float(x)}")
            lines.append(f"      - {float(y)}")
            lines.append("      - 0")
            lines.append("    orientation:")
            lines.append(f"      - {qx}")
            lines.append(f"      - {qy}")
            lines.append(f"      - {qz}")
            lines.append(f"      - {qw}")
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("\n".join(lines) + "\n")

    # Internal helpers -------------------------------------------------

    def _process_map_image(self):
        image = self._map_image
        metadata = self._map_metadata
        occ_prob = (255.0 - image.astype(np.float32)) / 255.0
        free_thresh = metadata.get('free_thresh', 0.25)
        free_mask = (occ_prob < free_thresh).astype(np.uint8) * 255

        kernel = np.ones((max(1, self.settings.inflation_size), max(1, self.settings.inflation_size)), np.uint8)
        eroded = cv2.erode(free_mask, kernel, iterations=1)

        if self.settings.safety_margin > 0.0:
            extra_iters = max(1, int(round(self.settings.safety_margin / self.resolution)))
            eroded = cv2.erode(eroded, np.ones((3, 3), np.uint8), iterations=extra_iters)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded, 8, cv2.CV_32S)
        if num_labels <= 1:
            return None

        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        largest_comp = np.zeros(eroded.shape, dtype=np.uint8)
        largest_comp[labels == largest_label] = 255

        min_passage_px = int(round(max(0.0, self.settings.min_passage_width) / self.resolution))
        if min_passage_px > 1:
            kernel = np.ones((min_passage_px, min_passage_px), np.uint8)
            filtered = cv2.morphologyEx(largest_comp, cv2.MORPH_OPEN, kernel)
            if np.count_nonzero(filtered):
                largest_comp = filtered
        return largest_comp

    def _create_coverage_path(self, free_space) -> List[Tuple[int, int]]:
        contours, _ = cv2.findContours(free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        main_contour = max(contours, key=cv2.contourArea)
        path = []

        if self.settings.boundary_laps > 0:
            for lap_idx in range(self.settings.boundary_laps):
                erosion_pixels = int(self.settings.row_spacing / self.resolution) * lap_idx
                if erosion_pixels > 0:
                    kernel = np.ones((3, 3), np.uint8)
                    eroded = cv2.erode(free_space, kernel, iterations=erosion_pixels)
                else:
                    eroded = free_space.copy()

                lap_contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if lap_contours:
                    lap = max(lap_contours, key=cv2.contourArea)
                    epsilon = 0.01 * cv2.arcLength(lap, True)
                    approx = cv2.approxPolyDP(lap, epsilon, True)
                    lap_path = approx.reshape(-1, 2).tolist()
                    if lap_idx % 2 != 0:
                        lap_path.reverse()
                    path.extend(lap_path)

        x, y, w, h = cv2.boundingRect(main_contour)
        row_spacing_pixels = max(1, int(self.settings.row_spacing / self.resolution))
        y_coords = range(y + row_spacing_pixels // 2, y + h, row_spacing_pixels)

        for idx, y_scan in enumerate(y_coords):
            scanline_points = []
            for x_scan in range(x, x + w):
                if free_space[y_scan, x_scan] == 255:
                    scanline_points.append([x_scan, y_scan])

            if not scanline_points:
                continue

            if len(scanline_points) > 1:
                clustering = DBSCAN(eps=5, min_samples=1).fit(scanline_points)
                labels = clustering.labels_
                segments = []
                for label in sorted(set(labels)):
                    segment_points = [pt for label_idx, pt in enumerate(scanline_points) if labels[label_idx] == label]
                    start_point = min(segment_points, key=lambda p: p[0])
                    end_point = max(segment_points, key=lambda p: p[0])
                    segments.append((start_point, end_point))
                segments.sort(key=lambda seg: seg[0][0])
                if idx % 2 != 0:
                    segments.reverse()
                for start, end in segments:
                    path.append(start)
                    path.append(end)
        return path

    def _pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        map_height = self._map_image.shape[0]
        world_x = self.origin[0] + ((px + 0.5) * self.resolution)
        world_y = self.origin[1] + ((map_height - py - 0.5) * self.resolution)
        return (float(world_x), float(world_y))

    def _orientation_from_sequence(self, waypoints: Sequence[Tuple[float, float]], index: int) -> Tuple[float, float, float, float]:
        if self.settings.orientation_mode == 'identity':
            return (0.0, 0.0, 0.0, 1.0)

        if index + 1 < len(waypoints):
            next_x, next_y = waypoints[index + 1]
            yaw = np.arctan2(next_y - waypoints[index][1], next_x - waypoints[index][0])
        elif index > 0:
            prev_x, prev_y = waypoints[index - 1]
            yaw = np.arctan2(waypoints[index][1] - prev_y, waypoints[index][0] - prev_x)
        else:
            yaw = 0.0
        qx, qy, qz, qw = R.from_euler('z', yaw).as_quat()
        return (float(qx), float(qy), float(qz), float(qw))
