import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import yaml
import argparse
import os
import sys
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

class WaypointGenerator(Node):
    """
    Generates waypoints for a systematic cleaning pattern from a map file.
    """
    def __init__(self, map_file, row_spacing, inflation_size, boundary_laps, output_file, orientation_mode, safety_margin):
        super().__init__('waypoint_generator')
        self.map_file = map_file
        self.row_spacing_meters = row_spacing
        self.inflation_pixels = inflation_size
        self.boundary_laps = boundary_laps
        self.output_file = output_file
        self.orientation_mode = orientation_mode  # 'identity' or 'heading'
        self.safety_margin = safety_margin

        self.path_pub = self.create_publisher(Path, 'coverage_path', 10)
        self.get_logger().info('Waypoint Generator started.')

    def generate_waypoints(self):
        """
        Main function to load map, process it, and generate waypoints.
        """
        self.get_logger().info(f"Loading map from {self.map_file}")
        
        try:
            with open(self.map_file, 'r') as f:
                map_metadata = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not read map metadata file: {e}")
            return

        self.resolution = map_metadata['resolution']
        self.origin = map_metadata['origin']
        
        map_image_path = os.path.join(os.path.dirname(self.map_file), map_metadata['image'])
        
        try:
            self.map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
            if self.map_image is None:
                raise FileNotFoundError(f"Map image not found at {map_image_path}")
        except Exception as e:
            self.get_logger().error(f"Could not load map image: {e}")
            return

        self.get_logger().info("Processing map to find cleanable area...")
        free_space = self.process_map_image(self.map_image, map_metadata)

        if free_space is None:
            self.get_logger().error("No free space found on the map.")
            return

        self.get_logger().info("Generating waypoints...")
        waypoints_pixel = self.create_coverage_path(free_space)

        if not waypoints_pixel:
            self.get_logger().warn("No waypoints were generated.")
            return

        waypoints_world = [self.pixel_to_world(p[0], p[1]) for p in waypoints_pixel]

        self.save_waypoints_to_yaml(waypoints_world)
        self.publish_path(waypoints_world)
        
        self.get_logger().info(f"Generated {len(waypoints_world)} waypoints.")
        self.get_logger().info(f"Waypoints saved to {self.output_file}")

    def process_map_image(self, image, metadata):
        """
        Converts map image to a binary representation of free space.
        """
        # Convert pixel values (0-255) into occupancy probabilities as nav2 does
        # prob = (255 - pixel) / 255  -> 1 = occupied, 0 = free
        occ_prob = (255.0 - image.astype(np.float32)) / 255.0
        free_thresh = metadata.get('free_thresh', 0.25)
        occupied_thresh = metadata.get('occupied_thresh', 0.65)

        free_mask = (occ_prob < free_thresh).astype(np.uint8) * 255

        # Treat unknown/occupied as zero
        kernel = np.ones((max(1, self.inflation_pixels), max(1, self.inflation_pixels)), np.uint8)
        eroded_map = cv2.erode(free_mask, kernel, iterations=1)

        # Apply optional extra safety margin (meters -> pixels)
        if self.safety_margin > 0.0:
            extra_iters = max(1, int(round(self.safety_margin / self.resolution)))
            eroded_map = cv2.erode(eroded_map, np.ones((3, 3), np.uint8), iterations=extra_iters)

        # Find the largest connected component
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded_map, 8, cv2.CV_32S)
        if num_labels <= 1:
            return None
        
        # Find the label of the largest component (ignoring background label 0)
        largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        
        # Create a mask for the largest component
        largest_comp = np.zeros(eroded_map.shape, dtype=np.uint8)
        largest_comp[labels == largest_label] = 255

        return largest_comp

    def create_coverage_path(self, free_space):
        """
        Generates a boustrophedon path and boundary laps.
        """
        contours, _ = cv2.findContours(free_space, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        main_contour = max(contours, key=cv2.contourArea)
        
        path = []
        
        # 1. Boundary Laps
        if self.boundary_laps > 0:
            for i in range(self.boundary_laps):
                # Erode more for each lap
                erosion_pixels = int(self.row_spacing_meters / self.resolution) * i
                if erosion_pixels > 0:
                    kernel = np.ones((3,3), np.uint8)
                    eroded_lap = cv2.erode(free_space, kernel, iterations=erosion_pixels)
                else:
                    eroded_lap = free_space.copy()

                lap_contours, _ = cv2.findContours(eroded_lap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if lap_contours:
                    lap_contour = max(lap_contours, key=cv2.contourArea)
                    epsilon = 0.01 * cv2.arcLength(lap_contour, True)
                    approx = cv2.approxPolyDP(lap_contour, epsilon, True)
                    
                    lap_path = approx.reshape(-1, 2).tolist()
                    if i % 2 != 0: # Reverse every other lap
                        lap_path.reverse()
                    path.extend(lap_path)

        # 2. Boustrophedon (back-and-forth) pattern
        x, y, w, h = cv2.boundingRect(main_contour)
        row_spacing_pixels = int(self.row_spacing_meters / self.resolution)
        
        y_coords = range(y + row_spacing_pixels // 2, y + h, row_spacing_pixels)
        
        for i, y_scan in enumerate(y_coords):
            scanline_points = []
            for x_scan in range(x, x + w):
                if free_space[y_scan, x_scan] == 255:
                    scanline_points.append([x_scan, y_scan])
            
            if not scanline_points:
                continue

            if len(scanline_points) > 1:
                # Cluster points to handle separate rooms on the same horizontal line
                clustering = DBSCAN(eps=5, min_samples=1).fit(scanline_points)
                labels = clustering.labels_
                
                unique_labels = sorted(list(set(labels)))
                
                scan_segments = []
                for k in unique_labels:
                    if k == -1: continue
                    
                    segment_points = [p for j, p in enumerate(scanline_points) if labels[j] == k]
                    
                    start_point = min(segment_points, key=lambda p: p[0])
                    end_point = max(segment_points, key=lambda p: p[0])
                    scan_segments.append((start_point, end_point))

                # Sort segments from left to right
                scan_segments.sort(key=lambda s: s[0][0])

                if i % 2 != 0: # On odd rows, reverse the order of segments
                    scan_segments.reverse()

                for start, end in scan_segments:
                    path.append(start)
                    path.append(end)

        return path

    def pixel_to_world(self, px, py):
        """Converts a pixel index (col,row) to the map frame.

        We treat each pixel as a square whose centre should be on the path.
        Nav2 maps use the pixel's bottom-left corner as origin -> add 0.5 cell
        so waypoints land inside free cells instead of on/outside walls.
        """
        map_height = self.map_image.shape[0]
        world_x = self.origin[0] + ((px + 0.5) * self.resolution)
        world_y = self.origin[1] + ((map_height - py - 0.5) * self.resolution)
        return [world_x, world_y]

    def save_waypoints_to_yaml(self, waypoints):
        """Write waypoints in exact block style like testwp.yaml.

        Format:
        waypoints:
          waypoint0:
            pose:
              - x
              - y
              - 0
            orientation:
              - qx
              - qy
              - qz
              - qw
        """
        lines = ["waypoints:"]
        for i, wp in enumerate(waypoints):
            # Orientation selection
            if self.orientation_mode == 'identity':
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:  # heading
                if i + 1 < len(waypoints):
                    nx, ny = waypoints[i+1][0], waypoints[i+1][1]
                    yaw = np.arctan2(ny - wp[1], nx - wp[0])
                elif i > 0:
                    px, py = waypoints[i-1][0], waypoints[i-1][1]
                    yaw = np.arctan2(wp[1] - py, wp[0] - px)
                else:
                    yaw = 0.0
                q = R.from_euler('z', yaw).as_quat()  # x,y,z,w
                qx, qy, qz, qw = map(float, q)

            name = f"waypoint{i}"
            lines.append(f"  {name}:")
            lines.append("    pose:")
            lines.append(f"      - {float(wp[0])}")
            lines.append(f"      - {float(wp[1])}")
            lines.append("      - 0")
            lines.append("    orientation:")
            lines.append(f"      - {qx}")
            lines.append(f"      - {qy}")
            lines.append(f"      - {qz}")
            lines.append(f"      - {qw}")
        content = "\n".join(lines) + "\n"
        with open(self.output_file, 'w') as f:
            f.write(content)
        self.get_logger().info(f"Waypoints written (format=testwp style) to {self.output_file}")

    def publish_path(self, waypoints):
        """
        Publishes the generated path for visualization in RViz.
        """
        if not self.path_pub.get_subscription_count() > 0:
            self.get_logger().info("No subscribers on /coverage_path, skipping publish.")
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            path_msg.poses.append(pose)
        
        self.get_logger().info("Publishing path for RViz...")
        # Publish a few times to ensure it's received
        for _ in range(3):
            self.path_pub.publish(path_msg)
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info("Path published.")


def main(args=None):
    rclpy.init(args=args)
    
    # Use a dedicated parser for our script's arguments
    parser = argparse.ArgumentParser(description='Generate waypoints for cleaning.')
    parser.add_argument('--map-file', required=True, help='Path to the map YAML file.')
    parser.add_argument('--row-spacing', type=float, default=0.5, help='Spacing between cleaning rows in meters.')
    parser.add_argument('--inflation-size', type=int, default=5, help='Number of pixels to inflate obstacles by.')
    parser.add_argument('--boundary-laps', type=int, default=1, help='Number of laps to do around the boundary.')
    parser.add_argument('--output-file', default='waypoints.yaml', help='Output file for waypoints.')
    parser.add_argument('--orientation-mode', choices=['identity','heading'], default='heading', help='Orientation calculation: identity keeps (0,0,0,1); heading points toward next waypoint.')
    parser.add_argument('--safety-margin', type=float, default=0.0, help='Additional inward shrink in meters to keep waypoints away from walls.')
    
    # Filter out ROS-specific arguments before parsing
    filtered_args = rclpy.utilities.remove_ros_args(args=sys.argv[1:])
    parsed_args = parser.parse_args(args=filtered_args)

    node = WaypointGenerator(
        map_file=parsed_args.map_file,
        row_spacing=parsed_args.row_spacing,
        inflation_size=parsed_args.inflation_size,
        boundary_laps=parsed_args.boundary_laps,
        output_file=parsed_args.output_file,
        orientation_mode=parsed_args.orientation_mode,
        safety_margin=parsed_args.safety_margin
    )
    # Generate immediately
    node.generate_waypoints()
    # Give any late subscribers a brief chance (optional)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
