import argparse
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

from .coverage_planner_core import CoveragePlannerCore, PlannerSettings

class WaypointGenerator(Node):
    """
    Generates waypoints for a systematic cleaning pattern from a map file.
    """
    def __init__(self, map_file, row_spacing, inflation_size, boundary_laps, output_file, orientation_mode, safety_margin):
        super().__init__('waypoint_generator')
        self.output_file = output_file
        self.planner = CoveragePlannerCore(
            PlannerSettings(
                map_file=map_file,
                row_spacing=row_spacing,
                inflation_size=inflation_size,
                boundary_laps=boundary_laps,
                orientation_mode=orientation_mode,
                safety_margin=safety_margin,
            )
        )

        self.path_pub = self.create_publisher(Path, 'coverage_path', 10)
        self.get_logger().info('Waypoint Generator started.')

    def generate_waypoints(self):
        """
        Main function to load map, process it, and generate waypoints.
        """
        try:
            waypoints_world = self.planner.generate_waypoints_world()
        except Exception as exc:  # pragma: no cover - logged for CLI debugging
            self.get_logger().error(f"Failed to generate coverage path: {exc}")
            return

        self.planner.write_yaml(waypoints_world, self.output_file)
        self.publish_path(waypoints_world)
        
        self.get_logger().info(f"Generated {len(waypoints_world)} waypoints.")
        self.get_logger().info(f"Waypoints saved to {self.output_file}")

    def publish_path(self, waypoints):
        """
        Publishes the generated path for visualization in RViz.
        """
        if not self.path_pub.get_subscription_count() > 0:
            self.get_logger().info("No subscribers on /coverage_path, skipping publish.")
            return

        now = self.get_clock().now().to_msg()
        path_msg = self.planner.build_path_message(waypoints, frame_id='map', stamp=now)
        
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
