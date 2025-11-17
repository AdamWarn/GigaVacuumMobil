#!/usr/bin/env python3
"""
coverage_planner.py - Generate systematic coverage paths for cleaning

This node generates:
1. Boundary laps around the perimeter
2. Boustrophedon (lawn-mower) pattern for area coverage
3. Multi-area coverage with transitions between rooms

Based on occupancy grid map.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Trigger
import numpy as np
from scipy.ndimage import binary_erosion, binary_dilation
import cv2


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        
        # Parameters
        self.declare_parameter('robot_width', 0.24)  # Robot width in meters
        self.declare_parameter('path_spacing', 0.20)  # Distance between parallel paths
        self.declare_parameter('boundary_laps', 2)  # Number of boundary laps
        self.declare_parameter('coverage_angle', 0.0)  # Coverage pattern angle (radians)
        
        self.robot_width = self.get_parameter('robot_width').value
        self.path_spacing = self.get_parameter('path_spacing').value
        self.boundary_laps = self.get_parameter('boundary_laps').value
        self.coverage_angle = self.get_parameter('coverage_angle').value
        
        # Map data
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        
        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publishers
        self.coverage_path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.boundary_path_pub = self.create_publisher(Path, '/boundary_path', 10)
        
        # Services
        self.plan_coverage_srv = self.create_service(
            Trigger,
            'plan_coverage',
            self.plan_coverage_callback
        )
        
        self.get_logger().info('Coverage planner initialized')

    def map_callback(self, msg):
        """Store map data"""
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        # Convert map to numpy array
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        self.get_logger().info(f'Received map: {self.map_width}x{self.map_height}, resolution={self.map_resolution}')

    def plan_coverage_callback(self, request, response):
        """Service to generate coverage path"""
        if self.map_data is None:
            response.success = False
            response.message = 'No map available'
            return response
        
        try:
            # Generate boundary path
            boundary_path = self.generate_boundary_path()
            if boundary_path:
                self.boundary_path_pub.publish(boundary_path)
                self.get_logger().info(f'Published boundary path with {len(boundary_path.poses)} waypoints')
            
            # Generate coverage path
            coverage_path = self.generate_coverage_path()
            if coverage_path:
                self.coverage_path_pub.publish(coverage_path)
                self.get_logger().info(f'Published coverage path with {len(coverage_path.poses)} waypoints')
            
            response.success = True
            response.message = f'Generated paths: boundary={len(boundary_path.poses) if boundary_path else 0}, coverage={len(coverage_path.poses) if coverage_path else 0}'
        except Exception as e:
            response.success = False
            response.message = f'Error generating path: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def generate_boundary_path(self):
        """Generate boundary laps around the free space perimeter"""
        # Create binary map (free=1, occupied/unknown=0)
        free_space = (self.map_data == 0).astype(np.uint8)
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for lap in range(self.boundary_laps):
            # Erode to move inward for each lap
            erosion_size = int((lap * self.robot_width) / self.map_resolution)
            if erosion_size > 0:
                kernel = np.ones((erosion_size, erosion_size), np.uint8)
                eroded = cv2.erode(free_space, kernel, iterations=1)
            else:
                eroded = free_space
            
            # Find contours
            contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                break
            
            # Use largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Convert contour points to world coordinates
            for point in largest_contour:
                x_grid, y_grid = point[0][0], point[0][1]
                x_world, y_world = self.grid_to_world(x_grid, y_grid)
                
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x_world
                pose.pose.position.y = y_world
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                
                path.poses.append(pose)
        
        return path

    def generate_coverage_path(self):
        """Generate boustrophedon (lawn-mower) coverage pattern"""
        # Create binary map
        free_space = (self.map_data == 0).astype(np.uint8)
        
        # Erode to account for robot size
        robot_cells = int(self.robot_width / (2 * self.map_resolution))
        if robot_cells > 0:
            kernel = np.ones((robot_cells, robot_cells), np.uint8)
            free_space = cv2.erode(free_space, kernel, iterations=1)
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Calculate number of rows based on path spacing
        spacing_cells = int(self.path_spacing / self.map_resolution)
        
        # Generate parallel sweeps
        direction = 1  # Start going right
        for row in range(0, self.map_height, spacing_cells):
            # Find free cells in this row
            row_data = free_space[row, :]
            free_indices = np.where(row_data > 0)[0]
            
            if len(free_indices) == 0:
                continue
            
            # Get start and end of free space
            if direction == 1:
                # Left to right
                points = free_indices
            else:
                # Right to left
                points = free_indices[::-1]
            
            # Add points to path
            for x_grid in [points[0], points[-1]]:
                x_world, y_world = self.grid_to_world(x_grid, row)
                
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x_world
                pose.pose.position.y = y_world
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                
                path.poses.append(pose)
            
            # Alternate direction
            direction *= -1
        
        return path

    def grid_to_world(self, x_grid, y_grid):
        """Convert grid coordinates to world coordinates"""
        x_world = self.map_origin.position.x + (x_grid * self.map_resolution)
        y_world = self.map_origin.position.y + (y_grid * self.map_resolution)
        return x_world, y_world


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
