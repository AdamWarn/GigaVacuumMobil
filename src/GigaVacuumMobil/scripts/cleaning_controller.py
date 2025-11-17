#!/usr/bin/env python3
"""
cleaning_controller.py - Coordinate systematic cleaning missions

This node:
1. Executes boundary laps
2. Performs area coverage with boustrophedon pattern
3. Handles multi-room cleaning
4. Monitors cleaning progress
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import time


class CleaningController(Node):
    def __init__(self):
        super().__init__('cleaning_controller')
        
        # Parameters
        self.declare_parameter('start_with_boundary', True)
        self.declare_parameter('auto_start', False)
        
        self.start_with_boundary = self.get_parameter('start_with_boundary').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # State
        self.boundary_path = None
        self.coverage_path = None
        self.cleaning_active = False
        self.current_stage = 'idle'  # idle, boundary, coverage, complete
        
        # Subscribe to paths
        self.boundary_sub = self.create_subscription(
            Path,
            '/boundary_path',
            self.boundary_path_callback,
            10
        )
        
        self.coverage_sub = self.create_subscription(
            Path,
            '/coverage_path',
            self.coverage_path_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Service clients
        self.plan_coverage_client = self.create_client(Trigger, 'plan_coverage')
        
        # Services
        self.start_cleaning_srv = self.create_service(
            Trigger,
            'start_cleaning',
            self.start_cleaning_callback
        )
        
        self.stop_cleaning_srv = self.create_service(
            Trigger,
            'stop_cleaning',
            self.stop_cleaning_callback
        )
        
        self.get_logger().info('Cleaning controller initialized')
        
        # Auto-plan coverage if requested
        if self.auto_start:
            self.create_timer(2.0, self.auto_start_callback)

    def auto_start_callback(self):
        """Auto-start cleaning after delay"""
        self.get_logger().info('Auto-starting coverage planning...')
        self.request_coverage_plan()
        self.destroy_timer(self.auto_start_timer)

    def boundary_path_callback(self, msg):
        """Store boundary path"""
        self.boundary_path = msg
        self.get_logger().info(f'Received boundary path with {len(msg.poses)} waypoints')

    def coverage_path_callback(self, msg):
        """Store coverage path"""
        self.coverage_path = msg
        self.get_logger().info(f'Received coverage path with {len(msg.poses)} waypoints')

    def request_coverage_plan(self):
        """Request coverage planning"""
        if not self.plan_coverage_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Coverage planner service not available')
            return False
        
        request = Trigger.Request()
        future = self.plan_coverage_client.call_async(request)
        future.add_done_callback(self.coverage_plan_response_callback)
        return True

    def coverage_plan_response_callback(self, future):
        """Handle coverage planning response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Coverage planning successful: {response.message}')
            else:
                self.get_logger().error(f'Coverage planning failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def start_cleaning_callback(self, request, response):
        """Start cleaning mission"""
        if self.cleaning_active:
            response.success = False
            response.message = 'Cleaning already in progress'
            return response
        
        if self.boundary_path is None and self.coverage_path is None:
            # Request coverage planning first
            self.get_logger().info('No paths available, requesting coverage plan...')
            if not self.request_coverage_plan():
                response.success = False
                response.message = 'Failed to request coverage planning'
                return response
            
            # Wait a bit for paths to be generated
            time.sleep(2.0)
        
        if self.boundary_path is None and self.coverage_path is None:
            response.success = False
            response.message = 'No cleaning paths available'
            return response
        
        self.cleaning_active = True
        
        # Start with boundary or coverage based on parameter
        if self.start_with_boundary and self.boundary_path is not None:
            self.execute_boundary_cleaning()
        elif self.coverage_path is not None:
            self.execute_coverage_cleaning()
        else:
            response.success = False
            response.message = 'No valid cleaning path to execute'
            self.cleaning_active = False
            return response
        
        response.success = True
        response.message = 'Cleaning mission started'
        return response

    def stop_cleaning_callback(self, request, response):
        """Stop cleaning mission"""
        if not self.cleaning_active:
            response.success = False
            response.message = 'No cleaning in progress'
            return response
        
        # Cancel current navigation
        self.nav_client.cancel_goal()
        
        self.cleaning_active = False
        self.current_stage = 'idle'
        
        response.success = True
        response.message = 'Cleaning mission stopped'
        return response

    def execute_boundary_cleaning(self):
        """Execute boundary laps"""
        if self.boundary_path is None or len(self.boundary_path.poses) == 0:
            self.get_logger().warn('No boundary path to execute')
            self.execute_coverage_cleaning()
            return
        
        self.current_stage = 'boundary'
        self.get_logger().info('Starting boundary cleaning...')
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.boundary_path.poses
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.boundary_goal_response_callback)

    def boundary_goal_response_callback(self, future):
        """Handle boundary navigation response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Boundary navigation goal rejected')
            self.cleaning_active = False
            return
        
        self.get_logger().info('Boundary navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.boundary_result_callback)

    def boundary_result_callback(self, future):
        """Handle boundary navigation completion"""
        result = future.result().result
        self.get_logger().info('Boundary cleaning completed')
        
        # Start coverage cleaning
        if self.cleaning_active:
            self.execute_coverage_cleaning()

    def execute_coverage_cleaning(self):
        """Execute area coverage"""
        if self.coverage_path is None or len(self.coverage_path.poses) == 0:
            self.get_logger().warn('No coverage path to execute')
            self.cleaning_complete()
            return
        
        self.current_stage = 'coverage'
        self.get_logger().info('Starting coverage cleaning...')
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.coverage_path.poses
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.coverage_goal_response_callback)

    def coverage_goal_response_callback(self, future):
        """Handle coverage navigation response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Coverage navigation goal rejected')
            self.cleaning_active = False
            return
        
        self.get_logger().info('Coverage navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.coverage_result_callback)

    def coverage_result_callback(self, future):
        """Handle coverage navigation completion"""
        result = future.result().result
        self.get_logger().info('Coverage cleaning completed')
        self.cleaning_complete()

    def cleaning_complete(self):
        """Handle cleaning completion"""
        self.cleaning_active = False
        self.current_stage = 'complete'
        self.get_logger().info('✅ Cleaning mission complete!')


def main(args=None):
    rclpy.init(args=args)
    node = CleaningController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
