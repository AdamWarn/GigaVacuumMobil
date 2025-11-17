#!/usr/bin/env python3
"""
odom_frame_fixer.py - Fix empty frame_ids in odometry messages

This node subscribes to odometry with empty frame_ids and republishes
with correct frame_ids set.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomFrameFixer(Node):
    def __init__(self):
        super().__init__('odom_frame_fixer')
        
        # Subscribe to odometry with empty frames
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        
        # Publish fixed odometry
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.get_logger().info('Odometry frame fixer started')
        self.get_logger().info('Subscribing: /diff_drive_controller/odom')
        self.get_logger().info('Publishing: /odom (with frame_ids fixed)')

    def odom_callback(self, msg):
        """Fix the frame_ids and republish"""
        # Set the correct frame_ids
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Republish
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
