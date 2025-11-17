#!/usr/bin/env python3
"""
cmd_vel_relay.py - Relay cmd_vel commands to diff_drive_controller

This node subscribes to /cmd_vel and republishes to /diff_drive_controller/cmd_vel
to work with the controller's expected topic name and message type.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to standard cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to controller's expected topic with TwistStamped
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )
        
        self.get_logger().info('cmd_vel relay node started')
        self.get_logger().info('Relaying /cmd_vel (Twist) -> /diff_drive_controller/cmd_vel (TwistStamped)')

    def cmd_vel_callback(self, msg):
        """Relay the cmd_vel message to the controller with timestamp"""
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
