#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class ScanOdomRepublisher(Node):
    def __init__(self):
        super().__init__('scan_odom_republisher')
        self.sub = self.create_subscription(Odometry, '/scan_odom', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

    def callback(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

