#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        
        # Parameters
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('path_topic', '/odom_path')
        self.declare_parameter('max_path_length', 10000)
        
        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.max_path_length = self.get_parameter('max_path_length').value
        
        # Subscriber and Publisher
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        # Path message
        self.path = Path()
        
        self.get_logger().info(f'Odom to Path converter started')
        self.get_logger().info(f'Subscribing to: {odom_topic}')
        self.get_logger().info(f'Publishing to: {path_topic}')
    
    def odom_callback(self, msg):
        # Create PoseStamped from Odometry
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        # Add to path
        self.path.header = msg.header
        self.path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)
        
        # Publish path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
