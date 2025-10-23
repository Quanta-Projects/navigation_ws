#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class StaticDockPose(Node):
    def __init__(self):
        super().__init__('static_dock_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, '/dock_pose', 10)
        self.timer = self.create_timer(0.5, self.tick)

        # Set this to your dock pose (in the frame you configured)
        self.frame = 'map'
        self.x, self.y, self.yaw = 2.35, -1.10, 1.57

    def tick(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = 0.0

        # yaw -> quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticDockPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
