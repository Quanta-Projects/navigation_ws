#!/usr/bin/env python3
"""
Check if KF Fusion evaluation topics are ready.
"""

import rclpy
from rclpy.node import Node
import sys
import time


class TopicChecker(Node):
    def __init__(self):
        super().__init__('kf_topic_checker')
        self.required_topics = [
            '/fused_humans_kf/poses',
            '/actors/ground_truth/actor1/odom',
            '/actors/ground_truth/actor2/odom',
        ]
        
    def check_topics(self):
        """Check if required topics are available."""
        self.get_logger().info('Checking for required KF evaluation topics...')
        
        available_topics = [name for name, _ in self.get_topic_names_and_types()]
        
        missing = []
        for topic in self.required_topics:
            if topic in available_topics:
                self.get_logger().info(f'  ✓ {topic}')
            else:
                self.get_logger().warn(f'  ✗ {topic} (missing)')
                missing.append(topic)
        
        if missing:
            self.get_logger().error(f'\n{len(missing)} required topic(s) missing!')
            self.get_logger().info('\nMake sure the following are running:')
            self.get_logger().info('  1. Gazebo simulation with actors')
            self.get_logger().info('  2. KF fusion node: ros2 launch smrr_crowdnav human_fusion_kf.launch.py')
            return False
        else:
            self.get_logger().info('\n✓ All required topics are available!')
            self.get_logger().info('Ready to run: ros2 run smrr_crowdnav evaluate_kf_only')
            return True


def main(args=None):
    rclpy.init(args=args)
    checker = TopicChecker()
    
    # Wait a moment for topic discovery
    time.sleep(1.0)
    
    success = checker.check_topics()
    
    checker.destroy_node()
    rclpy.shutdown()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
