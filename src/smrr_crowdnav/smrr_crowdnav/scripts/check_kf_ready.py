#!/usr/bin/env python3
"""
Pre-Evaluation Readiness Checker for KF-Only Evaluation

Verifies that ground truth and Kalman Filter fusion are publishing.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import time
from collections import defaultdict


class KFReadinessChecker(Node):
    def __init__(self):
        super().__init__('kf_readiness_checker')
        
        # Track which topics have received messages
        self.topics_status = {
            'ground_truth': defaultdict(bool),  # actor1-10
            'kf_fusion': False,
        }
        
        # Ground truth subscribers (one per actor)
        self.gt_subscribers = []
        for i in range(1, 11):  # actors 1-10
            actor_name = f'actor{i}'
            sub = self.create_subscription(
                Odometry,
                f'/actors/ground_truth/{actor_name}/odom',
                lambda msg, name=actor_name: self.gt_callback(name),
                10
            )
            self.gt_subscribers.append(sub)
        
        # KF fusion subscriber
        self.kf_fusion_sub = self.create_subscription(
            PoseArray,
            '/fused_humans_kf/poses',
            self.kf_fusion_callback,
            10
        )
        
        # Timer to check status every 2 seconds
        self.check_timer = self.create_timer(2.0, self.check_status)
        self.start_time = time.time()
        self.max_wait_time = 30.0  # Wait up to 30 seconds
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('KF-Only Evaluation Readiness Checker')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Checking for required topics...')
        self.get_logger().info('')
    
    def gt_callback(self, actor_name):
        """Mark ground truth topic as active."""
        if not self.topics_status['ground_truth'][actor_name]:
            self.topics_status['ground_truth'][actor_name] = True
            self.get_logger().info(f'✓ Ground truth detected: {actor_name}')
    
    def kf_fusion_callback(self, msg):
        """Mark KF fusion as active."""
        if not self.topics_status['kf_fusion']:
            self.topics_status['kf_fusion'] = True
            self.get_logger().info(f'✓ Kalman Filter fusion detected: /fused_humans_kf/poses ({len(msg.poses)} humans)')
    
    def check_status(self):
        """Check if all required topics are publishing."""
        elapsed = time.time() - self.start_time
        
        # Check ground truth (at least 1 actor required)
        gt_actors = [name for name, status in self.topics_status['ground_truth'].items() if status]
        gt_ready = len(gt_actors) > 0
        
        # Check KF fusion
        kf_ready = self.topics_status['kf_fusion']
        
        # Summary
        self.get_logger().info('')
        self.get_logger().info('─' * 70)
        self.get_logger().info(f'Status Check (elapsed: {elapsed:.1f}s / {self.max_wait_time:.1f}s):')
        self.get_logger().info('')
        
        # Ground truth status
        if gt_ready:
            self.get_logger().info(f'  [✓] Ground Truth: {len(gt_actors)} actor(s) detected')
            for actor in sorted(gt_actors):
                self.get_logger().info(f'      • {actor}')
        else:
            self.get_logger().warn('  [✗] Ground Truth: No actors detected')
            self.get_logger().warn('      Expected topics: /actors/ground_truth/actor{1-10}/odom')
        
        # KF fusion status
        if kf_ready:
            self.get_logger().info('  [✓] Kalman Filter Fusion: /fused_humans_kf/poses')
        else:
            self.get_logger().warn('  [✗] Kalman Filter Fusion: /fused_humans_kf/poses not publishing')
        
        self.get_logger().info('─' * 70)
        self.get_logger().info('')
        
        # Check if all ready
        if gt_ready and kf_ready:
            self.get_logger().info('═' * 70)
            self.get_logger().info('✓✓✓ READY FOR KF-ONLY EVALUATION ✓✓✓')
            self.get_logger().info('═' * 70)
            self.get_logger().info('')
            self.get_logger().info('You can now run the KF-only evaluation:')
            self.get_logger().info('  ros2 run smrr_crowdnav evaluate_kf_only')
            self.get_logger().info('')
            self.get_logger().info('With custom parameters:')
            self.get_logger().info('  ros2 run smrr_crowdnav evaluate_kf_only --ros-args \\')
            self.get_logger().info('    -p test_duration:=120.0 \\')
            self.get_logger().info('    -p association_threshold:=1.5 \\')
            self.get_logger().info('    -p output_file:=/tmp/kf_results.txt')
            self.get_logger().info('')
            self.check_timer.cancel()
            rclpy.shutdown()
            return
        
        # Timeout check
        if elapsed >= self.max_wait_time:
            self.get_logger().error('')
            self.get_logger().error('═' * 70)
            self.get_logger().error('✗✗✗ TIMEOUT: Not all required topics are publishing ✗✗✗')
            self.get_logger().error('═' * 70)
            self.get_logger().error('')
            self.get_logger().error('TROUBLESHOOTING:')
            
            if not gt_ready:
                self.get_logger().error('')
                self.get_logger().error('1. Ground Truth Missing:')
                self.get_logger().error('   • Make sure Gazebo simulation is running')
                self.get_logger().error('   • Check if actors are spawned in the world')
                self.get_logger().error('   • Verify ground truth publisher is active:')
                self.get_logger().error('     ros2 topic list | grep ground_truth')
            
            if not kf_ready:
                self.get_logger().error('')
                self.get_logger().error('2. Kalman Filter Fusion Missing:')
                self.get_logger().error('   • Launch the KF fusion node:')
                self.get_logger().error('     ros2 launch smrr_crowdnav human_fusion_kf.launch.py use_sim_time:=true')
                self.get_logger().error('   • Or check if it\'s running:')
                self.get_logger().error('     ros2 topic info /fused_humans_kf/poses')
                self.get_logger().error('     ros2 topic echo /fused_humans_kf/poses --once')
            
            self.get_logger().error('')
            self.get_logger().error('General checks:')
            self.get_logger().error('  • Verify nodes are running: ros2 node list')
            self.get_logger().error('  • Check topic list: ros2 topic list')
            self.get_logger().error('  • View logs: ros2 node info <node_name>')
            self.get_logger().error('')
            
            self.check_timer.cancel()
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    checker = KFReadinessChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        checker.get_logger().info('')
        checker.get_logger().info('Interrupted by user')
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
