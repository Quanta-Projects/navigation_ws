#!/usr/bin/env python3
"""
Fusion Evaluation Node

Compares Standard EMA Fusion vs Kalman Filter Fusion against ground truth.
Calculates position and velocity errors for each detected human.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from collections import defaultdict
import time


class FusionEvaluator(Node):
    def __init__(self):
        super().__init__('fusion_evaluator')
        
        # Parameters
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('association_threshold', 2.0)  # meters
        self.declare_parameter('output_file', '/tmp/fusion_evaluation_results.txt')
        
        self.test_duration = self.get_parameter('test_duration').value
        self.association_threshold = self.get_parameter('association_threshold').value
        self.output_file = self.get_parameter('output_file').value
        
        # Ground truth subscribers (one per actor)
        self.gt_subscribers = []
        self.ground_truth = {}  # {actor_name: Odometry}
        for i in range(1, 6):  # actors 1-5
            actor_name = f'actor{i}'
            sub = self.create_subscription(
                Odometry,
                f'/actors/ground_truth/{actor_name}/odom',
                lambda msg, name=actor_name: self.gt_callback(msg, name),
                10
            )
            self.gt_subscribers.append(sub)
        
        # Standard fusion subscribers
        self.std_fusion_poses = None
        self.std_fusion_sub = self.create_subscription(
            PoseArray,
            '/fused_humans/poses',
            self.std_fusion_callback,
            10
        )
        
        # KF fusion subscribers
        self.kf_fusion_poses = None
        self.kf_fusion_sub = self.create_subscription(
            PoseArray,
            '/fused_humans_kf/poses',
            self.kf_fusion_callback,
            10
        )
        
        # Data storage for errors
        # Structure: {actor_name: {'std': {'pos': [], 'vel': []}, 'kf': {'pos': [], 'vel': []}}}
        self.errors = defaultdict(lambda: {
            'std': {'pos': [], 'vel': []},
            'kf': {'pos': [], 'vel': []}
        })
        
        # Tracking statistics
        self.std_detections = defaultdict(int)  # Count detections per actor
        self.kf_detections = defaultdict(int)
        self.total_samples = 0
        
        # Position history for velocity calculation
        self.std_last_positions = {}  # {actor_name: (position, timestamp)}
        self.kf_last_positions = {}   # {actor_name: (position, timestamp)}
        
        # Timer for evaluation
        self.start_time = time.time()
        self.evaluation_timer = self.create_timer(0.1, self.evaluate)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Fusion Evaluation Node Started')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')
        self.get_logger().info(f'Association Threshold: {self.association_threshold} meters')
        self.get_logger().info(f'Output File: {self.output_file}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Collecting data...')
    
    def gt_callback(self, msg, actor_name):
        """Store ground truth data for an actor."""
        self.ground_truth[actor_name] = msg
    
    def std_fusion_callback(self, msg):
        """Store standard fusion poses."""
        self.std_fusion_poses = msg
    
    def kf_fusion_callback(self, msg):
        """Store KF fusion poses."""
        self.kf_fusion_poses = msg
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions."""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_velocity_magnitude(self, twist):
        """Calculate velocity magnitude from twist."""
        vx = twist.linear.x
        vy = twist.linear.y
        return math.sqrt(vx**2 + vy**2)
    
    def calculate_velocity_from_positions(self, pos1, pos2, dt):
        """Calculate velocity magnitude from position change."""
        if dt <= 0:
            return 0.0
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        distance = math.sqrt(dx**2 + dy**2)
        return distance / dt
    
    def find_closest_detection(self, gt_pos, fusion_poses):
        """
        Find the closest detected human to a ground truth position.
        Returns (index, distance) or (None, None) if no match within threshold.
        """
        if fusion_poses is None or len(fusion_poses.poses) == 0:
            return None, None
        
        min_dist = float('inf')
        min_idx = None
        
        for idx, pose in enumerate(fusion_poses.poses):
            dist = self.calculate_distance(gt_pos, pose.position)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        
        # Only return if within association threshold
        if min_dist < self.association_threshold:
            return min_idx, min_dist
        else:
            return None, None
    
    def evaluate(self):
        """Main evaluation loop - called at 10 Hz."""
        elapsed_time = time.time() - self.start_time
        
        # Check if test duration reached
        if elapsed_time >= self.test_duration:
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info('Test duration reached. Generating results...')
            self.generate_results()
            self.evaluation_timer.cancel()
            return
        
        # Need at least ground truth data
        if len(self.ground_truth) == 0:
            return
        
        self.total_samples += 1
        
        current_time = time.time()
        
        # For each ground truth actor
        for actor_name, gt_odom in self.ground_truth.items():
            gt_pos = gt_odom.pose.pose.position
            gt_vel = self.calculate_velocity_magnitude(gt_odom.twist.twist)
            
            # Evaluate Standard Fusion
            if self.std_fusion_poses is not None:
                idx, dist = self.find_closest_detection(gt_pos, self.std_fusion_poses)
                if idx is not None:
                    # Found a match
                    detected_pos = self.std_fusion_poses.poses[idx].position
                    pos_error = self.calculate_distance(gt_pos, detected_pos)
                    
                    self.errors[actor_name]['std']['pos'].append(pos_error)
                    
                    # Calculate velocity from position history
                    if actor_name in self.std_last_positions:
                        last_pos, last_time = self.std_last_positions[actor_name]
                        dt = current_time - last_time
                        if dt > 0.05:  # At least 50ms between measurements
                            estimated_vel = self.calculate_velocity_from_positions(last_pos, detected_pos, dt)
                            vel_error = abs(estimated_vel - gt_vel)
                            self.errors[actor_name]['std']['vel'].append(vel_error)
                    
                    # Update position history
                    self.std_last_positions[actor_name] = (detected_pos, current_time)
                    self.std_detections[actor_name] += 1
            
            # Evaluate KF Fusion
            if self.kf_fusion_poses is not None:
                idx, dist = self.find_closest_detection(gt_pos, self.kf_fusion_poses)
                if idx is not None:
                    # Found a match
                    detected_pos = self.kf_fusion_poses.poses[idx].position
                    pos_error = self.calculate_distance(gt_pos, detected_pos)
                    
                    self.errors[actor_name]['kf']['pos'].append(pos_error)
                    
                    # Calculate velocity from position history
                    if actor_name in self.kf_last_positions:
                        last_pos, last_time = self.kf_last_positions[actor_name]
                        dt = current_time - last_time
                        if dt > 0.05:  # At least 50ms between measurements
                            estimated_vel = self.calculate_velocity_from_positions(last_pos, detected_pos, dt)
                            vel_error = abs(estimated_vel - gt_vel)
                            self.errors[actor_name]['kf']['vel'].append(vel_error)
                    
                    # Update position history
                    self.kf_last_positions[actor_name] = (detected_pos, current_time)
                    self.kf_detections[actor_name] += 1
        
        # Progress update every 5 seconds
        if self.total_samples % 50 == 0:  # 10 Hz * 5 sec = 50 samples
            self.get_logger().info(
                f'Progress: {elapsed_time:.1f}s / {self.test_duration:.1f}s '
                f'({100*elapsed_time/self.test_duration:.0f}%)'
            )
    
    def generate_results(self):
        """Generate and display evaluation results."""
        
        results = []
        results.append('\n' + '=' * 80)
        results.append('FUSION EVALUATION RESULTS')
        results.append('=' * 80)
        results.append(f'Test Duration: {self.test_duration} seconds')
        results.append(f'Total Samples: {self.total_samples}')
        results.append(f'Association Threshold: {self.association_threshold} meters')
        results.append('=' * 80)
        results.append('')
        
        # Per-actor results
        results.append('PER-ACTOR POSITION RESULTS:')
        results.append('-' * 100)
        results.append(f'{"Actor":<10} {"Method":<15} {"Detections":<12} {"Mean Err (m)":<15} {"RMSE (m)":<15} {"Std Dev (m)":<15}')
        results.append('-' * 100)
        
        all_std_pos_errors = []
        all_kf_pos_errors = []
        all_std_vel_errors = []
        all_kf_vel_errors = []
        
        for actor_name in sorted(self.ground_truth.keys()):
            # Standard Fusion - Position
            std_pos_errors = self.errors[actor_name]['std']['pos']
            if len(std_pos_errors) > 0:
                std_pos_mean = np.mean(std_pos_errors)
                std_pos_rmse = np.sqrt(np.mean(np.square(std_pos_errors)))
                std_pos_std = np.std(std_pos_errors)
                std_detections = self.std_detections[actor_name]
                all_std_pos_errors.extend(std_pos_errors)
            else:
                std_pos_mean = std_pos_rmse = std_pos_std = float('nan')
                std_detections = 0
            
            results.append(
                f'{actor_name:<10} {"Standard EMA":<15} {std_detections:<12} '
                f'{std_pos_mean:<15.4f} {std_pos_rmse:<15.4f} {std_pos_std:<15.4f}'
            )
            
            # KF Fusion - Position
            kf_pos_errors = self.errors[actor_name]['kf']['pos']
            if len(kf_pos_errors) > 0:
                kf_pos_mean = np.mean(kf_pos_errors)
                kf_pos_rmse = np.sqrt(np.mean(np.square(kf_pos_errors)))
                kf_pos_std = np.std(kf_pos_errors)
                kf_detections = self.kf_detections[actor_name]
                all_kf_pos_errors.extend(kf_pos_errors)
            else:
                kf_pos_mean = kf_pos_rmse = kf_pos_std = float('nan')
                kf_detections = 0
            
            results.append(
                f'{actor_name:<10} {"Kalman Filter":<15} {kf_detections:<12} '
                f'{kf_pos_mean:<15.4f} {kf_pos_rmse:<15.4f} {kf_pos_std:<15.4f}'
            )
            results.append('')
        
        results.append('-' * 100)
        results.append('')
        
        # Per-actor velocity results
        results.append('PER-ACTOR VELOCITY RESULTS:')
        results.append('-' * 100)
        results.append(f'{"Actor":<10} {"Method":<15} {"Vel Samples":<12} {"Mean Err (m/s)":<15} {"RMSE (m/s)":<15} {"Std Dev (m/s)":<15}')
        results.append('-' * 100)
        
        for actor_name in sorted(self.ground_truth.keys()):
            # Standard Fusion - Velocity
            std_vel_errors = self.errors[actor_name]['std']['vel']
            if len(std_vel_errors) > 0:
                std_vel_mean = np.mean(std_vel_errors)
                std_vel_rmse = np.sqrt(np.mean(np.square(std_vel_errors)))
                std_vel_std = np.std(std_vel_errors)
                std_vel_samples = len(std_vel_errors)
                all_std_vel_errors.extend(std_vel_errors)
            else:
                std_vel_mean = std_vel_rmse = std_vel_std = float('nan')
                std_vel_samples = 0
            
            results.append(
                f'{actor_name:<10} {"Standard EMA":<15} {std_vel_samples:<12} '
                f'{std_vel_mean:<15.4f} {std_vel_rmse:<15.4f} {std_vel_std:<15.4f}'
            )
            
            # KF Fusion - Velocity
            kf_vel_errors = self.errors[actor_name]['kf']['vel']
            if len(kf_vel_errors) > 0:
                kf_vel_mean = np.mean(kf_vel_errors)
                kf_vel_rmse = np.sqrt(np.mean(np.square(kf_vel_errors)))
                kf_vel_std = np.std(kf_vel_errors)
                kf_vel_samples = len(kf_vel_errors)
                all_kf_vel_errors.extend(kf_vel_errors)
            else:
                kf_vel_mean = kf_vel_rmse = kf_vel_std = float('nan')
                kf_vel_samples = 0
            
            results.append(
                f'{actor_name:<10} {"Kalman Filter":<15} {kf_vel_samples:<12} '
                f'{kf_vel_mean:<15.4f} {kf_vel_rmse:<15.4f} {kf_vel_std:<15.4f}'
            )
            results.append('')
        
        results.append('-' * 100)
        results.append('')
        
        # Overall results
        results.append('OVERALL POSITION RESULTS:')
        results.append('-' * 100)
        results.append(f'{"Method":<20} {"Total Detections":<20} {"Mean Err (m)":<15} {"RMSE (m)":<15} {"Std Dev (m)":<15}')
        results.append('-' * 100)
        
        # Standard Fusion Overall - Position
        if len(all_std_pos_errors) > 0:
            std_pos_mean = np.mean(all_std_pos_errors)
            std_pos_rmse = np.sqrt(np.mean(np.square(all_std_pos_errors)))
            std_pos_std = np.std(all_std_pos_errors)
            std_total_detections = sum(self.std_detections.values())
        else:
            std_pos_mean = std_pos_rmse = std_pos_std = float('nan')
            std_total_detections = 0
        
        results.append(
            f'{"Standard EMA":<20} {std_total_detections:<20} '
            f'{std_pos_mean:<15.4f} {std_pos_rmse:<15.4f} {std_pos_std:<15.4f}'
        )
        
        # KF Fusion Overall - Position
        if len(all_kf_pos_errors) > 0:
            kf_pos_mean = np.mean(all_kf_pos_errors)
            kf_pos_rmse = np.sqrt(np.mean(np.square(all_kf_pos_errors)))
            kf_pos_std = np.std(all_kf_pos_errors)
            kf_total_detections = sum(self.kf_detections.values())
        else:
            kf_pos_mean = kf_pos_rmse = kf_pos_std = float('nan')
            kf_total_detections = 0
        
        results.append(
            f'{"Kalman Filter":<20} {kf_total_detections:<20} '
            f'{kf_pos_mean:<15.4f} {kf_pos_rmse:<15.4f} {kf_pos_std:<15.4f}'
        )
        
        results.append('-' * 100)
        results.append('')
        
        # Overall velocity results
        results.append('OVERALL VELOCITY RESULTS:')
        results.append('-' * 100)
        results.append(f'{"Method":<20} {"Total Vel Samples":<20} {"Mean Err (m/s)":<15} {"RMSE (m/s)":<15} {"Std Dev (m/s)":<15}')
        results.append('-' * 100)
        
        # Standard Fusion Overall - Velocity
        if len(all_std_vel_errors) > 0:
            std_vel_mean = np.mean(all_std_vel_errors)
            std_vel_rmse = np.sqrt(np.mean(np.square(all_std_vel_errors)))
            std_vel_std = np.std(all_std_vel_errors)
            std_vel_samples = len(all_std_vel_errors)
        else:
            std_vel_mean = std_vel_rmse = std_vel_std = float('nan')
            std_vel_samples = 0
        
        results.append(
            f'{"Standard EMA":<20} {std_vel_samples:<20} '
            f'{std_vel_mean:<15.4f} {std_vel_rmse:<15.4f} {std_vel_std:<15.4f}'
        )
        
        # KF Fusion Overall - Velocity
        if len(all_kf_vel_errors) > 0:
            kf_vel_mean = np.mean(all_kf_vel_errors)
            kf_vel_rmse = np.sqrt(np.mean(np.square(all_kf_vel_errors)))
            kf_vel_std = np.std(all_kf_vel_errors)
            kf_vel_samples = len(all_kf_vel_errors)
        else:
            kf_vel_mean = kf_vel_rmse = kf_vel_std = float('nan')
            kf_vel_samples = 0
        
        results.append(
            f'{"Kalman Filter":<20} {kf_vel_samples:<20} '
            f'{kf_vel_mean:<15.4f} {kf_vel_rmse:<15.4f} {kf_vel_std:<15.4f}'
        )
        
        results.append('-' * 100)
        results.append('')
        
        # Comparison
        results.append('COMPARISON:')
        results.append('-' * 100)
        
        # Position comparison
        if len(all_std_pos_errors) > 0 and len(all_kf_pos_errors) > 0:
            pos_improvement = ((std_pos_rmse - kf_pos_rmse) / std_pos_rmse) * 100
            results.append('POSITION:')
            if pos_improvement > 0:
                results.append(f'  ✓ Kalman Filter is {pos_improvement:.1f}% more accurate (lower position RMSE)')
            else:
                results.append(f'  ✓ Standard EMA is {-pos_improvement:.1f}% more accurate (lower position RMSE)')
        else:
            results.append('POSITION: Insufficient data for comparison')
        
        results.append('')
        
        # Velocity comparison
        if len(all_std_vel_errors) > 0 and len(all_kf_vel_errors) > 0:
            vel_improvement = ((std_vel_rmse - kf_vel_rmse) / std_vel_rmse) * 100
            results.append('VELOCITY:')
            if vel_improvement > 0:
                results.append(f'  ✓ Kalman Filter is {vel_improvement:.1f}% more accurate (lower velocity RMSE)')
            else:
                results.append(f'  ✓ Standard EMA is {-vel_improvement:.1f}% more accurate (lower velocity RMSE)')
        else:
            results.append('VELOCITY: Insufficient data for comparison')
        
        results.append('=' * 100)
        
        # Print to console
        for line in results:
            self.get_logger().info(line)
        
        # Write to file
        try:
            with open(self.output_file, 'w') as f:
                f.write('\n'.join(results))
            self.get_logger().info(f'\nResults saved to: {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to write results to file: {e}')
        
        # Shutdown
        self.get_logger().info('\nEvaluation complete. Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    evaluator = FusionEvaluator()
    
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        evaluator.get_logger().info('Interrupted by user')
        evaluator.generate_results()
    finally:
        evaluator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
