#!/usr/bin/env python3
"""
Kalman Filter Only Evaluation Node

Evaluates ONLY the Kalman Filter fusion against ground truth.
Uses vector-based velocity comparison (IMM orientation.x/y vs Gazebo odom)
and Hungarian algorithm for globally optimal GT↔track association.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import math
import numpy as np
from collections import defaultdict
import time
from scipy.optimize import linear_sum_assignment


class KFOnlyEvaluator(Node):
    def __init__(self):
        super().__init__('kf_only_evaluator')
        
        # Parameters
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('association_threshold', 2.0)  # meters
        self.declare_parameter('output_file', '/tmp/kf_evaluation_results.txt')
        
        self.test_duration = self.get_parameter('test_duration').value
        self.association_threshold = self.get_parameter('association_threshold').value
        self.output_file = self.get_parameter('output_file').value
        
        # Ground truth subscribers (one per actor)
        self.gt_subscribers = []
        self.ground_truth = {}  # {actor_name: Odometry}
        for i in range(1, 11):  # actors 1-10
            actor_name = f'actor{i}'
            sub = self.create_subscription(
                Odometry,
                f'/actors/ground_truth/{actor_name}/odom',
                lambda msg, name=actor_name: self.gt_callback(msg, name),
                10
            )
            self.gt_subscribers.append(sub)
        
        # KF fusion subscriber
        self.kf_fusion_poses = None
        self.kf_fusion_sub = self.create_subscription(
            PoseArray,
            '/fused_humans_kf/poses',
            self.kf_fusion_callback,
            10
        )
        
        # Data storage for errors
        # Structure: {actor_name: {'pos': [], 'vel': []}}
        self.errors = defaultdict(lambda: {'pos': [], 'vel': []})
        
        # Tracking statistics
        self.kf_detections = defaultdict(int)
        self.total_samples = 0
        
        # Timer for evaluation
        self.start_time = time.time()
        self.evaluation_timer = self.create_timer(0.1, self.evaluate)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Kalman Filter Only Evaluation Node Started')
        self.get_logger().info(f'Test Duration: {self.test_duration} seconds')
        self.get_logger().info(f'Association Threshold: {self.association_threshold} meters')
        self.get_logger().info(f'Output File: {self.output_file}')
        self.get_logger().info('Velocity: vector-based (IMM orientation.x/y vs odom twist)')
        self.get_logger().info('Association: Hungarian algorithm (global optimal)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Collecting KF fusion data...')
    
    def gt_callback(self, msg, actor_name):
        """Store ground truth data for an actor."""
        self.ground_truth[actor_name] = msg
    
    def kf_fusion_callback(self, msg):
        """Store KF fusion poses."""
        self.kf_fusion_poses = msg
    
    def associate_detections(self, gt_positions, fusion_poses):
        """
        Globally optimal GT↔track association using the Hungarian algorithm.

        Args:
            gt_positions: list of (actor_name, gt_odom) tuples
            fusion_poses: PoseArray from KF fusion

        Returns:
            List of (actor_name, gt_odom, matched_pose_index) tuples.
            Only includes matches within the association threshold.
        """
        if fusion_poses is None or len(fusion_poses.poses) == 0:
            return []

        n_gt = len(gt_positions)
        n_det = len(fusion_poses.poses)

        # Build Euclidean distance cost matrix (rows=GT, cols=detections)
        cost_matrix = np.full((n_gt, n_det), fill_value=1e6)
        for i, (_, gt_odom) in enumerate(gt_positions):
            gt_x = gt_odom.pose.pose.position.x
            gt_y = gt_odom.pose.pose.position.y
            for j, pose in enumerate(fusion_poses.poses):
                dx = gt_x - pose.position.x
                dy = gt_y - pose.position.y
                cost_matrix[i, j] = math.sqrt(dx * dx + dy * dy)

        # Solve global optimal assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        matches = []
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.association_threshold:
                actor_name, gt_odom = gt_positions[r]
                matches.append((actor_name, gt_odom, c))

        return matches
    
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
        
        # Build list of GT entries
        gt_positions = [(name, odom) for name, odom in self.ground_truth.items()]
        
        # Global optimal association via Hungarian algorithm
        matches = self.associate_detections(gt_positions, self.kf_fusion_poses)
        
        for actor_name, gt_odom, det_idx in matches:
            detected_pose = self.kf_fusion_poses.poses[det_idx]
            gt_pos = gt_odom.pose.pose.position
            
            # --- Position error ---
            dx = gt_pos.x - detected_pose.position.x
            dy = gt_pos.y - detected_pose.position.y
            pos_error = math.sqrt(dx * dx + dy * dy)
            self.errors[actor_name]['pos'].append(pos_error)
            
            # --- Velocity error (vector-based Euclidean) ---
            # Ground truth velocity from Gazebo odom
            gt_vx = gt_odom.twist.twist.linear.x
            gt_vy = gt_odom.twist.twist.linear.y
            
            # Estimated velocity from IMM (packed in orientation.x/y)
            estimated_vx = detected_pose.orientation.x
            estimated_vy = detected_pose.orientation.y
            
            vel_error = math.sqrt(
                (gt_vx - estimated_vx) ** 2 + (gt_vy - estimated_vy) ** 2
            )
            self.errors[actor_name]['vel'].append(vel_error)
            
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
        results.append('KALMAN FILTER FUSION EVALUATION RESULTS')
        results.append('=' * 80)
        results.append(f'Test Duration: {self.test_duration} seconds')
        results.append(f'Total Samples: {self.total_samples}')
        results.append(f'Association Threshold: {self.association_threshold} meters')
        results.append(f'Association Method: Hungarian (global optimal)')
        results.append(f'Velocity Method: Vector Euclidean (IMM vx/vy vs odom vx/vy)')
        results.append('=' * 80)
        results.append('')
        
        # Per-actor results
        results.append('PER-ACTOR POSITION RESULTS:')
        results.append('-' * 80)
        results.append(f'{"Actor":<12} {"Detections":<12} {"Mean Err (m)":<15} {"RMSE (m)":<15} {"Std Dev (m)":<15}')
        results.append('-' * 80)
        
        all_pos_errors = []
        all_vel_errors = []
        
        for actor_name in sorted(self.ground_truth.keys()):
            # Position
            pos_errors = self.errors[actor_name]['pos']
            if len(pos_errors) > 0:
                pos_mean = np.mean(pos_errors)
                pos_rmse = np.sqrt(np.mean(np.square(pos_errors)))
                pos_std = np.std(pos_errors)
                detections = self.kf_detections[actor_name]
                all_pos_errors.extend(pos_errors)
            else:
                pos_mean = pos_rmse = pos_std = float('nan')
                detections = 0
            
            results.append(
                f'{actor_name:<12} {detections:<12} '
                f'{pos_mean:<15.4f} {pos_rmse:<15.4f} {pos_std:<15.4f}'
            )
        
        results.append('-' * 80)
        results.append('')
        
        # Per-actor velocity results
        results.append('PER-ACTOR VELOCITY RESULTS:')
        results.append('-' * 80)
        results.append(f'{"Actor":<12} {"Vel Samples":<12} {"Mean Err (m/s)":<15} {"RMSE (m/s)":<15} {"Std Dev (m/s)":<15}')
        results.append('-' * 80)
        
        for actor_name in sorted(self.ground_truth.keys()):
            # Velocity
            vel_errors = self.errors[actor_name]['vel']
            if len(vel_errors) > 0:
                vel_mean = np.mean(vel_errors)
                vel_rmse = np.sqrt(np.mean(np.square(vel_errors)))
                vel_std = np.std(vel_errors)
                vel_samples = len(vel_errors)
                all_vel_errors.extend(vel_errors)
            else:
                vel_mean = vel_rmse = vel_std = float('nan')
                vel_samples = 0
            
            results.append(
                f'{actor_name:<12} {vel_samples:<12} '
                f'{vel_mean:<15.4f} {vel_rmse:<15.4f} {vel_std:<15.4f}'
            )
        
        results.append('-' * 80)
        results.append('')
        
        # Overall results
        results.append('OVERALL POSITION RESULTS:')
        results.append('-' * 80)
        results.append(f'{"Metric":<25} {"Value":<15}')
        results.append('-' * 80)
        
        if len(all_pos_errors) > 0:
            pos_mean = np.mean(all_pos_errors)
            pos_rmse = np.sqrt(np.mean(np.square(all_pos_errors)))
            pos_std = np.std(all_pos_errors)
            pos_min = np.min(all_pos_errors)
            pos_max = np.max(all_pos_errors)
            total_detections = sum(self.kf_detections.values())
            
            results.append(f'{"Total Detections":<25} {total_detections:<15}')
            results.append(f'{"Mean Error (m)":<25} {pos_mean:<15.4f}')
            results.append(f'{"RMSE (m)":<25} {pos_rmse:<15.4f}')
            results.append(f'{"Std Deviation (m)":<25} {pos_std:<15.4f}')
            results.append(f'{"Min Error (m)":<25} {pos_min:<15.4f}')
            results.append(f'{"Max Error (m)":<25} {pos_max:<15.4f}')
        else:
            results.append('No position data collected')
        
        results.append('-' * 80)
        results.append('')
        
        # Overall velocity results
        results.append('OVERALL VELOCITY RESULTS:')
        results.append('-' * 80)
        results.append(f'{"Metric":<25} {"Value":<15}')
        results.append('-' * 80)
        
        if len(all_vel_errors) > 0:
            vel_mean = np.mean(all_vel_errors)
            vel_rmse = np.sqrt(np.mean(np.square(all_vel_errors)))
            vel_std = np.std(all_vel_errors)
            vel_min = np.min(all_vel_errors)
            vel_max = np.max(all_vel_errors)
            vel_samples = len(all_vel_errors)
            
            results.append(f'{"Total Vel Samples":<25} {vel_samples:<15}')
            results.append(f'{"Mean Error (m/s)":<25} {vel_mean:<15.4f}')
            results.append(f'{"RMSE (m/s)":<25} {vel_rmse:<15.4f}')
            results.append(f'{"Std Deviation (m/s)":<25} {vel_std:<15.4f}')
            results.append(f'{"Min Error (m/s)":<25} {vel_min:<15.4f}')
            results.append(f'{"Max Error (m/s)":<25} {vel_max:<15.4f}')
        else:
            results.append('No velocity data collected')
        
        results.append('-' * 80)
        results.append('')
        
        # Summary
        results.append('SUMMARY:')
        results.append('-' * 80)
        
        if len(all_pos_errors) > 0 and len(all_vel_errors) > 0:
            results.append(f'✓ Position Accuracy:  RMSE = {pos_rmse:.4f} m')
            results.append(f'✓ Velocity Accuracy:  RMSE = {vel_rmse:.4f} m/s')
            results.append(f'✓ Total Tracked Time: {total_detections / 10.0:.1f} seconds')
            results.append(f'✓ Actors Detected:    {len([k for k in self.ground_truth.keys() if self.kf_detections[k] > 0])}')
        else:
            results.append('✗ Insufficient data collected')
        
        results.append('=' * 80)
        
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
    evaluator = KFOnlyEvaluator()
    
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
