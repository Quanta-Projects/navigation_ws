#!/usr/bin/env python3
"""
Human Detection Fusion Node

Fuses vision-based (YOLO) and LiDAR-based (DR-SPAAM) human detections
with velocity estimation and intelligent FOV-based filtering.

Author: Navigation System
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseArray, Pose, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import math
from collections import defaultdict
from scipy.optimize import linear_sum_assignment


class HumanTrack:
    """Represents a tracked human with position history for velocity estimation."""
    
    def __init__(self, track_id, position, timestamp, confidence=0.5):
        self.id = track_id
        self.positions = [position]
        self.timestamps = [timestamp]
        self.velocity = np.array([0.0, 0.0])
        self.confidence = confidence  # Track confidence
        self.last_seen = timestamp
        self.max_history = 5
        
    def update(self, position, timestamp, confidence=None, alpha=0.3):
        """Update track with new position and calculate smoothed velocity."""
        self.positions.append(position)
        self.timestamps.append(timestamp)
        self.last_seen = timestamp
        
        # Update confidence if provided
        if confidence is not None:
            self.confidence = confidence
        
        # Keep limited history
        if len(self.positions) > self.max_history:
            self.positions.pop(0)
            self.timestamps.pop(0)
        
        # Calculate velocity using finite difference
        if len(self.positions) >= 2:
            dt = (self.timestamps[-1] - self.timestamps[-2]).nanoseconds / 1e9
            if dt > 0:
                raw_velocity = (self.positions[-1] - self.positions[-2]) / dt
                # Low-pass filter (exponential smoothing)
                self.velocity = alpha * raw_velocity + (1 - alpha) * self.velocity


class HumanFusionNode(Node):
    """
    Fuses YOLO and LiDAR human detections with intelligent FOV-based filtering.
    
    Strategy:
    - YOLO detections are anchor truth inside camera FOV
    - Match YOLO with nearby LiDAR for weighted fusion
    - LiDAR detections outside FOV provide blind spot coverage
    - Track fused detections over time to estimate velocity
    """
    
    def __init__(self):
        super().__init__('human_fusion_node')
        
        # Parameters
        self.declare_parameter('camera_fov_degrees', 110.0)
        self.declare_parameter('fusion_distance_threshold', 1.0)
        self.declare_parameter('yolo_confidence', 0.7)
        self.declare_parameter('lidar_confidence', 0.8)
        self.declare_parameter('velocity_alpha', 0.3)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('max_track_distance', 2.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.camera_fov_rad = math.radians(self.get_parameter('camera_fov_degrees').value)
        self.fusion_threshold = self.get_parameter('fusion_distance_threshold').value
        self.yolo_conf = self.get_parameter('yolo_confidence').value
        self.lidar_conf = self.get_parameter('lidar_confidence').value
        self.velocity_alpha = self.get_parameter('velocity_alpha').value
        self.track_timeout = self.get_parameter('track_timeout_sec').value
        self.max_track_dist = self.get_parameter('max_track_distance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.yolo_sub = self.create_subscription(
            PoseArray,
            'tracked_humans/poses',
            self.yolo_callback,
            10
        )
        self.lidar_sub = self.create_subscription(
            PoseArray,
            'detected_people',
            self.lidar_callback,
            10
        )
        
        # Publishers
        self.fused_poses_pub = self.create_publisher(
            PoseArray,
            'fused_humans/poses',
            10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            'fused_humans/markers',
            10
        )
        
        # State
        self.latest_yolo = None
        self.latest_lidar = None
        self.tracks = {}  # track_id -> HumanTrack
        self.next_track_id = 0
        
        # Fusion timer (10 Hz)
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)
        
        self.get_logger().info(f'Human Fusion Node initialized')
        self.get_logger().info(f'  Camera FOV: {self.get_parameter("camera_fov_degrees").value}°')
        self.get_logger().info(f'  Fusion threshold: {self.fusion_threshold}m')
        self.get_logger().info(f'  YOLO confidence: {self.yolo_conf}, LiDAR confidence: {self.lidar_conf}')
    
    def yolo_callback(self, msg):
        """Store latest YOLO detections."""
        self.latest_yolo = msg
    
    def lidar_callback(self, msg):
        """Store latest LiDAR detections."""
        self.latest_lidar = msg
    
    def get_robot_pose_and_yaw(self):
        """Get robot's position and heading in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            quat = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            return np.array([x, y]), yaw
        
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None
    
    def is_in_camera_fov(self, point, robot_pos, robot_yaw):
        """
        Check if a point (x, y) is within the robot's triangular camera FOV.
        
        Args:
            point: np.array([x, y]) in map frame
            robot_pos: np.array([x, y]) robot position in map frame
            robot_yaw: robot heading in radians
        
        Returns:
            bool: True if point is inside FOV
        """
        # Vector from robot to point
        to_point = point - robot_pos
        distance = np.linalg.norm(to_point)
        
        if distance < 0.1:  # Too close
            return True
        
        # Angle to point relative to robot heading
        angle_to_point = math.atan2(to_point[1], to_point[0])
        angle_diff = self.normalize_angle(angle_to_point - robot_yaw)
        
        # Check if within FOV
        half_fov = self.camera_fov_rad / 2.0
        return abs(angle_diff) <= half_fov
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def fusion_callback(self):
        """Main fusion loop executed at 10 Hz."""
        if self.latest_yolo is None or self.latest_lidar is None:
            return
        
        # Get robot pose
        robot_pos, robot_yaw = self.get_robot_pose_and_yaw()
        if robot_pos is None:
            return
        
        # Convert poses to numpy arrays and extract confidence from published topics
        yolo_points = np.array([[p.position.x, p.position.y] for p in self.latest_yolo.poses])
        # Extract actual confidence from YOLO tracker (orientation.z field)
        yolo_conf = np.array([
            p.orientation.z if (p.orientation.z > 0.0) else self.yolo_conf 
            for p in self.latest_yolo.poses
        ])
        
        lidar_points = np.array([[p.position.x, p.position.y] for p in self.latest_lidar.poses])
        # Extract actual confidence from LiDAR detector (orientation.z field)
        lidar_conf = np.array([
            p.orientation.z if (p.orientation.z > 0.0) else self.lidar_conf 
            for p in self.latest_lidar.poses
        ])
        
        # Log confidence statistics
        if len(yolo_conf) > 0:
            yolo_using_actual = sum(1 for p in self.latest_yolo.poses if p.orientation.z > 0.0)
            self.get_logger().info(
                f'YOLO: {len(yolo_conf)} detections, {yolo_using_actual} with confidence '
                f'(avg: {np.mean(yolo_conf):.3f}, range: {np.min(yolo_conf):.3f}-{np.max(yolo_conf):.3f})',
                throttle_duration_sec=2.0
            )
        
        if len(lidar_conf) > 0:
            lidar_using_actual = sum(1 for p in self.latest_lidar.poses if p.orientation.z > 0.0)
            self.get_logger().info(
                f'LiDAR: {len(lidar_conf)} detections, {lidar_using_actual} with confidence '
                f'(avg: {np.mean(lidar_conf):.3f}, range: {np.min(lidar_conf):.3f}-{np.max(lidar_conf):.3f})',
                throttle_duration_sec=2.0
            )
        
        fused_positions = []
        fused_confidences = []  # Track confidence for each fused detection
        used_lidar_indices = set()
        current_time = self.get_clock().now()
        
        # ======== STEP A: Process YOLO Detections (Anchor Truth) ========
        for idx, yolo_point in enumerate(yolo_points):
            best_match_idx = None
            best_match_dist = float('inf')
            
            # Find closest LiDAR detection
            for i, lidar_point in enumerate(lidar_points):
                if i in used_lidar_indices:
                    continue
                dist = np.linalg.norm(yolo_point - lidar_point)
                if dist < self.fusion_threshold and dist < best_match_dist:
                    best_match_dist = dist
                    best_match_idx = i
            
            if best_match_idx is not None:
                # MATCH FOUND: Weighted average fusion using actual confidence from detectors
                lidar_point = lidar_points[best_match_idx]
                y_conf = float(yolo_conf[idx])
                l_conf = float(lidar_conf[best_match_idx])
                
                # Weighted position fusion based on detector confidence
                fused_point = (
                    yolo_point * y_conf + lidar_point * l_conf
                ) / (y_conf + l_conf)
                
                # Average confidence from both sensors
                fused_confidence = (y_conf + l_conf) / 2.0
                
                fused_positions.append(fused_point)
                fused_confidences.append(fused_confidence)
                used_lidar_indices.add(best_match_idx)
                
                # Log fusion details
                self.get_logger().debug(
                    f'Fused YOLO (conf={y_conf:.3f}) + LiDAR (conf={l_conf:.3f}) '
                    f'-> final_conf={fused_confidence:.3f}, dist={best_match_dist:.3f}m'
                )
            else:
                # NO MATCH: Keep YOLO detection with its original confidence
                y_conf = float(yolo_conf[idx])
                fused_positions.append(yolo_point)
                fused_confidences.append(y_conf)
                
                self.get_logger().debug(
                    f'YOLO-only detection (conf={y_conf:.3f}, no LiDAR match)'
                )
        
        # ======== STEP B: Process Remaining LiDAR Detections ========
        for i, lidar_point in enumerate(lidar_points):
            if i in used_lidar_indices:
                continue
            
            in_fov = self.is_in_camera_fov(lidar_point, robot_pos, robot_yaw)
            
            if in_fov:
                # Inside FOV but no YOLO match -> Discard (assume false positive)
                self.get_logger().debug(
                    f'Discarding LiDAR detection in FOV (conf={lidar_conf[i]:.3f}) - no YOLO confirmation'
                )
            else:
                # Outside FOV -> Keep with original LiDAR confidence (blind spot coverage)
                l_conf = float(lidar_conf[i])
                fused_positions.append(lidar_point)
                fused_confidences.append(l_conf)
                
                self.get_logger().debug(
                    f'LiDAR blind-spot detection (conf={l_conf:.3f}, outside FOV)'
                )
        
        # ======== STEP C: Track Association & Velocity Estimation ========
        fused_positions = np.array(fused_positions) if fused_positions else np.empty((0, 2))
        fused_confidences = np.array(fused_confidences) if fused_confidences else np.empty(0)
        
        if len(fused_positions) > 0:
            # Associate with existing tracks using Hungarian algorithm
            track_ids = list(self.tracks.keys())
            
            if len(track_ids) > 0:
                # Build cost matrix
                cost_matrix = np.zeros((len(track_ids), len(fused_positions)))
                for i, tid in enumerate(track_ids):
                    track_pos = self.tracks[tid].positions[-1]
                    for j, fused_pos in enumerate(fused_positions):
                        cost_matrix[i, j] = np.linalg.norm(track_pos - fused_pos)
                
                # Solve assignment problem
                row_ind, col_ind = linear_sum_assignment(cost_matrix)
                
                # Update matched tracks
                matched_fused = set()
                for i, j in zip(row_ind, col_ind):
                    if cost_matrix[i, j] < self.max_track_dist:
                        tid = track_ids[i]
                        self.tracks[tid].update(
                            fused_positions[j], 
                            current_time, 
                            confidence=fused_confidences[j],
                            alpha=self.velocity_alpha
                        )
                        matched_fused.add(j)
                
                # Create new tracks for unmatched detections
                for j, fused_pos in enumerate(fused_positions):
                    if j not in matched_fused:
                        self.tracks[self.next_track_id] = HumanTrack(
                            self.next_track_id,
                            fused_pos,
                            current_time,
                            confidence=fused_confidences[j]
                        )
                        self.next_track_id += 1
            else:
                # No existing tracks, create new ones
                for i, fused_pos in enumerate(fused_positions):
                    self.tracks[self.next_track_id] = HumanTrack(
                        self.next_track_id,
                        fused_pos,
                        current_time,
                        confidence=fused_confidences[i]
                    )
                    self.next_track_id += 1
        
        # Remove stale tracks
        timeout_ns = int(self.track_timeout * 1e9)
        stale_tracks = [
            tid for tid, track in self.tracks.items()
            if (current_time - track.last_seen).nanoseconds > timeout_ns
        ]
        for tid in stale_tracks:
            del self.tracks[tid]
        
        # ======== STEP D: Publish Results ========
        if len(self.tracks) > 0:
            # Log fusion summary
            avg_confidence = np.mean([t.confidence for t in self.tracks.values()])
            self.get_logger().info(
                f'Publishing {len(self.tracks)} tracked humans '
                f'(avg confidence: {avg_confidence:.3f})',
                throttle_duration_sec=1.0
            )
        
        self.publish_fused_poses(current_time)
        self.publish_markers(current_time)
    
    def publish_fused_poses(self, timestamp):
        """Publish fused human poses with confidence."""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp.to_msg()
        pose_array.header.frame_id = self.map_frame
        
        for track in self.tracks.values():
            pose = Pose()
            pose.position.x = track.positions[-1][0]
            pose.position.y = track.positions[-1][1]
            pose.position.z = 0.0
            
            # Encode confidence in orientation.z
            pose.orientation.z = track.confidence
            
            # Encode velocity magnitude in orientation.w (for downstream use)
            vel_mag = np.linalg.norm(track.velocity)
            pose.orientation.w = min(vel_mag, 5.0)  # Clamp to 5 m/s
            
            pose_array.poses.append(pose)
        
        self.fused_poses_pub.publish(pose_array)
    
    def publish_markers(self, timestamp):
        """Publish visualization markers (cylinders + velocity arrows)."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for track in self.tracks.values():
            pos = track.positions[-1]
            vel = track.velocity
            
            # Cylinder marker for human
            cylinder = Marker()
            cylinder.header.stamp = timestamp.to_msg()
            cylinder.header.frame_id = self.map_frame
            cylinder.ns = 'fused_humans'
            cylinder.id = marker_id
            marker_id += 1
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            
            cylinder.pose.position.x = pos[0]
            cylinder.pose.position.y = pos[1]
            cylinder.pose.position.z = 0.9  # Human height center
            cylinder.pose.orientation.w = 1.0
            
            cylinder.scale.x = 0.4  # Diameter
            cylinder.scale.y = 0.4
            cylinder.scale.z = 1.8  # Human height
            
            cylinder.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)  # Blue
            cylinder.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(cylinder)
            
            # Velocity arrow
            vel_mag = np.linalg.norm(vel)
            if vel_mag > 0.1:  # Only show if moving
                arrow = Marker()
                arrow.header.stamp = timestamp.to_msg()
                arrow.header.frame_id = self.map_frame
                arrow.ns = 'velocities'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                # Arrow from current position to position + velocity
                start = Point(x=pos[0], y=pos[1], z=0.1)
                end = Point(x=pos[0] + vel[0], y=pos[1] + vel[1], z=0.1)
                arrow.points = [start, end]
                
                arrow.scale.x = 0.1  # Shaft diameter
                arrow.scale.y = 0.15  # Head diameter
                arrow.scale.z = 0.2  # Head length
                
                arrow.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
                arrow.lifetime = Duration(seconds=0.5).to_msg()
                marker_array.markers.append(arrow)
            
            # Text label with track ID, velocity, and confidence
            text = Marker()
            text.header.stamp = timestamp.to_msg()
            text.header.frame_id = self.map_frame
            text.ns = 'labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.pose.position.x = pos[0]
            text.pose.position.y = pos[1]
            text.pose.position.z = 2.0
            text.pose.orientation.w = 1.0
            
            # Include confidence in label
            text.text = f'ID:{track.id}\n{vel_mag:.2f}m/s\nConf:{track.confidence:.2f}'
            text.scale.z = 0.3
            
            # Color by confidence: green=high, yellow=medium, red=low
            if track.confidence >= 0.7:
                text.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            elif track.confidence >= 0.5:
                text.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
            else:
                text.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
            
            text.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(text)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HumanFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
