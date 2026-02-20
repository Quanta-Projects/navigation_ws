#!/usr/bin/env python3
"""
Human Detection Fusion Node with Kalman Filter

Fuses vision-based (YOLO) and LiDAR-based (DR-SPAAM) human detections
with Kalman Filter tracking using constant velocity model.

Author: Achira Hansindu
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import math
from scipy.optimize import linear_sum_assignment
from message_filters import ApproximateTimeSynchronizer, Subscriber


class KalmanFilter:
    """
    Kalman Filter with constant velocity model for 2D tracking.
    
    State: [x, y, vx, vy]
    Measurement: [x, y]
    """
    
    def __init__(self, initial_position, initial_confidence=0.5, 
                 process_noise_pos=0.1, process_noise_vel=0.5,
                 measurement_noise=0.3, dt=0.1):
        """
        Initialize Kalman Filter
        
        Args:
            initial_position: [x, y] initial position
            initial_confidence: Detection confidence
            process_noise_pos: Process noise for position (m²)
            process_noise_vel: Process noise for velocity (m²/s²)
            measurement_noise: Measurement noise (m²)
            dt: Time step (s)
        """
        # State vector: [x, y, vx, vy]
        self.x = np.array([
            initial_position[0],
            initial_position[1],
            0.0,  # Initial velocity x
            0.0   # Initial velocity y
        ])
        
        # State covariance matrix (initial uncertainty)
        self.P = np.diag([
            measurement_noise,  # x uncertainty
            measurement_noise,  # y uncertainty
            1.0,  # vx uncertainty
            1.0   # vy uncertainty
        ])
        
        # Time step
        self.dt = dt
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],   # x = x + vx*dt
            [0, 1, 0, dt],   # y = y + vy*dt
            [0, 0, 1, 0],    # vx = vx
            [0, 0, 0, 1]     # vy = vy
        ])
        
        # Measurement matrix (measure position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        self.Q = np.diag([
            process_noise_pos,  # Position noise x
            process_noise_pos,  # Position noise y
            process_noise_vel,  # Velocity noise x
            process_noise_vel   # Velocity noise y
        ])
        
        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise
        
        # Tracking metadata
        self.confidence = initial_confidence
        self.last_update = None
        
    def predict(self, dt=None):
        """Predict next state"""
        if dt is not None:
            # Update state transition matrix with new dt
            self.F[0, 2] = dt
            self.F[1, 3] = dt
        
        # Predict state: x = F * x
        self.x = self.F @ self.x
        
        # Predict covariance: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x[:2]  # Return predicted position
    
    def update(self, measurement, confidence=None):
        """Update state with measurement"""
        z = np.array(measurement)
        
        # Innovation (measurement residual): y = z - H * x
        y = z - self.H @ self.x
        
        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state: x = x + K * y
        self.x = self.x + K @ y
        
        # Update covariance: P = (I - K * H) * P
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P
        
        # Update confidence if provided
        if confidence is not None:
            self.confidence = confidence
    
    def get_position(self):
        """Get current position estimate"""
        return self.x[:2]
    
    def get_velocity(self):
        """Get current velocity estimate"""
        return self.x[2:]
    
    def get_state(self):
        """Get full state [x, y, vx, vy]"""
        return self.x.copy()


class HumanTrackKF:
    """Represents a tracked human with Kalman Filter."""
    
    def __init__(self, track_id, position, timestamp, confidence=0.5,
                 process_noise_pos=0.1, process_noise_vel=0.5,
                 measurement_noise=0.3):
        self.id = track_id
        self.kf = KalmanFilter(
            position, 
            confidence,
            process_noise_pos,
            process_noise_vel,
            measurement_noise
        )
        self.last_seen = timestamp
        self.confidence = confidence
        
    def predict(self, current_time):
        """Predict track state (with Zero Delta-T guard)"""
        if self.kf.last_update is not None:
            dt = (current_time - self.kf.last_update).nanoseconds / 1e9
        else:
            dt = 0.1  # Default 10Hz
        
        # Guard: skip prediction if dt is negligibly small to prevent
        # covariance inflation (P += Q) without meaningful state propagation.
        if dt < 1e-4:
            return
        
        # Dynamic Q: discrete white-noise acceleration model.
        # Replaces the fixed diagonal Q so that process noise scales
        # correctly with dt and captures position-velocity cross-covariance.
        noise_accel = 0.5  # Variance of unknown acceleration (m²/s⁴), tuned for pedestrians

        q_pos = (dt**3) / 3.0 * noise_accel
        q_vel = dt * noise_accel
        q_cov = (dt**2) / 2.0 * noise_accel

        self.kf.Q = np.array([
            [q_pos, 0.0,   q_cov, 0.0  ],
            [0.0,   q_pos, 0.0,   q_cov],
            [q_cov, 0.0,   q_vel, 0.0  ],
            [0.0,   q_cov, 0.0,   q_vel]
        ])
        
        self.kf.predict(dt)
        self.kf.last_update = current_time
        
    def update(self, measurement, timestamp, confidence=None):
        """Update track with new measurement.
        
        NOTE: predict() is already called for all tracks in the fusion loop
        before association. Do NOT call self.kf.predict() here — doing so
        would double-predict: state projected by 2*dt and Q injected twice,
        causing severe velocity noise.
        """
        # Measurement update only — no prediction step
        self.kf.update(measurement, confidence)
        
        self.last_seen = timestamp
        self.kf.last_update = timestamp
        
        if confidence is not None:
            self.confidence = confidence
    
    def get_position(self):
        """Get current position"""
        return self.kf.get_position()
    
    def get_velocity(self):
        """Get current velocity"""
        return self.kf.get_velocity()


class HumanFusionKFNode(Node):
    """
    Fuses YOLO and LiDAR human detections with Kalman Filter tracking.
    
    Uses constant velocity model with Kalman Filter for optimal state estimation.
    """
    
    def __init__(self):
        super().__init__('human_fusion_kf_node')
        
        # Parameters
        self.declare_parameter('camera_fov_degrees', 110.0)
        self.declare_parameter('fusion_distance_threshold', 1.0)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('max_track_distance', 2.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # Kalman Filter parameters
        self.declare_parameter('process_noise_pos', 0.1)  # Position process noise (m²)
        self.declare_parameter('process_noise_vel', 0.5)  # Velocity process noise (m²/s²)
        self.declare_parameter('measurement_noise', 0.3)  # Measurement noise (m²)
        
        self.camera_fov_rad = math.radians(self.get_parameter('camera_fov_degrees').value)
        self.fusion_threshold = self.get_parameter('fusion_distance_threshold').value
        self.track_timeout = self.get_parameter('track_timeout_sec').value
        self.max_track_dist = self.get_parameter('max_track_distance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # KF noise parameters
        self.process_noise_pos = self.get_parameter('process_noise_pos').value
        self.process_noise_vel = self.get_parameter('process_noise_vel').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Synchronized subscribers — replaces the old manual timer + cache
        # approach that fused temporally mismatched YOLO / LiDAR frames.
        self.yolo_sub = Subscriber(self, PoseArray, 'tracked_humans/poses')
        self.lidar_sub = Subscriber(self, PoseArray, 'detected_people')
        
        self.ts = ApproximateTimeSynchronizer(
            [self.yolo_sub, self.lidar_sub],
            queue_size=50,
            slop=0.15  # 100 ms tolerance between YOLO and LiDAR stamps
        )
        self.ts.registerCallback(self.fusion_callback)
        
        # Publishers (different topics from non-KF version)
        self.fused_poses_pub = self.create_publisher(
            PoseArray,
            'fused_humans_kf/poses',
            10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            'fused_humans_kf/markers',
            10
        )
        
        # State
        self.tracks = {}  # track_id -> HumanTrackKF
        self.next_track_id = 0
        
        self.get_logger().info(f'Human Fusion KF Node initialized')
        self.get_logger().info(f'  Camera FOV: {self.get_parameter("camera_fov_degrees").value}°')
        self.get_logger().info(f'  Fusion threshold: {self.fusion_threshold}m')
        self.get_logger().info(f'  KF Process noise (pos): {self.process_noise_pos} m²')
        self.get_logger().info(f'  KF Process noise (vel): {self.process_noise_vel} m²/s²')
        self.get_logger().info(f'  KF Measurement noise: {self.measurement_noise} m²')
        self.get_logger().info(f'  Publishing to: fused_humans_kf/poses, fused_humans_kf/markers')
    
    def get_robot_pose_and_yaw(self):
        """Get robot's position and heading in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
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
        """Check if a point is within the robot's triangular camera FOV."""
        to_point = point - robot_pos
        distance = np.linalg.norm(to_point)
        
        if distance < 0.1:
            return True
        
        angle_to_point = math.atan2(to_point[1], to_point[0])
        angle_diff = self.normalize_angle(angle_to_point - robot_yaw)
        
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
    
    def fusion_callback(self, yolo_msg, lidar_msg):
        """Main fusion loop with Kalman Filter tracking.
        
        Called by ApproximateTimeSynchronizer with temporally matched
        YOLO and LiDAR PoseArray messages (slop ≤ 100 ms).
        """
        robot_pos, robot_yaw = self.get_robot_pose_and_yaw()
        if robot_pos is None:
            return
        
        # Extract detections and confidence
        yolo_points = np.array([[p.position.x, p.position.y] for p in yolo_msg.poses])
        yolo_conf = np.array([
            p.position.z if (p.position.z > 0.0) else 0.7
            for p in yolo_msg.poses
        ])
        
        lidar_points = np.array([[p.position.x, p.position.y] for p in lidar_msg.poses])
        lidar_conf = np.array([
            p.position.z if (p.position.z > 0.0) else 0.8
            for p in lidar_msg.poses
        ])
        
        fused_positions = []
        fused_confidences = []
        # Use the synchronized sensor timestamp instead of wall-clock
        current_time = rclpy.time.Time.from_msg(yolo_msg.header.stamp)
        
        # ======== STEP A: Optimal YOLO↔LiDAR Association (Hungarian) ========
        # Build a full Euclidean distance cost matrix (rows=YOLO, cols=LiDAR)
        # and solve the global optimal assignment to prevent greedy mis-pairing
        # when two humans stand close together.
        matched_yolo = set()
        matched_lidar = set()
        
        n_yolo = len(yolo_points)
        n_lidar = len(lidar_points)
        
        if n_yolo > 0 and n_lidar > 0:
            # Cost matrix: pairwise Euclidean distances
            cost_matrix = np.linalg.norm(
                yolo_points[:, np.newaxis, :] - lidar_points[np.newaxis, :, :],
                axis=2
            )  # shape (n_yolo, n_lidar)
            
            # Solve optimal assignment via Hungarian algorithm
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            # Accept only pairs within the fusion distance threshold
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < self.fusion_threshold:
                    # Confidence-weighted fusion (confidence in position.z)
                    y_conf = float(yolo_conf[r])
                    l_conf = float(lidar_conf[c])
                    
                    fused_point = (
                        yolo_points[r] * y_conf + lidar_points[c] * l_conf
                    ) / (y_conf + l_conf)
                    fused_confidence = (y_conf + l_conf) / 2.0
                    
                    fused_positions.append(fused_point)
                    fused_confidences.append(fused_confidence)
                    matched_yolo.add(r)
                    matched_lidar.add(c)
        
        # Unmatched YOLO detections → camera-only tracking
        for idx in range(n_yolo):
            if idx not in matched_yolo:
                fused_positions.append(yolo_points[idx])
                fused_confidences.append(float(yolo_conf[idx]))
        
        # ======== STEP B: Unmatched LiDAR → blind-spot coverage ========
        for i in range(n_lidar):
            if i not in matched_lidar:
                # Only keep LiDAR detections that are outside the camera FOV;
                # inside-FOV unmatched LiDAR is discarded (no camera corroboration).
                if not self.is_in_camera_fov(lidar_points[i], robot_pos, robot_yaw):
                    fused_positions.append(lidar_points[i])
                    fused_confidences.append(float(lidar_conf[i]))
        
        # ======== STEP C: Kalman Filter Tracking ========
        fused_positions = np.array(fused_positions) if fused_positions else np.empty((0, 2))
        fused_confidences = np.array(fused_confidences) if fused_confidences else np.empty(0)
        
        # Predict all existing tracks
        for track in self.tracks.values():
            track.predict(current_time)
        
        if len(fused_positions) > 0:
            track_ids = list(self.tracks.keys())
            
            if len(track_ids) > 0:
                # Build cost matrix using predicted positions
                cost_matrix = np.zeros((len(track_ids), len(fused_positions)))
                for i, tid in enumerate(track_ids):
                    track_pos = self.tracks[tid].get_position()
                    for j, fused_pos in enumerate(fused_positions):
                        cost_matrix[i, j] = np.linalg.norm(track_pos - fused_pos)
                
                # Hungarian algorithm
                row_ind, col_ind = linear_sum_assignment(cost_matrix)
                
                # Update matched tracks
                matched_fused = set()
                for i, j in zip(row_ind, col_ind):
                    if cost_matrix[i, j] < self.max_track_dist:
                        tid = track_ids[i]
                        self.tracks[tid].update(
                            fused_positions[j],
                            current_time,
                            confidence=fused_confidences[j]
                        )
                        matched_fused.add(j)
                
                # Create new tracks for unmatched detections
                for j, fused_pos in enumerate(fused_positions):
                    if j not in matched_fused:
                        self.tracks[self.next_track_id] = HumanTrackKF(
                            self.next_track_id,
                            fused_pos,
                            current_time,
                            confidence=fused_confidences[j],
                            process_noise_pos=self.process_noise_pos,
                            process_noise_vel=self.process_noise_vel,
                            measurement_noise=self.measurement_noise
                        )
                        self.next_track_id += 1
            else:
                # No existing tracks, create new ones
                for i, fused_pos in enumerate(fused_positions):
                    self.tracks[self.next_track_id] = HumanTrackKF(
                        self.next_track_id,
                        fused_pos,
                        current_time,
                        confidence=fused_confidences[i],
                        process_noise_pos=self.process_noise_pos,
                        process_noise_vel=self.process_noise_vel,
                        measurement_noise=self.measurement_noise
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
        self.publish_fused_poses(current_time)
        self.publish_markers(current_time)
    
    def publish_fused_poses(self, timestamp):
        """Publish fused human poses with KF estimates."""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp.to_msg()
        pose_array.header.frame_id = self.map_frame
        
        for track in self.tracks.values():
            pose = Pose()
            pos = track.get_position()
            vel = track.get_velocity()
            
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            # Encode confidence in position.z (2D tracking, Z unused)
            pose.position.z = track.confidence
            
            # Encode velocity magnitude in orientation.w
            vel_mag = np.linalg.norm(vel)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = min(vel_mag, 5.0)
            
            pose_array.poses.append(pose)
        
        self.fused_poses_pub.publish(pose_array)
    
    def publish_markers(self, timestamp):
        """Publish visualization markers with KF-tracked humans."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for track in self.tracks.values():
            pos = track.get_position()
            vel = track.get_velocity()
            
            # Circle circumference marker (LINE_STRIP)
            circle = Marker()
            circle.header.stamp = timestamp.to_msg()
            circle.header.frame_id = self.map_frame
            circle.ns = 'fused_humans_kf'
            circle.id = marker_id
            marker_id += 1
            circle.type = Marker.LINE_STRIP
            circle.action = Marker.ADD

            circle.pose.position.x = float(pos[0])
            circle.pose.position.y = float(pos[1])
            circle.pose.position.z = 0.0
            circle.pose.orientation.w = 1.0

            circle.scale.x = 0.05  # Line width in metres

            radius = 0.4  # Person bounding radius (metres)
            n_pts = 36
            for i in range(n_pts + 1):           # +1 closes the loop
                angle = 2.0 * math.pi * i / n_pts
                p = Point()
                p.x = radius * math.cos(angle)
                p.y = radius * math.sin(angle)
                p.z = 1.0                        # Waist height for visibility
                circle.points.append(p)

            circle.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange for KF
            circle.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(circle)
            
            # Velocity arrow
            vel_mag = np.linalg.norm(vel)
            if vel_mag > 0.1:
                arrow = Marker()
                arrow.header.stamp = timestamp.to_msg()
                arrow.header.frame_id = self.map_frame
                arrow.ns = 'velocities_kf'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                start = Point(x=float(pos[0]), y=float(pos[1]), z=0.1)
                end = Point(x=float(pos[0] + vel[0]), y=float(pos[1] + vel[1]), z=0.1)
                arrow.points = [start, end]
                
                arrow.scale.x = 0.1
                arrow.scale.y = 0.15
                arrow.scale.z = 0.2
                
                arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
                arrow.lifetime = Duration(seconds=0.5).to_msg()
                marker_array.markers.append(arrow)
            
            # Text label
            text = Marker()
            text.header.stamp = timestamp.to_msg()
            text.header.frame_id = self.map_frame
            text.ns = 'labels_kf'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = 2.0
            text.pose.orientation.w = 1.0
            
            text.text = f'KF-ID:{track.id}\n{vel_mag:.2f}m/s\nConf:{track.confidence:.2f}'
            text.scale.z = 0.3
            
            # Color by confidence
            if track.confidence >= 0.7:
                text.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif track.confidence >= 0.5:
                text.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                text.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            
            text.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(text)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HumanFusionKFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
