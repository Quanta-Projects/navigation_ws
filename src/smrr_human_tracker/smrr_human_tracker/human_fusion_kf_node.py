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
from enum import Enum


class TrackStatus(Enum):
    TENTATIVE = 0  # Still in Logic Initiator buffer
    NEW = 1        # Just confirmed, first frame published
    MATCHED = 2    # Seen and updated this frame
    MISSED = 3     # Not seen this frame, coasting
    DELETED = 4    # Marked for physical removal


class ConstantMotionModel:
    """
    SPENCER CV (Constant Velocity) Model.
    Assumes linear, steady movement.
    Translated from constant_motion_model.h
    """
    def __init__(self, process_noise):
        self.q = process_noise
        self.dim = 5

    def get_A(self, dt, state):
        A = np.eye(self.dim)
        A[0, 2] = dt
        A[1, 3] = dt
        return A

    def get_Q(self, dt):
        Q = np.zeros((self.dim, self.dim))
        q = self.q
        Q[0, 0] = Q[1, 1] = (dt**3) * q / 3.0
        Q[0, 2] = Q[1, 3] = Q[2, 0] = Q[3, 1] = 0.5 * (dt**2) * q
        Q[2, 2] = Q[3, 3] = dt * q
        # Omega (turn rate) is completely un-modeled and gets zero noise in CV
        return Q


class CoordinatedTurnMotionModel:
    """
    SPENCER CT (Coordinated Turn) Model.
    Nonlinear EKF Jacobian for curved paths.
    Translated from coordinated_turn_motion_model.h
    """
    def __init__(self, process_noise, turn_rate_variance):
        self.q = process_noise
        self.q_omega = turn_rate_variance
        self.dim = 5

    def get_A(self, dt, state):
        omega = state[4]
        A = np.eye(self.dim)
        # Prevent division by zero if turn rate is negligible
        if abs(omega) > 1e-4:
            A[0, 2] = np.sin(omega * dt) / omega
            A[0, 3] = -(1.0 - np.cos(omega * dt)) / omega
            A[1, 2] = (1.0 - np.cos(omega * dt)) / omega
            A[1, 3] = np.sin(omega * dt) / omega
            A[2, 2] = np.cos(omega * dt)
            A[2, 3] = -np.sin(omega * dt)
            A[3, 2] = np.sin(omega * dt)
            A[3, 3] = np.cos(omega * dt)
        else:
            # Fallback to CV Jacobian if no active rotation
            A[0, 2] = dt
            A[1, 3] = dt
        return A

    def get_Q(self, dt):
        Q = np.zeros((self.dim, self.dim))
        q = self.q
        Q[0, 0] = Q[1, 1] = (dt**3) * q / 3.0
        Q[0, 2] = Q[1, 3] = Q[2, 0] = Q[3, 1] = 0.5 * (dt**2) * q
        Q[2, 2] = Q[3, 3] = dt * q
        Q[4, 4] = dt * self.q_omega
        return Q


class BrownianMotionModel:
    """
    SPENCER BM (Brownian Motion) Model.
    Assumes zero velocity (standing still).
    Translated from brownian_motion_model.h
    """
    def __init__(self, process_noise):
        self.q = process_noise
        self.dim = 5

    def get_A(self, dt, state):
        A = np.eye(self.dim)
        # Sever the velocity coupling: position does not change based on velocity
        A[0, 2] = A[1, 3] = 0.0
        # Decay velocity to zero quickly so the tracker stops predicting forward movement
        A[2, 2] = A[3, 3] = 0.0
        A[4, 4] = 0.0
        return A

    def get_Q(self, dt):
        Q = np.zeros((self.dim, self.dim))
        Q[0, 0] = Q[1, 1] = dt * self.q
        # No velocity process noise, cementing the "standing still" assumption
        return Q


class IMMFilter:
    """3-Model IMM Filter mirroring SPENCER architecture."""
    def __init__(self, initial_pos, measurement_noise=0.05):
        # Initialize the 3 SPENCER Motion Models with Realistic Human Dynamics
        self.models = [
            ConstantMotionModel(process_noise=1.5),               # IMM0: CV (Dynamic walking)
            CoordinatedTurnMotionModel(1.5, 2.0),                 # IMM1: CT (Turning)
            BrownianMotionModel(process_noise=0.1)                # IMM2: BM (Firmly stationary)
        ]
        self.num_models = 3

        # SPENCER tuning_imm.yaml Markov Transition Matrix
        self.trans_prob = np.array([
            [0.574, 0.426, 0.250],
            [0.571, 0.429, 0.250],
            [0.250, 0.250, 0.250]
        ])

        # Initialize hypotheses probabilities (equally weighted)
        self.probs = np.ones(self.num_models) / self.num_models

        # State vectors: [x, y, vx, vy, omega]
        self.x = [np.array([initial_pos[0], initial_pos[1], 0.0, 0.0, 0.0]) for _ in range(self.num_models)]

        # Covariance matrices
        self.P = [np.eye(5) * measurement_noise for _ in range(self.num_models)]
        for P in self.P:
            P[2, 2] = P[3, 3] = 2.0  # Initial velocity uncertainty (cosvxx/cosvyy)
            P[4, 4] = 0.8            # Initial turn rate uncertainty (cosw)

        # Measurement matrix (we only measure x, y)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0]
        ])
        self.R = np.eye(2) * measurement_noise

        # Mixed (Output) State
        self.x_mixed = self.x[0].copy()
        self.P_mixed = self.P[0].copy()

        self.last_update = None

    def predict(self, dt):
        """Execute IMM Prediction step (Mixing -> Predict -> Mix)."""
        if dt < 1e-4:
            return self.x_mixed[:2]

        # 1. Compute Mixing Probabilities (c_j normalizer)
        c = np.dot(self.trans_prob.T, self.probs)
        # Prevent division by zero
        c[c == 0] = 1e-9
        mu_mix = (self.trans_prob * self.probs[:, None]) / c[None, :]

        # 2. Compute Mixed Initial States (for each model)
        x_0j = [np.zeros(5) for _ in range(self.num_models)]
        P_0j = [np.zeros((5, 5)) for _ in range(self.num_models)]

        for j in range(self.num_models):
            for i in range(self.num_models):
                x_0j[j] += mu_mix[i, j] * self.x[i]

            for i in range(self.num_models):
                diff = self.x[i] - x_0j[j]
                P_0j[j] += mu_mix[i, j] * (self.P[i] + np.outer(diff, diff))

        # 3. Independent EKF Predictions
        for i in range(self.num_models):
            A = self.models[i].get_A(dt, x_0j[i])
            Q = self.models[i].get_Q(dt)
            self.x[i] = A @ x_0j[i]
            self.P[i] = A @ P_0j[i] @ A.T + Q

        self._update_mixed_estimate()
        return self.x_mixed[:2]

    def update(self, measurement, confidence=None):
        """Execute IMM Update step (Update -> Likelihood -> Bayesian Mode Prob)."""
        likelihoods = np.zeros(self.num_models)
        z = np.array(measurement)

        # 4. Independent EKF Updates & Likelihood calculation
        for i in range(self.num_models):
            y = z - self.H @ self.x[i]
            S = self.H @ self.P[i] @ self.H.T + self.R
            try:
                S_inv = np.linalg.inv(S)
                K = self.P[i] @ self.H.T @ S_inv
                self.x[i] = self.x[i] + K @ y
                self.P[i] = (np.eye(5) - K @ self.H) @ self.P[i]

                # Gaussian Likelihood calculation
                d = y.T @ S_inv @ y
                det_S = max(np.linalg.det(S), 1e-9)
                likelihoods[i] = np.exp(-d / 2.0) / np.sqrt(det_S)
            except np.linalg.LinAlgError:
                likelihoods[i] = 1e-9  # Singular matrix fallback

        # 5. Bayesian Mode Probability Update
        c = np.dot(self.trans_prob.T, self.probs)
        raw_probs = likelihoods * c
        sum_probs = np.sum(raw_probs)

        if sum_probs > 0:
            self.probs = raw_probs / sum_probs
        else:
            # Fallback if likelihoods crash
            self.probs = np.ones(self.num_models) / self.num_models

        self._update_mixed_estimate()

    def _update_mixed_estimate(self):
        """6. Fuse all hypotheses into a final mixed estimate."""
        self.x_mixed = sum(self.probs[i] * self.x[i] for i in range(self.num_models))
        self.P_mixed = sum(
            self.probs[i] * (
                self.P[i] + np.outer(self.x[i] - self.x_mixed, self.x[i] - self.x_mixed)
            )
            for i in range(self.num_models)
        )

    def get_position(self):
        return self.x_mixed[:2]

    def get_velocity(self):
        return self.x_mixed[2:4]

    def mahalanobis_distance(self, z):
        """Global Mahalanobis distance using the mixed state."""
        z = np.array(z)
        y = z - self.H @ self.x_mixed
        S = self.H @ self.P_mixed @ self.H.T + self.R
        try:
            return np.sqrt(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return float('inf')


class HumanTrackKF:
    """Represents a tracked human with an IMM Filter."""

    def __init__(self, track_id, position, timestamp, confidence=0.5,
                 measurement_noise=0.3, visually_confirmed=False):
        self.id = track_id
        # Instantiate the new IMM Filter
        self.kf = IMMFilter(position, measurement_noise)
        self.last_seen = timestamp
        self.confidence = confidence

        # SPENCER Lifecycle State Machine
        self.status = TrackStatus.TENTATIVE
        self.consecutive_hits = 1
        self.consecutive_misses = 0
        self.total_matches = 1

        # Gating
        self.visually_confirmed = visually_confirmed
        self.static_time_start = None
        self.is_static_false_positive = False

    def predict(self, current_time):
        """Predict track state using IMM."""
        if self.kf.last_update is not None:
            # Calculate time since the last actual measurement update
            dt = (current_time - self.kf.last_update).nanoseconds / 1e9
        else:
            dt = 0.1  # Default 10Hz

        if dt < 1e-4:
            return

        # Execute IMM prediction
        self.kf.predict(dt)
        # DO NOT update self.kf.last_update here! Only update it when a true measurement arrives.

    def update(self, measurement, timestamp, confidence=None, visually_confirmed=False):
        """Update track with new measurement (MATCHED)."""
        self.kf.update(measurement)  # IMM update handles likelihoods internally
        self.last_seen = timestamp
        self.kf.last_update = timestamp

        if confidence is not None:
            self.confidence = confidence

        self.visually_confirmed = visually_confirmed

        # SPENCER State Machine Transition: MATCHED
        self.consecutive_misses = 0
        self.consecutive_hits += 1
        self.total_matches += 1

        # Logic Initiator: Promote TENTATIVE to NEW/MATCHED after hit streak
        hit_streak_required = 3 if not visually_confirmed else 1
        if self.status == TrackStatus.TENTATIVE and self.consecutive_hits >= hit_streak_required:
            self.status = TrackStatus.NEW
        elif self.status == TrackStatus.NEW or self.status == TrackStatus.MISSED:
            self.status = TrackStatus.MATCHED

        # Velocity-Based Static Gating with Visual Immunity
        vel_mag = np.linalg.norm(self.kf.get_velocity())
        if self.visually_confirmed:
            self.static_time_start = None
            self.is_static_false_positive = False
        elif vel_mag < 0.1 and self.status != TrackStatus.TENTATIVE:
            if self.static_time_start is None:
                self.static_time_start = timestamp
            else:
                time_stationary = (timestamp - self.static_time_start).nanoseconds / 1e9
                if time_stationary > 2.0:
                    self.is_static_false_positive = True
                    self.confidence = min(self.confidence, 0.2)
        else:
            self.static_time_start = None
            self.is_static_false_positive = False

    def mark_missed(self, current_time):
        """Transition track to MISSED and increment coasting counters."""
        if self.status != TrackStatus.TENTATIVE:
            self.status = TrackStatus.MISSED
        self.consecutive_hits = 0
        self.consecutive_misses += 1

        # Advance the internal clock during coasting so dt doesn't explode upon re-association
        self.kf.last_update = current_time
    
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
        self.declare_parameter('fusion_distance_threshold', 0.4)
        self.declare_parameter('track_timeout_sec', 1.0)
        self.declare_parameter('max_track_distance', 2.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # IMM Filter parameters
        self.declare_parameter('measurement_noise', 0.05)  # 5cm LiDAR tracking variance
        
        self.camera_fov_rad = math.radians(self.get_parameter('camera_fov_degrees').value)
        self.fusion_threshold = self.get_parameter('fusion_distance_threshold').value
        self.track_timeout = self.get_parameter('track_timeout_sec').value
        self.max_track_dist = self.get_parameter('max_track_distance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # IMM noise parameter
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
            slop=0.03  # Tightened to 30 ms tolerance
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
        self.get_logger().info(f'  IMM Measurement noise: {self.measurement_noise} m²')
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
        fused_visual_flags = []
        # Use the monotonic LiDAR hardware timestamp to guarantee accurate dt integration
        current_time = rclpy.time.Time.from_msg(lidar_msg.header.stamp)
        
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
                    fused_visual_flags.append(True)
                    matched_yolo.add(r)
                    matched_lidar.add(c)
        
        # Unmatched YOLO detections → camera-only tracking
        for idx in range(n_yolo):
            if idx not in matched_yolo:
                fused_positions.append(yolo_points[idx])
                fused_confidences.append(float(yolo_conf[idx]))
                fused_visual_flags.append(True)
        
        # ======== STEP B: Unmatched LiDAR → blind-spot coverage ========
        for i in range(n_lidar):
            if i not in matched_lidar:
                # Only keep LiDAR detections that are outside the camera FOV;
                # inside-FOV unmatched LiDAR is discarded (no camera corroboration).
                if not self.is_in_camera_fov(lidar_points[i], robot_pos, robot_yaw):
                    fused_positions.append(lidar_points[i])
                    fused_confidences.append(float(lidar_conf[i]))
                    fused_visual_flags.append(False)
        
        # ======== STEP C: Kalman Filter Tracking ========
        fused_positions = np.array(fused_positions) if fused_positions else np.empty((0, 2))
        fused_confidences = np.array(fused_confidences) if fused_confidences else np.empty(0)
        
        # Predict all existing tracks
        for track in self.tracks.values():
            track.predict(current_time)
        
        if len(fused_positions) > 0:
            track_ids = list(self.tracks.keys())

            if len(track_ids) > 0:
                # Build cost matrix using Mahalanobis distance instead of Euclidean
                cost_matrix = np.zeros((len(track_ids), len(fused_positions)))
                for i, tid in enumerate(track_ids):
                    for j, fused_pos in enumerate(fused_positions):
                        m_dist = self.tracks[tid].kf.mahalanobis_distance(fused_pos)
                        cost_matrix[i, j] = m_dist

                # Reset all existing published tracks to MISSED before association
                for track in self.tracks.values():
                    if track.status != TrackStatus.TENTATIVE:
                        track.status = TrackStatus.MISSED

                # Hungarian algorithm
                row_ind, col_ind = linear_sum_assignment(cost_matrix)

                # Update matched tracks (threshold tuned for Mahalanobis, ~2 std devs)
                matched_fused = set()
                mahalanobis_threshold = 2.0

                for i, j in zip(row_ind, col_ind):
                    if cost_matrix[i, j] < mahalanobis_threshold:
                        tid = track_ids[i]
                        self.tracks[tid].update(
                            fused_positions[j],
                            current_time,
                            confidence=fused_confidences[j],
                            visually_confirmed=fused_visual_flags[j]
                        )
                        matched_fused.add(j)

                # Mark unmatched tracks as missed to begin coasting
                for tid, track in self.tracks.items():
                    if track.status == TrackStatus.MISSED and tid not in [track_ids[i] for i in row_ind]:
                        track.mark_missed(current_time)
                    elif track.status == TrackStatus.TENTATIVE and tid not in [track_ids[i] for i in row_ind]:
                        track.mark_missed(current_time)
                
                # Create new tracks for unmatched detections
                for j, fused_pos in enumerate(fused_positions):
                    if j not in matched_fused:
                        self.tracks[self.next_track_id] = HumanTrackKF(
                            self.next_track_id,
                            fused_pos,
                            current_time,
                            confidence=fused_confidences[j],
                            measurement_noise=self.measurement_noise,
                            visually_confirmed=fused_visual_flags[j]
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
                        measurement_noise=self.measurement_noise,
                        visually_confirmed=fused_visual_flags[i]
                    )
                    self.next_track_id += 1
        
        # ======== SPENCER BasicOcclusionManager Deletion Logic ========
        stale_tracks = []
        for tid, track in self.tracks.items():
            # TENTATIVE tracks are deleted quickly if they fail to initiate
            if track.status == TrackStatus.TENTATIVE and track.consecutive_misses > 2:
                stale_tracks.append(tid)
            # Confirmed tracks undergo maturity checks
            elif track.status == TrackStatus.MISSED:
                is_mature = track.total_matches >= 20  # ~2 seconds of solid tracking at 10Hz
                miss_limit = 30 if is_mature else 5    # Mature tracks coast for 3s, fresh tracks 0.5s

                if track.consecutive_misses > miss_limit:
                    stale_tracks.append(tid)

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
            if track.confidence < 0.3 or track.status == TrackStatus.TENTATIVE:
                continue  # Skip gated static false positives and unconfirmed tentative tracks

            pose = Pose()
            pos = track.get_position()
            vel = track.get_velocity()
            
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            # Encode confidence in position.z (2D tracking, Z unused)
            pose.position.z = track.confidence
            
            # Encode velocity vector in orientation.x/y (for evaluator) and
            # magnitude in orientation.w (for downstream consumers).
            vel_mag = np.linalg.norm(vel)
            pose.orientation.x = float(vel[0])   # vx  — read by evaluate_kf_only.py
            pose.orientation.y = float(vel[1])   # vy  — read by evaluate_kf_only.py
            pose.orientation.z = 0.0
            pose.orientation.w = min(vel_mag, 5.0)
            
            pose_array.poses.append(pose)
        
        self.fused_poses_pub.publish(pose_array)
    
    def publish_markers(self, timestamp):
        """Publish visualization markers with KF-tracked humans."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for track in self.tracks.values():
            if track.confidence < 0.3 or track.status == TrackStatus.TENTATIVE:
                continue  # Skip gated static false positives and unconfirmed tentative tracks

            pos = track.get_position()
            vel = track.get_velocity()

            # Cylinder marker
            cylinder = Marker()
            cylinder.header.stamp = timestamp.to_msg()
            cylinder.header.frame_id = self.map_frame
            cylinder.ns = 'fused_humans_kf'
            cylinder.id = marker_id
            marker_id += 1
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            
            cylinder.pose.position.x = float(pos[0])
            cylinder.pose.position.y = float(pos[1])
            cylinder.pose.position.z = 0.9
            cylinder.pose.orientation.w = 1.0
            
            cylinder.scale.x = 0.4
            cylinder.scale.y = 0.4
            cylinder.scale.z = 1.8
            
            cylinder.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange for KF
            cylinder.lifetime = Duration(seconds=0.5).to_msg()
            marker_array.markers.append(cylinder)
            
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