#!/usr/bin/env python3
"""
Human Detection Fusion Node with IMM (Interacting Multiple Model) Filter

Fuses vision-based (YOLO) and LiDAR-based (DR-SPAAM) human detections
with a 2-model IMM filter: Constant Position (CP) and Constant Velocity
(CV).  Includes Mahalanobis outlier gating and dynamic measurement noise.

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


class IMMFilter:
    """
    Interacting Multiple Model (IMM) Filter with 2 motion models.

    Models:
      0 — Constant Position (CP): human standing still
      1 — Constant Velocity (CV): human walking steadily

    State:       [x, y, vx, vy]  (4D)
    Measurement: [x, y]          (2D)
    """

    def __init__(self, initial_position, initial_confidence=0.5,
                 measurement_noise=0.3):
        """
        Initialize the 2-model IMM filter.

        Args:
            initial_position: [x, y] initial position
            initial_confidence: Detection confidence (0-1)
            measurement_noise: Base measurement noise variance (m²)
        """
        self.n_models = 2
        self.state_dim = 4   # [x, y, vx, vy]
        self.meas_dim = 2    # [x, y]

        # ---------- Observation matrix (shared by all models) ---------
        # We only observe position; velocity is hidden.
        self.H = np.zeros((self.meas_dim, self.state_dim))
        self.H[0, 0] = 1.0  # observe x
        self.H[1, 1] = 1.0  # observe y

        # ---------- Fast-Init P0 ----------
        P0 = np.diag([0.3, 0.3, 2.0, 2.0])

        # ---------- Per-model states and covariances ----------
        x0 = np.array([
            initial_position[0], initial_position[1],
            0.0, 0.0   # velocity unknown
        ])
        self.x_models = [x0.copy() for _ in range(self.n_models)]
        self.P_models = [P0.copy() for _ in range(self.n_models)]

        # ---------- Combined (output) state ----------
        self.x = x0.copy()
        self.P = P0.copy()

        # ---------- Mode probabilities: CP, CV ----------
        self.mu = np.array([0.2, 0.8])

        # ---------- Markov mode transition matrix ----------
        # Rows = from-model, Cols = to-model.  High diagonal = aggressive
        # mode commitment to prevent CP from dragging CV backward.
        self.M = np.array([
            [0.98, 0.02],   # CP -> CP / CV
            [0.05, 0.95]    # CV -> CP / CV
        ])

        # Base measurement noise (scaled dynamically in update())
        self.base_R = measurement_noise

        # Tracking metadata
        self.confidence = initial_confidence
        self.last_update = None

    # ------------------------------------------------------------------
    #                     TRANSITION MATRICES (F)                       
    # ------------------------------------------------------------------

    @staticmethod
    def _build_F_cp(dt):
        """Constant Position: position stays, velocity -> 0."""
        F = np.zeros((4, 4))
        F[0, 0] = 1.0
        F[1, 1] = 1.0
        # velocity rows are zero -> forced to 0
        return F

    @staticmethod
    def _build_F_cv(dt):
        """Constant Velocity: x += v*dt, velocity stays."""
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    # ------------------------------------------------------------------
    #                        PROCESS NOISE (Q)                          
    # ------------------------------------------------------------------

    @staticmethod
    def _build_Q_cp(dt):
        """CP process noise: very small position jitter, no v noise."""
        Q = np.zeros((4, 4))
        q_pos = 0.01 * dt   # near-zero positional noise
        Q[0, 0] = q_pos
        Q[1, 1] = q_pos
        return Q

    @staticmethod
    def _build_Q_cv(dt):
        """CV process noise: white-noise acceleration model for velocity."""
        sigma_a = 0.5   # acceleration variance (m^2/s^4)
        dt2 = dt * dt
        dt3 = dt2 * dt
        Q = np.zeros((4, 4))
        # Position block
        Q[0, 0] = dt3 / 3.0 * sigma_a
        Q[1, 1] = dt3 / 3.0 * sigma_a
        # Position-velocity cross
        Q[0, 2] = dt2 / 2.0 * sigma_a
        Q[2, 0] = dt2 / 2.0 * sigma_a
        Q[1, 3] = dt2 / 2.0 * sigma_a
        Q[3, 1] = dt2 / 2.0 * sigma_a
        # Velocity block
        Q[2, 2] = dt * sigma_a
        Q[3, 3] = dt * sigma_a
        return Q

    # ------------------------------------------------------------------
    #                            PREDICT                                
    # ------------------------------------------------------------------

    def predict(self, dt):
        """
        IMM predict step: mix models -> predict each model forward.

        1. Compute mixing probabilities from the Markov transition matrix.
        2. Mix per-model states and covariances.
        3. Propagate each mixed state through its own F and Q.
        4. Update predicted mode probabilities.
        5. Combine into single output estimate.
        """
        # --- 1. Build transition & process-noise matrices ---
        F_list = [self._build_F_cp(dt), self._build_F_cv(dt)]
        Q_list = [self._build_Q_cp(dt), self._build_Q_cv(dt)]

        # --- 2. Compute mixing probabilities ---
        # c_bar[j] = sum_i( M[i,j] * mu[i] )  --> predicted prob of model j
        c_bar = self.M.T @ self.mu                    # shape (2,)
        c_bar = np.maximum(c_bar, 1e-30)              # prevent div-by-zero

        # mu_mix[i,j] = M[i,j] * mu[i] / c_bar[j]
        # "probability that model j was in model i at the previous step"
        mu_mix = np.zeros((self.n_models, self.n_models))
        for i in range(self.n_models):
            for j in range(self.n_models):
                mu_mix[i, j] = self.M[i, j] * self.mu[i] / c_bar[j]

        # --- 3. Mix states and covariances for each target model j ---
        x_mixed = []
        P_mixed = []
        for j in range(self.n_models):
            # Mixed state: weighted sum of all model states
            x_j = np.zeros(self.state_dim)
            for i in range(self.n_models):
                x_j += mu_mix[i, j] * self.x_models[i]

            # Mixed covariance: weighted sum of (P_i + spread term)
            P_j = np.zeros((self.state_dim, self.state_dim))
            for i in range(self.n_models):
                dx = self.x_models[i] - x_j
                P_j += mu_mix[i, j] * (self.P_models[i] + np.outer(dx, dx))

            x_mixed.append(x_j)
            P_mixed.append(P_j)

        # --- 4. Predict each model forward ---
        for j in range(self.n_models):
            self.x_models[j] = F_list[j] @ x_mixed[j]
            self.P_models[j] = F_list[j] @ P_mixed[j] @ F_list[j].T + Q_list[j]

        # Update mode probabilities to predicted values
        self.mu = c_bar

        # --- 5. Combine for output (pre-update combined estimate) ---
        self._combine()

    # ------------------------------------------------------------------
    #                            UPDATE                                 
    # ------------------------------------------------------------------

    def update(self, measurement, confidence=0.5, distance=0.0):
        """
        IMM update step with dynamic R, Mahalanobis gating,
        per-model Kalman updates, and probability recombination.

        Args:
            measurement: [x, y] observed position
            confidence:  Detection confidence (0-1)
            distance:    Range from robot to detection (metres)
        """
        z = np.array(measurement)

        # --- 1. Dynamic measurement noise R ---
        # Higher noise for low-confidence or far-away detections.
        # Floor of 0.2 prevents over-confident R from causing jitter.
        dynamic_r = max(0.2, self.base_R * (2.0 - confidence) * (1.0 + 0.1 * distance))
        R = np.eye(self.meas_dim) * dynamic_r

        # --- 2. Mahalanobis gating on the COMBINED predicted state ---
        # Reject gross outliers before they can corrupt any model.
        y_comb = z - self.H @ self.x
        S_comb = self.H @ self.P @ self.H.T + R
        try:
            S_comb_inv = np.linalg.inv(S_comb)
        except np.linalg.LinAlgError:
            return   # Singular covariance -> skip update

        mahal_sq = float(y_comb.T @ S_comb_inv @ y_comb)

        if mahal_sq > 50.0:
            # Near-disabled gate — only reject catastrophic outliers (NaN, 1000m).
            return

        # --- 3. Per-model Kalman update & likelihood computation ---
        likelihoods = np.zeros(self.n_models)

        for j in range(self.n_models):
            x_j = self.x_models[j]
            P_j = self.P_models[j]

            # Innovation
            y_j = z - self.H @ x_j

            # Innovation covariance
            S_j = self.H @ P_j @ self.H.T + R

            try:
                S_j_inv = np.linalg.inv(S_j)
            except np.linalg.LinAlgError:
                likelihoods[j] = 1e-30
                continue

            # Kalman gain
            K_j = P_j @ self.H.T @ S_j_inv

            # State update
            self.x_models[j] = x_j + K_j @ y_j

            # Covariance update (Joseph form for numerical stability)
            I_KH = np.eye(self.state_dim) - K_j @ self.H
            self.P_models[j] = I_KH @ P_j @ I_KH.T + K_j @ R @ K_j.T

            # Gaussian likelihood for model probability update
            det_S = np.linalg.det(S_j)
            if det_S < 1e-30:
                likelihoods[j] = 1e-30
            else:
                exponent = -0.5 * float(y_j.T @ S_j_inv @ y_j)
                likelihoods[j] = np.exp(exponent) / np.sqrt(
                    (2.0 * np.pi) ** self.meas_dim * det_S
                )
                likelihoods[j] = max(likelihoods[j], 1e-30)

        # --- 4. Update mode probabilities ---
        self.mu = self.mu * likelihoods
        mu_sum = np.sum(self.mu)
        if mu_sum > 1e-30:
            self.mu /= mu_sum
        else:
            self.mu = np.array([0.2, 0.8])   # reset on degenerate case

        # --- 5. Combine models into single output estimate ---
        self._combine()

        # Update confidence
        self.confidence = confidence

    # ------------------------------------------------------------------
    #                         COMBINATION                               
    # ------------------------------------------------------------------

    def _combine(self):
        """Combine per-model estimates into a single output state & covariance."""
        self.x = np.zeros(self.state_dim)
        for j in range(self.n_models):
            self.x += self.mu[j] * self.x_models[j]

        self.P = np.zeros((self.state_dim, self.state_dim))
        for j in range(self.n_models):
            dx = self.x_models[j] - self.x
            self.P += self.mu[j] * (self.P_models[j] + np.outer(dx, dx))

    # ------------------------------------------------------------------
    #                          ACCESSORS                                
    # ------------------------------------------------------------------

    def get_position(self):
        """Get current position estimate [x, y]."""
        return self.x[:2]

    def get_velocity(self):
        """Get current velocity estimate [vx, vy]."""
        return self.x[2:4]

    def get_state(self):
        """Get full state [x, y, vx, vy]."""
        return self.x.copy()

    def get_dominant_mode(self):
        """Return index and name of the most probable motion mode."""
        mode_names = ['CP', 'CV']
        idx = int(np.argmax(self.mu))
        return idx, mode_names[idx]


class HumanTrackKF:
    """Represents a tracked human with an IMM (Interacting Multiple Model) filter."""

    def __init__(self, track_id, position, timestamp, confidence=0.5,
                 measurement_noise=0.3):
        self.id = track_id
        self.kf = IMMFilter(
            position,
            initial_confidence=confidence,
            measurement_noise=measurement_noise
        )
        self.last_seen = timestamp
        self.confidence = confidence

    def predict(self, current_time):
        """Predict track state forward."""
        if self.kf.last_update is not None:
            dt = (current_time - self.kf.last_update).nanoseconds / 1e9
        else:
            dt = 0.1  # Default 10 Hz

        # Guard: skip prediction if dt is negligibly small to prevent
        # covariance inflation without meaningful state propagation.
        if dt < 1e-4:
            return

        self.kf.predict(dt)
        self.kf.last_update = current_time

    def update(self, measurement, timestamp, confidence=None, distance=0.0):
        """Update track with new measurement.

        predict() is already called for all tracks in the fusion loop
        before association.  Do NOT call self.kf.predict() here.

        Args:
            measurement: [x, y] observed position
            timestamp:   ROS Time of the measurement
            confidence:  Detection confidence (0-1)
            distance:    Range from robot to detection (metres)
        """
        conf = confidence if confidence is not None else 0.5
        self.kf.update(measurement, conf, distance)

        self.last_seen = timestamp
        self.kf.last_update = timestamp

        if confidence is not None:
            self.confidence = confidence

    def get_position(self):
        """Get current position [x, y]."""
        return self.kf.get_position()

    def get_velocity(self):
        """Get current velocity [vx, vy]."""
        return self.kf.get_velocity()


class HumanFusionKFNode(Node):
    """
    Fuses YOLO and LiDAR human detections with IMM tracking.

    Uses a 2-model Interacting Multiple Model filter (CP / CV)
    with Mahalanobis gating and dynamic measurement noise.
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
        self.declare_parameter('measurement_noise', 0.15)  # Measurement noise (m²)
        
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
            slop=0.3  # 300 ms tolerance — relaxed to prevent data starvation
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
        
        self.get_logger().info(f'Human Fusion IMM Node initialized')
        self.get_logger().info(f'  Camera FOV: {self.get_parameter("camera_fov_degrees").value}°')
        self.get_logger().info(f'  Fusion threshold: {self.fusion_threshold}m')
        self.get_logger().info(f'  IMM models: CP / CV  |  Mahalanobis gate: 50.0')
        self.get_logger().info(f'  Base measurement noise: {self.measurement_noise} m²')
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
        YOLO and LiDAR PoseArray messages (slop ≤ 300 ms).
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
                        dist_to_robot = float(np.linalg.norm(
                            fused_positions[j] - robot_pos
                        ))
                        self.tracks[tid].update(
                            fused_positions[j],
                            current_time,
                            confidence=fused_confidences[j],
                            distance=dist_to_robot
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
            
            # Encode directional velocity for RL observation space
            pose.orientation.x = float(vel[0])   # vx (m/s)
            pose.orientation.y = float(vel[1])   # vy (m/s)
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            
            pose_array.poses.append(pose)
        
        self.fused_poses_pub.publish(pose_array)
    
    def publish_markers(self, timestamp):
        """Publish visualization markers with KF-tracked humans."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for track in self.tracks.values():
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
            
            _, mode_name = track.kf.get_dominant_mode()
            text.text = f'IMM-ID:{track.id}\n{vel_mag:.2f}m/s\nMode:{mode_name} Conf:{track.confidence:.2f}'
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
