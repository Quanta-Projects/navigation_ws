#!/usr/bin/env python3
"""
ROS 2 Node for DR-SPAAM 2D LiDAR Person Detection
Detects people using 2D LiDAR scans with DR-SPAAM or DROW3 models
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from dr_spaam.detector import Detector
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time
import math
from collections import deque


class DrSpaamNode(Node):
    """ROS 2 node to detect pedestrians using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__('dr_spaam_node')

        # Declare parameters
        self.declare_parameter('weight_file', '')
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('stride', 1)
        self.declare_parameter('detector_model', 'DR-SPAAM')  # or 'DROW3'
        self.declare_parameter('panoramic_scan', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'map')  # Frame for published poses (map/odom/base_link)
        self.declare_parameter('detections_topic', 'detected_people')
        self.declare_parameter('marker_topic', 'detected_people_markers')

        # Get parameters
        weight_file = self.get_parameter('weight_file').value
        self.conf_thresh = self.get_parameter('conf_thresh').value
        stride = self.get_parameter('stride').value
        detector_model = self.get_parameter('detector_model').value
        panoramic_scan = self.get_parameter('panoramic_scan').value
        scan_topic = self.get_parameter('scan_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        detections_topic = self.get_parameter('detections_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        # TF2 buffer and listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Check if weight file is provided
        if not weight_file:
            self.get_logger().error('weight_file parameter is required!')
            raise ValueError('weight_file parameter must be set')

        # Initialize detector
        self.get_logger().info(f'Loading {detector_model} model: {weight_file}')
        try:
            self._detector = Detector(
                weight_file,
                model=detector_model,
                gpu=True,
                stride=stride,
                panoramic_scan=panoramic_scan,
            )
            self.get_logger().info(f'{detector_model} detector initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to load detector: {e}')
            raise

        # Publishers
        self._dets_pub = self.create_publisher(PoseArray, detections_topic, 10)
        self._rviz_pub = self.create_publisher(Marker, marker_topic, 10)
        self._filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)

        # Subscriber
        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            10
        )

        # Map state variables for static obstacle filtering
        self.map_data = None
        self.map_info = None

        # Map subscriber with Transient Local QoS (for latched map topics)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos
        )

        # Performance metrics
        self.frame_times = deque(maxlen=30)  # Store last 30 frame processing times
        self.last_metrics_log = time.time()
        self.frame_count = 0

        self.get_logger().info('DR-SPAAM Node initialized')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing detections to: {detections_topic}')
        self.get_logger().info(f'Publishing markers to: {marker_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

    def _scan_callback(self, msg):
        """Process incoming laser scan and detect people"""
        if (self._dets_pub.get_subscription_count() == 0 and 
            self._rviz_pub.get_subscription_count() == 0):
            return

        start_time = time.time()

        if not self._detector.is_ready():
            fov_deg = np.rad2deg(msg.angle_increment * len(msg.ranges))
            self._detector.set_laser_fov(fov_deg)
            self.get_logger().info(f'Laser FOV set to: {fov_deg:.2f} degrees')

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # Filter out static obstacles using map grid masking
        scan = self._filter_static_obstacles(scan, msg)

        # Lazily publish filtered scan for RViz debugging
        if self._filtered_scan_pub.get_subscription_count() > 0:
            filtered_msg = LaserScan()
            filtered_msg.header = msg.header
            filtered_msg.angle_min = msg.angle_min
            filtered_msg.angle_max = msg.angle_max
            filtered_msg.angle_increment = msg.angle_increment
            filtered_msg.time_increment = msg.time_increment
            filtered_msg.scan_time = msg.scan_time
            filtered_msg.range_min = msg.range_min
            filtered_msg.range_max = msg.range_max
            filtered_msg.intensities = msg.intensities
            filtered_msg.ranges = [float(r) for r in scan.tolist()]
            self._filtered_scan_pub.publish(filtered_msg)

        dets_xy, dets_cls, _ = self._detector(scan)

        # Apply nms* centroid averaging on raw output BEFORE confidence threshold
        # so cluster averaging can dilute false positive spikes across soft detections
        dets_xy, dets_cls = self._nms_star(dets_xy, dets_cls)

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        self.get_logger().info(f'Detected {len(dets_xy)} people', throttle_duration_sec=1.0)

        dets_msg = self._detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        
        if self.target_frame and self.target_frame != msg.header.frame_id:
            dets_msg = self._transform_pose_array_to_target_frame(dets_msg)
        
        # Inject confidence AFTER frame transform to prevent quaternion corruption
        for pose, d_cls in zip(dets_msg.poses, dets_cls):
            pose.position.z = float(d_cls)
        
        self._dets_pub.publish(dets_msg)

        if self._rviz_pub.get_subscription_count() > 0:
            rviz_msg = self._detections_to_rviz_marker(dets_xy, dets_cls)
            rviz_msg.header = msg.header
            
            if self.target_frame and self.target_frame != msg.header.frame_id:
                rviz_msg = self._transform_marker_to_target_frame(rviz_msg)
            
            self._rviz_pub.publish(rviz_msg)

        processing_time = time.time() - start_time
        self.frame_times.append(processing_time)
        self.frame_count += 1
        
        current_time = time.time()
        if current_time - self.last_metrics_log >= 5.0:
            avg_time = np.mean(self.frame_times)
            fps = 1.0 / avg_time if avg_time > 0 else 0
            min_time = np.min(self.frame_times)
            max_time = np.max(self.frame_times)
            self.get_logger().info(
                f'[DR-SPAAM Performance] FPS: {fps:.2f} | Avg: {avg_time*1000:.1f}ms'
            )
            self.last_metrics_log = current_time

    def _map_callback(self, msg):
        """Store the occupancy grid map as a 2D numpy array."""
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f}m/cell',
            throttle_duration_sec=10.0
        )

    def _filter_static_obstacles(self, scan_ranges, msg):
        """
        Vectorized Map Grid Masking with 3x3 Software Dilation.
        Filters out LiDAR points that hit known occupied or unknown cells in
        the occupancy grid (checking a 3x3 window to tolerate AMCL jitter),
        so DR-SPAAM only sees dynamic objects (people).
        """
        if self.map_data is None:
            return scan_ranges

        # --- 1. Get transform from laser frame to map frame (zero-wait) ---
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
            except (TransformException, tf2_ros.LookupException,
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
        except Exception as e:
            self.get_logger().warn(
                f'Map filter: TF lookup failed, skipping filter: {e}',
                throttle_duration_sec=5.0
            )
            return scan_ranges

        # --- 2. Identify valid (non-max-range) points ---
        valid_mask = scan_ranges < 29.99
        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            return scan_ranges

        valid_ranges = scan_ranges[valid_indices]

        # Compute angles for valid points
        angles = msg.angle_min + valid_indices.astype(np.float64) * msg.angle_increment

        # --- 3. Polar to local Cartesian ---
        local_x = valid_ranges * np.cos(angles)
        local_y = valid_ranges * np.sin(angles)

        # --- 4. Apply TF transform (translation + yaw rotation) ---
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        global_x = cos_yaw * local_x - sin_yaw * local_y + t.x
        global_y = sin_yaw * local_x + cos_yaw * local_y + t.y

        # --- 5. Convert global coordinates to map grid indices ---
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        grid_x = ((global_x - origin_x) / resolution).astype(np.int32)
        grid_y = ((global_y - origin_y) / resolution).astype(np.int32)

        # --- 6. Boundary check with 1-pixel safe margin for 3x3 window ---
        in_bounds = (
            (grid_x >= 1) & (grid_x < self.map_info.width - 1) &
            (grid_y >= 1) & (grid_y < self.map_info.height - 1)
        )

        bounded_indices = np.where(in_bounds)[0]
        if len(bounded_indices) == 0:
            return scan_ranges

        valid_grid_x = grid_x[bounded_indices]
        valid_grid_y = grid_y[bounded_indices]

        # --- 7. 3x3 Dilation Check ---
        # A point is filtered if ANY cell in its 3x3 neighbourhood is
        # occupied (> 50) or unknown (-1), tolerating AMCL localisation jitter.
        occupied = np.zeros(len(bounded_indices), dtype=bool)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                ny = valid_grid_y + dy
                nx = valid_grid_x + dx
                occ_values = self.map_data[ny, nx]
                occupied |= (occ_values > 50) | (occ_values == -1)

        # --- 8. Overwrite occupied scan points with max range ---
        original_indices = valid_indices[bounded_indices[occupied]]
        scan_ranges[original_indices] = 29.99

        return scan_ranges

    def _nms_star(self, dets_xy, dets_cls, radius=0.5):
        """
        ETH Zurich nms* Centroid Averaging.
        Replaces naive winner-takes-all NMS by averaging the spatial coordinates
        and confidences of all raw detections within the cluster radius.
        """
        dets_cls = dets_cls.reshape(-1)
        if len(dets_cls) == 0:
            return dets_xy, dets_cls

        order = np.argsort(dets_cls)[::-1]
        dets_xy = dets_xy[order]
        dets_cls = dets_cls[order]

        keep_xy = []
        keep_cls = []

        while len(dets_cls) > 0:
            center_xy = dets_xy[0]
            distances = np.linalg.norm(dets_xy - center_xy, axis=1)
            cluster_indices = np.where(distances <= radius)[0]

            cluster_xy = dets_xy[cluster_indices]
            cluster_cls = dets_cls[cluster_indices]

            avg_xy = np.mean(cluster_xy, axis=0)
            avg_cls = np.mean(cluster_cls)

            keep_xy.append(avg_xy)
            keep_cls.append(avg_cls)

            dets_xy = np.delete(dets_xy, cluster_indices, axis=0)
            dets_cls = np.delete(dets_cls, cluster_indices, axis=0)

        return np.array(keep_xy), np.array(keep_cls)

    def _detections_to_rviz_marker(self, dets_xy, dets_cls):
        """
        Convert detections to RViz marker. Each detection is marked as
        a circle approximated by line segments.
        """
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = "dr_spaam_detections"
        msg.id = 0
        msg.type = Marker.LINE_LIST

        # Set quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.05  # line width
        # Red color
        msg.color.r = 1.0
        msg.color.a = 1.0

        # Circle parameters
        r = 0.4  # radius
        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        # Create circle for each detection
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            for i in range(len(xy_offsets) - 1):
                # Start point of segment
                p0 = Point()
                p0.x = float(d_xy[0] + xy_offsets[i, 0])
                p0.y = float(d_xy[1] + xy_offsets[i, 1])
                p0.z = 0.0
                msg.points.append(p0)

                # End point
                p1 = Point()
                p1.x = float(d_xy[0] + xy_offsets[i + 1, 0])
                p1.y = float(d_xy[1] + xy_offsets[i + 1, 1])
                p1.z = 0.0
                msg.points.append(p1)

        return msg

    def _detections_to_pose_array(self, dets_xy, dets_cls):
        """Convert detections to PoseArray message with pure identity quaternions"""
        pose_array = PoseArray()
        for d_xy in dets_xy:
            p = Pose()
            p.position.x = float(d_xy[0])
            p.position.y = float(d_xy[1])
            p.position.z = 0.0
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0  # Clean quaternion!
            p.orientation.w = 1.0
            pose_array.poses.append(p)
        return pose_array

    def _transform_pose_array_to_target_frame(self, pose_array):
        """Transform PoseArray using zero-wait Ego-Motion fix"""
        try:
            try:
                # Try exact sensor time instantly
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pose_array.header.frame_id,
                    rclpy.time.Time.from_msg(pose_array.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
            except (TransformException, tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Fallback to latest instantly
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pose_array.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
            
            transformed_poses = []
            for pose in pose_array.poses:
                pose_stamped = PoseStamped()
                pose_stamped.header = pose_array.header
                pose_stamped.pose = pose
                transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
                transformed_poses.append(transformed_pose_stamped.pose)
            
            transformed_pose_array = PoseArray()
            transformed_pose_array.header.stamp = pose_array.header.stamp
            transformed_pose_array.header.frame_id = self.target_frame
            transformed_pose_array.poses = transformed_poses
            return transformed_pose_array
            
        except Exception as e:
            self.get_logger().warn(f'Failed to transform pose array: {e}', throttle_duration_sec=5.0)
            return pose_array

    def _transform_marker_to_target_frame(self, marker):
        """Transform markers using zero-wait Ego-Motion fix"""
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    marker.header.frame_id,
                    rclpy.time.Time.from_msg(marker.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
            except (TransformException, tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    marker.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )
            
            transformed_points = []
            for point in marker.points:
                point_stamped = PoseStamped()
                point_stamped.header = marker.header
                point_stamped.pose.position = point
                point_stamped.pose.orientation.w = 1.0
                transformed_point_stamped = tf2_geometry_msgs.do_transform_pose_stamped(point_stamped, transform)
                transformed_points.append(transformed_point_stamped.pose.position)
            
            marker.header.frame_id = self.target_frame
            marker.points = transformed_points
            return marker
            
        except Exception as e:
            self.get_logger().warn(f'Failed to transform marker: {e}', throttle_duration_sec=5.0)
            return marker


def main(args=None):
    rclpy.init(args=args)
    node = DrSpaamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
