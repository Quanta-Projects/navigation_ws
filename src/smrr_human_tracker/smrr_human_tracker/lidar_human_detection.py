#!/usr/bin/env python3
"""
ROS 2 Node for DR-SPAAM 2D LiDAR Person Detection
Detects people using 2D LiDAR scans with DR-SPAAM or DROW3 models.

Supports two inference backends:
  • PyTorch (.pth) — original DR-SPAAM with spatial-attention gate
  • ONNX Runtime (.onnx) — stateless CNN backbone, ~27 % faster on CUDA
If a sibling .onnx file exists next to the .pth weight file the ONNX
backend is chosen automatically (override with ``use_onnx`` parameter).
"""

import os
import numpy as np
import cv2
import torch
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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
from collections import deque


# ---------------------------------------------------------------------------
# ONNX Runtime backend for DR-SPAAM inference
# ---------------------------------------------------------------------------

def _ensure_cudnn_visible():
    """Append pip-installed nvidia-cudnn lib dir to LD_LIBRARY_PATH so that
    onnxruntime's CUDAExecutionProvider can find libcudnn.so.9 at runtime."""
    try:
        import nvidia.cudnn as _cudnn
        init_file = getattr(_cudnn, "__file__", None)
        if init_file is None:
            # namespace-package: use the loader path instead
            init_file = _cudnn.__path__[0]
        cudnn_lib = os.path.join(os.path.dirname(init_file), "lib")
        ld = os.environ.get("LD_LIBRARY_PATH", "")
        if cudnn_lib not in ld:
            os.environ["LD_LIBRARY_PATH"] = f"{cudnn_lib}:{ld}"
    except (ImportError, Exception):
        pass


class ONNXModelWrapper:
    """Drop-in replacement for ``detector._model`` that runs ONNX Runtime.

    The Detector's ``__call__`` invokes ``self._model(ct.unsqueeze(0), inference=True)``
    and expects ``(pred_cls, pred_reg, sim)`` back.  This wrapper fulfils that
    contract using an ONNX Runtime session.
    """

    def __init__(self, onnx_path: str, logger):
        _ensure_cudnn_visible()
        import onnxruntime as ort

        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        self._session = ort.InferenceSession(onnx_path, providers=providers)
        self._input_name = self._session.get_inputs()[0].name
        active = self._session.get_providers()
        logger.info(f"ONNX Runtime providers: {active}")
        if "CUDAExecutionProvider" not in active:
            logger.warn(
                "CUDAExecutionProvider NOT available — falling back to CPU. "
                "Set LD_LIBRARY_PATH to include libcudnn.so.9 for GPU inference."
            )

    # --- public interface expected by Detector.__call__ ---

    def __call__(self, x, inference=True):
        """Run ONNX inference on cutout tensor *x*.

        Args:
            x: torch.Tensor shaped (B, CT, 1, 56), typically on CUDA.
            inference: ignored (stateless model).

        Returns:
            (pred_cls, pred_reg, sim) matching DR-SPAAM convention.
            sim is always ``None`` (no spatial attention in the ONNX graph).
        """
        x_np = x.detach().cpu().numpy()
        cls_np, reg_np = self._session.run(None, {self._input_name: x_np})
        # Return CPU tensors — Detector immediately calls .cpu().numpy() anyway.
        pred_cls = torch.from_numpy(cls_np)
        pred_reg = torch.from_numpy(reg_np)
        return pred_cls, pred_reg, None

    def eval(self):
        """No-op, keeps the Detector happy if it ever calls model.eval()."""
        return self

    def cuda(self):
        """No-op — inference device is managed by the ORT session."""
        return self


class LidarHumanDetectionNode(Node):
    """ROS 2 node to detect pedestrians using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__('lidar_human_detection_node')

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
        self.declare_parameter('use_onnx', True)  # auto-use ONNX if .onnx sibling exists

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
        use_onnx = self.get_parameter('use_onnx').value

        # TF2 buffer and listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Check if weight file is provided
        if not weight_file:
            self.get_logger().error('weight_file parameter is required!')
            raise ValueError('weight_file parameter must be set')

        # Initialize detector (always loads .pth for preprocessing / NMS)
        self.get_logger().info(f'Loading {detector_model} model: {weight_file}')
        try:
            self._detector = Detector(
                detector_model,
                weight_file,
                gpu=True,
                stride=stride,
            )
            self.get_logger().info(f'{detector_model} detector initialized (PyTorch)')
        except Exception as e:
            self.get_logger().error(f'Failed to load detector: {e}')
            raise

        # ── ONNX backend hot-swap ──────────────────────────────────
        # If a sibling .onnx file exists and use_onnx is True, replace
        # the PyTorch _model with an ONNX Runtime session. The Detector
        # still owns preprocessing (scans_to_cutout) and postprocessing
        # (NMS), which are pure NumPy / CPU — only the neural-network
        # forward pass is swapped.
        onnx_path = os.path.splitext(weight_file)[0] + ".onnx"
        if use_onnx and os.path.isfile(onnx_path):
            self.get_logger().info(f"ONNX model found — switching backend: {onnx_path}")
            self._detector._model = ONNXModelWrapper(onnx_path, self.get_logger())
        elif use_onnx:
            self.get_logger().info(
                f"No sibling .onnx file at {onnx_path} — keeping PyTorch backend"
            )

        # Publishers
        self._dets_pub = self.create_publisher(PoseArray, detections_topic, 10)
        self._rviz_pub = self.create_publisher(Marker, marker_topic, 10)
        self._filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)

        # Subscriber
        # MutuallyExclusiveCallbackGroup serializes scan callbacks — prevents concurrent
        # CUDA access (CUBLAS_STATUS_ALLOC_FAILED) while MultiThreadedExecutor still
        # serves TF and map callbacks in parallel threads.
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            10,
            callback_group=self.cb_group
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

        self.get_logger().info('LidarHumanDetectionNode initialized')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing detections to: {detections_topic}')
        self.get_logger().info(f'Publishing markers to: {marker_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

    def _scan_callback(self, msg):
        """Process incoming laser scan and detect people"""
        start_time = time.time()

        # --- 1. STRICT TF LOOKUP (Zero Ego-Motion with Clock Skew Buffer) ---
        # One lookup per frame eliminates cascading timeout overhead.
        try:
            target_time = msg.header.stamp
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(target_time),
                timeout=rclpy.duration.Duration(seconds=0.05)  # 50ms buffer for hardware clock skew
            )
        except Exception as e:
            self.get_logger().warn(f'Dropped frame (TF Sync): {e}', throttle_duration_sec=2.0)
            return

        if not self._detector.laser_spec_set():
            self._detector.set_laser_spec(msg.angle_increment, len(msg.ranges))
            fov_deg = np.rad2deg(msg.angle_increment * len(msg.ranges))
            self.get_logger().info(f'Laser spec set: angle_inc={msg.angle_increment:.5f} rad, num_pts={len(msg.ranges)}, FOV={fov_deg:.2f} deg')

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # Filter out static obstacles using inflated binary grid (O(1) lookup)
        filtered_scan = self._filter_static_obstacles(scan, msg, transform)
        if filtered_scan is None:
            return  # STRICT: Skip frame if TF fails to prevent hallucinating walls
        scan = filtered_scan

        # Always publish filtered scan — unconditional publish eliminates the
        # "blinking" caused by gating on subscriber count. Explicitly setting
        # the stamp preserves temporal alignment with /scan.
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.header.stamp = msg.header.stamp  # explicit for temporal alignment
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
            dets_msg = self._transform_pose_array_to_target_frame(dets_msg, transform)
        
        # Inject confidence AFTER frame transform to prevent quaternion corruption
        for pose, d_cls in zip(dets_msg.poses, dets_cls):
            pose.position.z = float(d_cls)
        
        self._dets_pub.publish(dets_msg)

        if self._rviz_pub.get_subscription_count() > 0:
            rviz_msg = self._detections_to_rviz_marker(dets_xy, dets_cls)
            rviz_msg.header = msg.header
            
            if self.target_frame and self.target_frame != msg.header.frame_id:
                rviz_msg = self._transform_marker_to_target_frame(rviz_msg, transform)
            
            self._rviz_pub.publish(rviz_msg)

        processing_time = time.time() - start_time
        self.frame_times.append(processing_time)
        self.frame_count += 1
        
        current_time = time.time()
        if current_time - self.last_metrics_log >= 5.0:
            avg_time = np.mean(self.frame_times)
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(
                f'[LidarHumanDetection Performance] FPS: {fps:.2f} | Avg: {avg_time*1000:.1f}ms'
            )
            self.last_metrics_log = current_time

    def _map_callback(self, msg):
        """Store the map and build an inflated binary grid for O(1) static obstacle filtering."""
        self.map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # 1. Create a pure uint8 binary image for OpenCV
        # Free space = 0, Walls (>50) and Unknown (-1) = 255
        binary_map = np.zeros_like(grid, dtype=np.uint8)
        binary_map[(grid > 50) | (grid == -1)] = 255

        # 2. Calculate physical inflation in pixels (0.25m radius)
        inflation_radius_m = 0.35
        res = self.map_info.resolution
        inflation_pixels = int(inflation_radius_m / res)

        # 3. Create a circular kernel (must be an odd number size: 2*r + 1)
        k_size = (2 * inflation_pixels) + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))

        # 4. Perform the ultra-fast OpenCV dilation
        dilated_map = cv2.dilate(binary_map, kernel, iterations=1)

        # 5. Store as a boolean NumPy array for O(1) laser lookups
        self.inflated_map = (dilated_map == 255)

        self.get_logger().info(
            f'Map processed and dilated. Size: {msg.info.width}x{msg.info.height}. '
            f'Inflation radius: {inflation_pixels}px.',
            throttle_duration_sec=10.0
        )

    def _filter_static_obstacles(self, scan_ranges, msg, transform):
        """Filters out LiDAR points that hit mapped obstacles using O(1) Grid Indexing."""
        if not hasattr(self, 'inflated_map') or self.inflated_map is None:
            return scan_ranges

        # --- 1. Identify valid points ---
        valid_mask = scan_ranges < 29.99
        valid_indices = np.where(valid_mask)[0]

        if len(valid_indices) == 0:
            return scan_ranges

        valid_ranges = scan_ranges[valid_indices]
        angles = msg.angle_min + valid_indices.astype(np.float64) * msg.angle_increment

        # --- 2. Polar to local Cartesian ---
        local_x = valid_ranges * np.cos(angles)
        local_y = valid_ranges * np.sin(angles)

        # --- 3. Vectorized Homogeneous Transform to Map Frame ---
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Create 3xN matrix of local points: [x; y; 1]
        local_points = np.vstack((local_x, local_y, np.ones_like(local_x)))

        # 3x3 Homogeneous Transformation Matrix
        H_matrix = np.array([
            [cos_y, -sin_y, t.x],
            [sin_y,  cos_y, t.y],
            [0.0,    0.0,   1.0]
        ])

        # Matrix multiplication (3x3 @ 3xN) -> 3xN global points
        global_points = H_matrix @ local_points
        global_x = global_points[0, :]
        global_y = global_points[1, :]

        # --- 4. Convert to Grid Indices ---
        res = self.map_info.resolution
        orig_x = self.map_info.origin.position.x
        orig_y = self.map_info.origin.position.y

        cols = ((global_x - orig_x) / res).astype(int)
        rows = ((global_y - orig_y) / res).astype(int)

        # --- 5. O(1) Direct Array Indexing ---
        map_h, map_w = self.inflated_map.shape

        # Clip bounds to prevent IndexError if laser shoots outside the map
        valid_bounds = (rows >= 0) & (rows < map_h) & (cols >= 0) & (cols < map_w)

        hit_wall = np.zeros(len(global_x), dtype=bool)
        hit_wall[valid_bounds] = self.inflated_map[rows[valid_bounds], cols[valid_bounds]]

        # Overwrite points hitting dilated walls with max range
        original_indices = valid_indices[hit_wall]
        scan_ranges[original_indices] = 29.99

        return scan_ranges

    def _nms_star(self, dets_xy, dets_cls, radius=0.5):
        """
        ETH Zurich nms* Centroid Averaging.
        Replaces naive winner-takes-all NMS by averaging the spatial coordinates
        and confidences of all raw detections within the cluster radius.
        """
        dets_cls = dets_cls.reshape(-1)

        # HARD PRE-FILTER: Drop absolute garbage noise to save CPU in the while loop
        valid_mask = dets_cls > 0.05
        dets_xy = dets_xy[valid_mask]
        dets_cls = dets_cls[valid_mask]

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

    def _transform_pose_array_to_target_frame(self, pose_array, transform):
        """Transform PoseArray using the pre-computed per-frame transform."""
        try:
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

    def _transform_marker_to_target_frame(self, marker, transform):
        """Transform markers using the pre-computed per-frame transform."""
        try:
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
    node = LidarHumanDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
