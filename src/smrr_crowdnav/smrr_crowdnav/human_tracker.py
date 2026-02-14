#!/usr/bin/env python3
"""
YOLO26 Instance Segmentation and Tracking Node
Tracks humans using YOLO26 instance segmentation with persistent IDs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters
import time
from collections import deque
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os


class HumanInstanceTracker(Node):
    """
    ROS 2 Node for tracking humans using YOLO26 instance segmentation
    """

    def __init__(self):
        super().__init__('human_tracker')

        # Get default model path from package share directory
        pkg_share = get_package_share_directory('smrr_crowdnav')
        default_model = os.path.join(pkg_share, 'models', 'yolo26n-seg.pt')

        # Declare parameters
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rgb_topic', '/zed2_left_camera/image_raw')
        self.declare_parameter('depth_topic', '/zed2_left_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/zed2_left_camera/camera_info')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('max_detection_distance', 15.0)
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('target_frame', 'map')  # Frame for published poses (map/odom/base_link)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_viz = self.get_parameter('publish_visualization').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        tracker_type = self.get_parameter('tracker').value
        iou_threshold = self.get_parameter('iou_threshold').value
        self.target_frame = self.get_parameter('target_frame').value

        # TF2 for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Predefined colors for different tracked instances (BGR format)
        self.track_colors = [
            (255, 0, 0),      # Blue
            (0, 255, 0),      # Green
            (0, 0, 255),      # Red
            (255, 255, 0),    # Cyan
            (255, 0, 255),    # Magenta
            (0, 255, 255),    # Yellow
            (128, 0, 255),    # Purple
            (0, 128, 255),    # Orange
            (255, 128, 0),    # Sky Blue
            (128, 255, 0),    # Spring Green
            (255, 128, 128),  # Light Blue
            (128, 255, 128),  # Light Green
            (128, 128, 255),  # Light Red
            (0, 165, 255),    # Deep Orange
            (255, 191, 0),    # Deep Sky Blue
        ]

        # Initialize YOLO26 Instance Segmentation with built-in tracking
        self.get_logger().info(f'Loading YOLO26 model: {model_path}')
        
        # Check GPU availability
        import torch
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        if torch.cuda.is_available():
            self.get_logger().info(f'GPU: {torch.cuda.get_device_name(0)}')
        
        try:
            # Load YOLO26 model for direct access to tracking results
            self.model = YOLO(model_path)
            self.model.to(self.device)
            self.tracker_type = tracker_type
            self.iou_threshold = iou_threshold
            self.get_logger().info(f'YOLO26 Instance Segmentation loaded on {self.device}. Using tracker: {tracker_type}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO26 model: {e}')
            raise

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Synchronized subscribers for RGB and Depth
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        # Time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, 'tracked_humans/poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tracked_humans/markers', 10)
        
        if self.publish_viz:
            self.viz_pub = self.create_publisher(Image, 'tracked_humans/visualization', 10)

        # Performance metrics
        self.frame_times = deque(maxlen=30)  # Store last 30 frame processing times
        self.last_metrics_log = time.time()
        self.frame_count = 0

        self.get_logger().info('Human Instance Tracker Node initialized')

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.get_logger().info('Camera intrinsics received')

    def synchronized_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and Depth images"""
        try:
            # Start timing
            start_time = time.time()
            
            # Convert ROS images to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Run YOLO26 instance segmentation with tracking
            results = self.model.track(
                rgb_image,
                persist=True,
                tracker=self.tracker_type,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                classes=[0],  # Only track persons
                verbose=False,
                device=self.device
            )

            # Extract tracked instances from the results
            tracked_humans = []
            if results is not None and len(results) > 0:
                result = results[0]
                
                # Check if we have tracking IDs
                if result.boxes is not None and result.boxes.id is not None:
                    track_ids = result.boxes.id.cpu().numpy().astype(int)
                    confidences = result.boxes.conf.cpu().numpy()
                    xyxy = result.boxes.xyxy.cpu().numpy()
                    
                    # Get segmentation masks if available
                    masks = None
                    if hasattr(result, 'masks') and result.masks is not None:
                        masks = result.masks.data.cpu().numpy()
                    
                    for idx, track_id in enumerate(track_ids):
                        x1, y1, x2, y2 = xyxy[idx]
                        
                        # Calculate 3D position using depth
                        position_3d = self.calculate_3d_position(
                            int(x1), int(y1), int(x2), int(y2), depth_image
                        )
                        
                        if position_3d is not None:
                            # Get color for this track ID
                            color_idx = track_id % len(self.track_colors)
                            color = self.track_colors[color_idx]
                            
                            # Get mask for this instance if available
                            mask = masks[idx] if masks is not None else None
                            
                            tracked_humans.append({
                                'track_id': int(track_id),
                                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                'confidence': float(confidences[idx]),
                                'position_3d': position_3d,
                                'color': color,
                                'mask': mask
                            })

            # Publish results
            self.publish_detections(tracked_humans, rgb_msg.header)

            # Publish visualization
            if self.publish_viz:
                viz_image = self.create_visualization(rgb_image, tracked_humans)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = rgb_msg.header
                self.viz_pub.publish(viz_msg)

            # Calculate performance metrics
            processing_time = time.time() - start_time
            self.frame_times.append(processing_time)
            self.frame_count += 1
            
            # Log performance metrics every 5 seconds
            current_time = time.time()
            if current_time - self.last_metrics_log >= 5.0:
                avg_time = np.mean(self.frame_times)
                fps = 1.0 / avg_time if avg_time > 0 else 0
                min_time = np.min(self.frame_times)
                max_time = np.max(self.frame_times)
                
                self.get_logger().info(
                    f'[YOLO26 Performance] FPS: {fps:.2f} | '
                    f'Avg: {avg_time*1000:.1f}ms | '
                    f'Min: {min_time*1000:.1f}ms | '
                    f'Max: {max_time*1000:.1f}ms | '
                    f'Frames: {self.frame_count}'
                )
                self.last_metrics_log = current_time

            if len(tracked_humans) > 0:
                self.get_logger().info(
                    f'Tracking {len(tracked_humans)} humans: ' + 
                    ', '.join([f'ID {h["track_id"]}' for h in tracked_humans]),
                    throttle_duration_sec=1.0
                )
            else:
                # Debug: No detections
                self.get_logger().debug(
                    'No humans detected in current frame',
                    throttle_duration_sec=2.0
                )

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def calculate_3d_position(self, x1, y1, x2, y2, depth_image):
        """
        Calculate 3D position of tracked human using depth information
        
        Args:
            x1, y1, x2, y2: Bounding box coordinates
            depth_image: Depth image array
            
        Returns:
            np.array: [x, y, z] position in camera frame or None
        """
        if self.camera_matrix is None:
            return None

        # Calculate center of bounding box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Get depth value at center
        h, w = depth_image.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return None

        # Average depth in a region around center
        region_size = 10
        y_start = max(0, cy - region_size)
        y_end = min(h, cy + region_size)
        x_start = max(0, cx - region_size)
        x_end = min(w, cx + region_size)

        depth_region = depth_image[y_start:y_end, x_start:x_end]
        
        # Filter out invalid depth values
        valid_depths = depth_region[(depth_region > 0) & np.isfinite(depth_region)]
        
        if len(valid_depths) == 0:
            return None

        # Use median for robustness
        depth = np.median(valid_depths)

        # Convert depth from mm to meters if needed
        if depth > 100:
            depth = depth / 1000.0

        # Check if within max distance
        if depth > self.max_distance or depth <= 0:
            return None

        # Convert pixel coordinates to 3D using camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]

        # Calculate 3D position in camera frame
        x = (cx - cx_cam) * depth / fx
        y = (cy - cy_cam) * depth / fy
        z = depth

        return np.array([x, y, z])

    def publish_detections(self, tracked_humans, header):
        """Publish tracked humans as PoseArray and MarkerArray with frame transformation"""
        # Debug: Log number of tracked humans
        self.get_logger().info(
            f'Publishing {len(tracked_humans)} humans from frame {header.frame_id} to {self.target_frame}',
            throttle_duration_sec=1.0
        )
        
        # Publish PoseArray in target frame
        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = header.stamp

        # Publish MarkerArray
        marker_array = MarkerArray()
        
        transform_success_count = 0
        transform_fail_count = 0

        for human in tracked_humans:
            pos_3d = human['position_3d']
            track_id = human['track_id']
            color_bgr = human['color']
            
            # Convert BGR to RGB and normalize to 0-1
            color_rgb = (color_bgr[2]/255.0, color_bgr[1]/255.0, color_bgr[0]/255.0)
            
            # Create Pose in camera frame
            pose_camera = Pose()
            pose_camera.position.x = float(pos_3d[0])
            pose_camera.position.y = float(pos_3d[1])
            pose_camera.position.z = float(pos_3d[2])
            pose_camera.orientation.z = float(human['confidence'])  # Encode confidence
            pose_camera.orientation.w = 1.0
            
            # Transform pose to target frame (map/odom)
            pose_map = self.transform_pose_to_map(pose_camera, header)
            
            if pose_map is not None:
                transform_success_count += 1
                pose_array.poses.append(pose_map)

                # Add text marker with track ID in target frame
                text_marker = Marker()
                text_marker.header.frame_id = self.target_frame
                text_marker.header.stamp = header.stamp
                text_marker.ns = "track_labels"
                text_marker.id = track_id + 10000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = pose_map.position.x
                text_marker.pose.position.y = pose_map.position.y
                text_marker.pose.position.z = 0.5  # Position text above footprint
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.3
                text_marker.color.r = 0.0
                text_marker.color.g = 0.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"ID {track_id}"
                text_marker.lifetime.sec = 0
                text_marker.lifetime.nanosec = 500000000
                
                # Add footprint marker (circular disk at ground level) - always green
                footprint_marker = Marker()
                footprint_marker.header.frame_id = self.target_frame
                footprint_marker.header.stamp = header.stamp
                footprint_marker.ns = "track_footprints"
                footprint_marker.id = track_id
                footprint_marker.type = Marker.CYLINDER
                footprint_marker.action = Marker.ADD
                # Position at ground level (z=0)
                footprint_marker.pose.position.x = pose_map.position.x
                footprint_marker.pose.position.y = pose_map.position.y
                footprint_marker.pose.position.z = 0.01  # Slightly above ground to avoid z-fighting
                footprint_marker.pose.orientation.w = 1.0
                # Flat disk representing footprint
                footprint_marker.scale.x = 0.5  # Footprint diameter
                footprint_marker.scale.y = 0.5
                footprint_marker.scale.z = 0.02  # Very thin disk
                # Always green
                footprint_marker.color.r = 0.0
                footprint_marker.color.g = 1.0
                footprint_marker.color.b = 0.0
                footprint_marker.color.a = 0.8
                footprint_marker.lifetime.sec = 0
                footprint_marker.lifetime.nanosec = 500000000

                marker_array.markers.append(text_marker)
                marker_array.markers.append(footprint_marker)
            else:
                transform_fail_count += 1

        # Log transform results
        if len(tracked_humans) > 0:
            self.get_logger().info(
                f'Transform results: {transform_success_count} succeeded, {transform_fail_count} failed. '
                f'Publishing {len(pose_array.poses)} poses in {self.target_frame} frame.',
                throttle_duration_sec=1.0
            )
        
        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)

    def transform_pose_to_map(self, pose, header):
        """
        Transform pose from camera frame to target frame using TF2
        
        Args:
            pose: Pose in camera frame
            header: Original message header with camera frame and timestamp
            
        Returns:
            Pose in target frame or None if transform fails
        """
        try:
            # Create PoseStamped in camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = header.frame_id
            # Use current time (Time(0)) to get latest available transform
            # This avoids extrapolation errors when camera timestamp is ahead of TF
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose
            
            # Transform to target frame using latest available transform
            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            return transformed_pose.pose
            
        except (TransformException, tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Log warning for transform failures
            if isinstance(e, tf2_ros.LookupException):
                self.get_logger().warn(
                    f'Transform lookup failed: {header.frame_id} -> {self.target_frame}. '
                    f'Ensure TF tree is published and frames exist. Error: {e}',
                    throttle_duration_sec=5.0
                )
            elif isinstance(e, tf2_ros.ExtrapolationException):
                self.get_logger().warn(
                    f'Transform extrapolation error: {e}',
                    throttle_duration_sec=5.0
                )
            else:
                self.get_logger().warn(
                    f'Transform failed from {header.frame_id} to {self.target_frame}: {e}',
                    throttle_duration_sec=5.0
                )
            return None

    def create_visualization(self, image, tracked_humans):
        """Create visualization image with bounding boxes and track IDs"""
        viz_image = image.copy()

        # Draw bounding boxes and labels
        for human in tracked_humans:
            x1, y1, x2, y2 = human['bbox']
            track_id = human['track_id']
            color = human['color']
            pos_3d = human['position_3d']
            distance = np.linalg.norm(pos_3d)
            
            # Draw bounding box
            cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)
            
            # Calculate center point
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            # Draw center point
            cv2.circle(viz_image, (cx, cy), 5, color, -1)
            cv2.circle(viz_image, (cx, cy), 6, (255, 255, 255), 2)

            # Draw label with track ID
            label = f"ID {track_id}: {distance:.2f}m"
            
            # Calculate text size for background
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # Draw background rectangle for text above bbox
            cv2.rectangle(
                viz_image,
                (x1, y1 - text_height - 10),
                (x1 + text_width + 10, y1),
                color,
                -1
            )
            
            # Draw text in white
            cv2.putText(
                viz_image,
                label,
                (x1 + 5, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # Draw center point
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(viz_image, (cx, cy), 5, color, -1)

        return viz_image


def main(args=None):
    rclpy.init(args=args)
    node = HumanInstanceTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
