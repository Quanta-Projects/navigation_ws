#!/usr/bin/env python3
"""
YOLO26-based Human Detection Node for RGBD Camera
Detects humans and estimates their 3D positions using depth information
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters
from scipy.spatial.distance import cdist
import time
from collections import deque
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os


class HumanDetector(Node):
    """
    ROS 2 Node for detecting humans using YOLO26 and RGBD camera
    """

    def __init__(self):
        super().__init__('human_detector')

        # Get default model path from package share directory
        pkg_share = get_package_share_directory('smrr_crowdnav')
        default_model = os.path.join(pkg_share, 'models', 'yolo26n.pt')

        # Declare parameters
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rgb_topic', '/zed2_left_camera/image_raw')
        self.declare_parameter('depth_topic', '/zed2_left_camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/zed2_left_camera/camera_info')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('max_detection_distance', 15.0)
        self.declare_parameter('tracking_distance_threshold', 3.0)  # Max distance to match same person
        self.declare_parameter('target_frame', 'map')  # Frame for published poses (map/odom/base_link)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_viz = self.get_parameter('publish_visualization').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        self.tracking_threshold = self.get_parameter('tracking_distance_threshold').value
        self.target_frame = self.get_parameter('target_frame').value

        # Instance tracking
        self.tracked_humans = {}  # {id: {'position': [x,y,z], 'last_seen': timestamp, 'color': (r,g,b)}}
        self.next_id = 1
        self.max_tracking_age = 2.0  # seconds before removing lost tracks
        
        # Predefined colors for different persons (BGR format for OpenCV)
        self.person_colors = [
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
        ]

        # TF2 for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize YOLO26 model
        self.get_logger().info(f'Loading YOLO26 model: {model_path}')
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO26 model loaded successfully')
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
        self.pose_pub = self.create_publisher(PoseArray, 'detected_humans/poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'detected_humans/markers', 10)
        
        if self.publish_viz:
            self.viz_pub = self.create_publisher(Image, 'detected_humans/visualization', 10)

        # Performance metrics
        self.frame_times = deque(maxlen=30)  # Store last 30 frame processing times
        self.last_metrics_log = time.time()
        self.frame_count = 0

        self.get_logger().info('Human Detector Node initialized')
        self.get_logger().info(f'Publishing poses in frame: {self.target_frame}')

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

            # Run YOLO26 detection
            results = self.model(rgb_image, verbose=False)

            # Process detections
            detected_humans = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Filter for person class (class 0 in COCO)
                    if int(box.cls) == 0 and float(box.conf) >= self.confidence_threshold:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf)

                        # Calculate 3D position using depth
                        human_3d_pos = self.calculate_3d_position(
                            x1, y1, x2, y2, depth_image
                        )

                        if human_3d_pos is not None:
                            detected_humans.append({
                                'bbox': (x1, y1, x2, y2),
                                'confidence': confidence,
                                'position_3d': human_3d_pos
                            })

            # Track instances and assign IDs
            current_time = self.get_clock().now().nanoseconds / 1e9
            tracked_detections = self.track_humans(detected_humans, current_time)

            # Publish results
            self.publish_detections(tracked_detections, rgb_msg.header)

            # Publish visualization with tracking info
            if self.publish_viz:
                viz_image = self.create_visualization(rgb_image, tracked_detections)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = rgb_msg.header
                self.viz_pub.publish(viz_msg)

            # Calculate performance metrics
            processing_time = time.time() - start_time
            self.frame_times.append(processing_time)
            self.frame_count += 1
            
            # Log performance metrics every 5 seconds
            current_time_metrics = time.time()
            if current_time_metrics - self.last_metrics_log >= 5.0:
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
                self.last_metrics_log = current_time_metrics

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')

    def track_humans(self, detected_humans, current_time):
        """
        Track human instances across frames using distance-based matching
        Returns list of detections with assigned IDs and colors
        """
        if len(detected_humans) == 0:
            return []

        # Remove stale tracks
        stale_ids = []
        for track_id, track_data in self.tracked_humans.items():
            if current_time - track_data['last_seen'] > self.max_tracking_age:
                stale_ids.append(track_id)
        
        for track_id in stale_ids:
            del self.tracked_humans[track_id]
            self.get_logger().info(f'Lost track of Person {track_id}')

        # If no existing tracks, assign new IDs to all
        if len(self.tracked_humans) == 0:
            tracked_detections = []
            for human in detected_humans:
                person_id = self.next_id
                color_idx = (person_id - 1) % len(self.person_colors)
                color = self.person_colors[color_idx]
                
                self.tracked_humans[person_id] = {
                    'position': human['position_3d'],
                    'last_seen': current_time,
                    'color': color
                }
                
                human['id'] = person_id
                human['color'] = color
                tracked_detections.append(human)
                
                self.next_id += 1
                self.get_logger().info(f'New track: Person {person_id}')
            
            return tracked_detections

        # Match detections to existing tracks
        track_ids = list(self.tracked_humans.keys())
        track_positions = np.array([self.tracked_humans[tid]['position'] for tid in track_ids])
        detection_positions = np.array([h['position_3d'] for h in detected_humans])

        # Calculate distance matrix
        distances = cdist(detection_positions, track_positions)

        # Simple greedy assignment
        matched_detections = []
        unmatched_detections = list(range(len(detected_humans)))
        matched_tracks = set()

        # Sort by minimum distance for each detection
        for det_idx in range(len(detected_humans)):
            min_dist_idx = np.argmin(distances[det_idx])
            min_dist = distances[det_idx, min_dist_idx]
            
            if min_dist < self.tracking_threshold and min_dist_idx not in matched_tracks:
                # Match found
                track_id = track_ids[min_dist_idx]
                matched_tracks.add(min_dist_idx)
                unmatched_detections.remove(det_idx)
                
                # Update track
                self.tracked_humans[track_id]['position'] = detected_humans[det_idx]['position_3d']
                self.tracked_humans[track_id]['last_seen'] = current_time
                
                # Add ID and color to detection
                detected_humans[det_idx]['id'] = track_id
                detected_humans[det_idx]['color'] = self.tracked_humans[track_id]['color']
                matched_detections.append(detected_humans[det_idx])

        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            person_id = self.next_id
            color_idx = (person_id - 1) % len(self.person_colors)
            color = self.person_colors[color_idx]
            
            self.tracked_humans[person_id] = {
                'position': detected_humans[det_idx]['position_3d'],
                'last_seen': current_time,
                'color': color
            }
            
            detected_humans[det_idx]['id'] = person_id
            detected_humans[det_idx]['color'] = color
            matched_detections.append(detected_humans[det_idx])
            
            self.next_id += 1
            self.get_logger().info(f'New track: Person {person_id}')

        return matched_detections

    def calculate_3d_position(self, x1, y1, x2, y2, depth_image):
        """
        Calculate 3D position of detected human using depth information
        
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

        # Get depth value at center (with averaging for robustness)
        h, w = depth_image.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return None

        # Average depth in a small region around center
        region_size = 10
        y_start = max(0, cy - region_size)
        y_end = min(h, cy + region_size)
        x_start = max(0, cx - region_size)
        x_end = min(w, cx + region_size)

        depth_region = depth_image[y_start:y_end, x_start:x_end]
        
        # Filter out invalid depth values (0 or NaN)
        valid_depths = depth_region[(depth_region > 0) & np.isfinite(depth_region)]
        
        if len(valid_depths) == 0:
            return None

        # Use median for robustness
        depth = np.median(valid_depths)

        # Convert depth from mm to meters (if needed, adjust based on your camera)
        if depth > 100:  # Likely in millimeters
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

    def publish_detections(self, detected_humans, header):
        """Publish detected humans as PoseArray and MarkerArray with tracking info"""
        # Debug: Log number of detected humans
        self.get_logger().info(
            f'Publishing {len(detected_humans)} humans from frame {header.frame_id} to {self.target_frame}',
            throttle_duration_sec=1.0
        )
        
        # Publish PoseArray in map frame
        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = header.stamp

        # Publish MarkerArray
        marker_array = MarkerArray()
        
        transform_success_count = 0
        transform_fail_count = 0

        for human in detected_humans:
            pos_3d = human['position_3d']
            person_id = human['id']
            color_bgr = human['color']
            
            # Convert BGR to RGB and normalize to 0-1
            color_rgb = (color_bgr[2]/255.0, color_bgr[1]/255.0, color_bgr[0]/255.0)
            
            # Create Pose in camera frame
            pose_camera = Pose()
            pose_camera.position.x = float(pos_3d[0])
            pose_camera.position.y = float(pos_3d[1])
            pose_camera.position.z = float(pos_3d[2])
            pose_camera.orientation.w = 1.0
            
            # Transform pose to map frame
            pose_map = self.transform_pose_to_map(pose_camera, header)
            
            if pose_map is not None:
                transform_success_count += 1
                pose_array.poses.append(pose_map)

                # Add text marker with person ID in map frame
                text_marker = Marker()
                text_marker.header.frame_id = self.target_frame
                text_marker.header.stamp = header.stamp
                text_marker.ns = "human_labels"
                text_marker.id = person_id + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = pose_map.position.x
                text_marker.pose.position.y = pose_map.position.y
                text_marker.pose.position.z = 0.5  # Position text above footprint
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.3
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"Person {person_id}"
                text_marker.lifetime.sec = 0
                text_marker.lifetime.nanosec = 500000000
                
                # Add footprint marker (circular disk at ground level) - always green
                footprint_marker = Marker()
                footprint_marker.header.frame_id = self.target_frame
                footprint_marker.header.stamp = header.stamp
                footprint_marker.ns = "human_footprints"
                footprint_marker.id = person_id
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
        if len(detected_humans) > 0:
            self.get_logger().info(
                f'Transform results: {transform_success_count} succeeded, {transform_fail_count} failed. '
                f'Publishing {len(pose_array.poses)} poses in {self.target_frame} frame.',
                throttle_duration_sec=1.0
            )
        
        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)

    def create_visualization(self, image, detected_humans):
        """Create visualization image with bounding boxes and tracking info"""
        viz_image = image.copy()

        for human in detected_humans:
            x1, y1, x2, y2 = human['bbox']
            pos_3d = human['position_3d']
            person_id = human['id']
            color = human['color']
            distance = np.linalg.norm(pos_3d)

            # Draw bounding box with person-specific color
            cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label with person ID and distance
            label = f"Person {person_id}: {distance:.2f}m"
            
            # Calculate text size for background
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # Draw background rectangle for text
            cv2.rectangle(
                viz_image,
                (x1, y1 - text_height - 10),
                (x1 + text_width, y1),
                color,
                -1
            )
            
            # Draw text in white
            cv2.putText(
                viz_image,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # Draw center point (red dot)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(viz_image, (cx, cy), 4, (0, 0, 255), -1)

        return viz_image

    def transform_pose_to_map(self, pose, header):
        """
        Transform pose from camera frame to map frame using TF2
        
        Args:
            pose: Pose in camera frame
            header: Original message header with camera frame and timestamp
            
        Returns:
            Pose in map frame or None if transform fails
        """
        try:
            # Create PoseStamped in camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = header.frame_id
            # Use current time (Time(0)) to get latest available transform
            # This avoids extrapolation errors when camera timestamp is ahead of TF
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose
            
            # Transform to map frame using latest available transform
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
                    f'Transform from {header.frame_id} to {self.target_frame} not available. '
                    f'Is localization/AMCL running? Set target_frame parameter to "odom" or "base_link" if map is not available.',
                    throttle_duration_sec=5.0
                )
            else:
                self.get_logger().warn(
                    f'Failed to transform pose from {header.frame_id} to {self.target_frame}: {type(e).__name__}: {e}',
                    throttle_duration_sec=0.5
                )
            return None


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
