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
from visualization_msgs.msg import Marker
from dr_spaam.detector import Detector
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class DrSpaamNode(Node):
    """ROS 2 node to detect pedestrians using DROW3 or DR-SPAAM."""

    def __init__(self):
        super().__init__('dr_spaam_node')

        # Declare parameters
        self.declare_parameter('weight_file', '')
        self.declare_parameter('conf_thresh', 0.8)
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

        # Subscriber
        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            10
        )

        self.get_logger().info('DR-SPAAM Node initialized')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing detections to: {detections_topic}')
        self.get_logger().info(f'Publishing markers to: {marker_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

    def _scan_callback(self, msg):
        """Process incoming laser scan and detect people"""
        # Skip if no subscribers
        if (self._dets_pub.get_subscription_count() == 0 and 
            self._rviz_pub.get_subscription_count() == 0):
            return

        # Set laser FOV if not set
        if not self._detector.is_ready():
            fov_deg = np.rad2deg(msg.angle_increment * len(msg.ranges))
            self._detector.set_laser_fov(fov_deg)
            self.get_logger().info(f'Laser FOV set to: {fov_deg:.2f} degrees')

        # Prepare scan data
        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # Run detection
        dets_xy, dets_cls, _ = self._detector(scan)

        # Apply confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # Debug logging
        self.get_logger().info(
            f'Detected {len(dets_xy)} people',
            throttle_duration_sec=1.0
        )

        # Convert and publish detections
        dets_msg = self._detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        
        # Transform to target frame if specified (optimized - single lookup)
        if self.target_frame and self.target_frame != msg.header.frame_id:
            dets_msg = self._transform_pose_array_to_target_frame(dets_msg)
        
        # Pack confidence into position.z AFTER frame transform so tf2
        # quaternion math cannot corrupt it (orientation is identity).
        for pose, d_cls in zip(dets_msg.poses, dets_cls):
            pose.position.z = float(d_cls)
        
        self._dets_pub.publish(dets_msg)

        # Publish RViz markers (only if there are subscribers)
        if self._rviz_pub.get_subscription_count() > 0:
            rviz_msg = self._detections_to_rviz_marker(dets_xy, dets_cls)
            rviz_msg.header = msg.header
            
            # Transform markers to target frame if specified
            if self.target_frame and self.target_frame != msg.header.frame_id:
                rviz_msg = self._transform_marker_to_target_frame(rviz_msg)
            
            self._rviz_pub.publish(rviz_msg)

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
        """Convert detections to PoseArray message"""
        pose_array = PoseArray()
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            # Detector frame convention: x forward, y rightward, z downward
            p = Pose()
            p.position.x = float(d_xy[0])
            p.position.y = float(d_xy[1])
            p.position.z = 0.0  # Confidence packed into position.z after frame transform
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        return pose_array

    def _transform_pose_array_to_target_frame(self, pose_array):
        """Transform PoseArray to target frame using TF2 efficiently"""
        try:
            # Use the sensor message timestamp to anchor the transform to the
            # moment the scan was captured, preventing ego-motion latency.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                pose_array.header.frame_id,
                rclpy.time.Time.from_msg(pose_array.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Transform each pose using the same transform
            transformed_poses = []
            for pose in pose_array.poses:
                # Create PoseStamped for transformation
                pose_stamped = PoseStamped()
                pose_stamped.header = pose_array.header
                pose_stamped.pose = pose
                
                # Apply transform
                transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_stamped,
                    transform
                )
                
                transformed_poses.append(transformed_pose_stamped.pose)
            
            # Create new PoseArray with transformed poses
            transformed_pose_array = PoseArray()
            transformed_pose_array.header.stamp = pose_array.header.stamp
            transformed_pose_array.header.frame_id = self.target_frame
            transformed_pose_array.poses = transformed_poses
            
            return transformed_pose_array
            
        except Exception as e:
            self.get_logger().warn(f'Failed to transform pose array: {e}', throttle_duration_sec=5.0)
            return pose_array

    def _transform_marker_to_target_frame(self, marker):
        """Transform marker points to target frame efficiently"""
        try:
            # Use the sensor message timestamp to anchor the transform to the
            # moment the scan was captured, preventing ego-motion latency.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                marker.header.frame_id,
                rclpy.time.Time.from_msg(marker.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Transform each point using the same transform
            transformed_points = []
            for point in marker.points:
                # Create PoseStamped for transformation
                point_stamped = PoseStamped()
                point_stamped.header = marker.header
                point_stamped.pose.position = point
                point_stamped.pose.orientation.w = 1.0
                
                # Apply transform
                transformed_point_stamped = tf2_geometry_msgs.do_transform_pose_stamped(
                    point_stamped,
                    transform
                )
                
                transformed_points.append(transformed_point_stamped.pose.position)
            
            # Update marker with transformed points
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
