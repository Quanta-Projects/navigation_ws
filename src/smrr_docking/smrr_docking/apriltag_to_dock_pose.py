#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
import math


class AprilTagToDockPose(Node):
    def __init__(self):
        super().__init__('apriltag_to_dock_pose')
        
        # Parameters
        self.declare_parameter('dock_tag_id', 0)  # Which AprilTag ID represents the dock
        self.declare_parameter('target_frame', 'map')  # Frame to publish dock pose in
        self.declare_parameter('camera_optical_frame', 'rear_left_camera_link_optical')
        
        self.dock_tag_id = self.get_parameter('dock_tag_id').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriber to AprilTag detections
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.detection_callback,
            10
        )
        
        # Publisher for dock pose
        self.dock_pose_pub = self.create_publisher(
            PoseStamped,
            '/dock_pose',
            10
        )
        
        self.get_logger().info(f'AprilTag to Dock Pose converter started')
        self.get_logger().info(f'Monitoring for AprilTag ID: {self.dock_tag_id}')
        self.get_logger().info(f'Publishing dock pose in frame: {self.target_frame}')

    def detection_callback(self, msg):
        """Process AprilTag detections and publish dock pose"""
        
        # Find the dock tag in detections
        dock_detection = None
        for detection in msg.detections:
            if detection.id == self.dock_tag_id:
                dock_detection = detection
                break
        
        if dock_detection is None:
            # No dock tag detected
            return
        
        # Get the transform from camera optical frame to target frame
        try:
            # The AprilTag detection is in the camera_optical_frame
            # We need to get the tag pose in the target frame (map)
            
            # Wait for transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,  # Should be rear_left_camera_link_optical
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Create PoseStamped for the detected tag
            # Note: AprilTag detection gives us the tag pose in camera optical frame
            # We need to look up the TF that was published by apriltag_ros
            tag_frame = f'tag_{self.dock_tag_id}'
            
            tag_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Create and publish dock pose
            dock_pose = PoseStamped()
            dock_pose.header.stamp = msg.header.stamp
            dock_pose.header.frame_id = self.target_frame
            
            # Use the transform data
            dock_pose.pose.position.x = tag_transform.transform.translation.x
            dock_pose.pose.position.y = tag_transform.transform.translation.y
            dock_pose.pose.position.z = tag_transform.transform.translation.z
            
            dock_pose.pose.orientation.x = tag_transform.transform.rotation.x
            dock_pose.pose.orientation.y = tag_transform.transform.rotation.y
            dock_pose.pose.orientation.z = tag_transform.transform.rotation.z
            dock_pose.pose.orientation.w = tag_transform.transform.rotation.w
            
            # Publish
            self.dock_pose_pub.publish(dock_pose)
            
            self.get_logger().info(
                f'Dock detected at: x={dock_pose.pose.position.x:.3f}, '
                f'y={dock_pose.pose.position.y:.3f}, '
                f'z={dock_pose.pose.position.z:.3f}',
                throttle_duration_sec=1.0
            )
            
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform dock pose: {ex}',
                throttle_duration_sec=2.0
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagToDockPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
