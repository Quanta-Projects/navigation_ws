import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_geometry_msgs  # Essential for transform conversions

class TfToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')

        # --- Parameters ---
        # The frame ID of the detected tag (published by apriltag_ros)
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.tag_frame = self.get_parameter('tag_frame').get_parameter_value().string_value

        # The reference frame for navigation (usually 'odom' or 'map')
        self.declare_parameter('reference_frame', 'odom')
        self.ref_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

        # Maximum age (seconds) of a TF before we stop publishing it.
        # This ensures the docking server's external_detection_timeout can fire
        # when the tag goes out of view during close approach.
        self.declare_parameter('max_tf_age', 0.5)
        self.max_tf_age = self.get_parameter('max_tf_age').get_parameter_value().double_value

        # --- TF Listener Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        # Publishes the pose for the docking server
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)

        # --- Timer ---
        # Check for the tag 10 times a second
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Bridge Node Started. Listening for {self.tag_frame} -> {self.ref_frame}")

    def timer_callback(self):
        try:
            # Look up the transform from the Tag to the Reference Frame (Odom/Map)
            # We use Time(seconds=0) to get the latest available transform
            transform = self.tf_buffer.lookup_transform(
                self.ref_frame,
                self.tag_frame,
                rclpy.time.Time()
            )

            # Reject stale transforms: compute age from the transform's own stamp.
            # Using now() as the stamp (old behaviour) defeats external_detection_timeout
            # in the docking server because every message appears fresh.
            tf_stamp = Time.from_msg(transform.header.stamp)
            age_sec = (self.get_clock().now() - tf_stamp).nanoseconds * 1e-9
            if age_sec > self.max_tf_age:
                # Tag not recently visible — let the docking server time out naturally
                return

            # Create the PoseStamped message
            pose_msg = PoseStamped()
            # Use the ACTUAL transform timestamp so the docking server's
            # external_detection_timeout correctly ages out stale detections.
            pose_msg.header.stamp = transform.header.stamp
            pose_msg.header.frame_id = self.ref_frame

            # Populate position
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z

            # Populate orientation
            pose_msg.pose.orientation = transform.transform.rotation

            # Publish
            self.pose_pub.publish(pose_msg)

        except Exception:
            # It is normal to see errors if the tag is not currently visible
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TfToPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()