#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image


def sanitize_qos(qos: QoSProfile) -> QoSProfile:
    if qos.history == QoSHistoryPolicy.UNKNOWN:
        qos.history = QoSHistoryPolicy.KEEP_LAST
    if qos.depth == 0:
        qos.depth = 10
    return qos


class ZedTopicRestamper(Node):
    def __init__(self) -> None:
        super().__init__('zed_topic_restamper')

        self.declare_parameter('rgb_input_topic', '/zed2/zed_node/rgb/color/rect/image')
        self.declare_parameter('rgb_output_topic', '/zed2_left_camera/image_raw')
        self.declare_parameter('depth_input_topic', '/zed2/zed_node/depth/depth_registered')
        self.declare_parameter('depth_output_topic', '/zed2_left_camera/depth/image_raw')
        self.declare_parameter('camera_info_input_topic', '/zed2/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('camera_info_output_topic', '/zed2_left_camera/camera_info')

        self._rgb_input   = self.get_parameter('rgb_input_topic').get_parameter_value().string_value
        self._rgb_output  = self.get_parameter('rgb_output_topic').get_parameter_value().string_value
        self._depth_input  = self.get_parameter('depth_input_topic').get_parameter_value().string_value
        self._depth_output = self.get_parameter('depth_output_topic').get_parameter_value().string_value
        self._info_input   = self.get_parameter('camera_info_input_topic').get_parameter_value().string_value
        self._info_output  = self.get_parameter('camera_info_output_topic').get_parameter_value().string_value

        self.rgb_pub   = None
        self.depth_pub = None
        self.info_pub  = None
        self.rgb_sub   = None
        self.depth_sub = None
        self.info_sub  = None

        self._setup_timer = self.create_timer(1.0, self._try_setup_bridges)
        self.get_logger().info('ZED topic restamper initialized, waiting for source topics...')

    def _qos_from_topic(self, topic_name: str) -> QoSProfile | None:
        publishers = self.get_publishers_info_by_topic(topic_name)
        if not publishers:
            return None
        return sanitize_qos(publishers[0].qos_profile)

    def _try_setup_bridges(self) -> None:
        if self.rgb_sub is not None and self.depth_sub is not None and self.info_sub is not None:
            self._setup_timer.cancel()
            return

        rgb_qos   = self._qos_from_topic(self._rgb_input)
        depth_qos = self._qos_from_topic(self._depth_input)
        info_qos  = self._qos_from_topic(self._info_input)

        if rgb_qos is None or depth_qos is None or info_qos is None:
            return

        if self.rgb_sub is None:
            self.rgb_pub = self.create_publisher(Image, self._rgb_output, rgb_qos)
            self.rgb_sub = self.create_subscription(Image, self._rgb_input, self._rgb_callback, rgb_qos)

        if self.depth_sub is None:
            self.depth_pub = self.create_publisher(Image, self._depth_output, depth_qos)
            self.depth_sub = self.create_subscription(Image, self._depth_input, self._depth_callback, depth_qos)

        if self.info_sub is None:
            self.info_pub = self.create_publisher(CameraInfo, self._info_output, info_qos)
            self.info_sub = self.create_subscription(CameraInfo, self._info_input, self._camera_info_callback, info_qos)

        self.get_logger().info('ZED topic restamper started.')

    def _rgb_callback(self, msg: Image) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_pub.publish(msg)

    def _depth_callback(self, msg: Image) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedTopicRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
