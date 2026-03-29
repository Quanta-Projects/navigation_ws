#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ZedTopicRestamper(Node):
    def __init__(self) -> None:
        super().__init__('zed_topic_restamper')

        self._latest_rgb_stamp = None

        self.declare_parameter('rgb_input_topic', '/zed2/zed_node/rgb/color/rect/image')
        self.declare_parameter('rgb_output_topic', '/zed2_left_camera/image_raw')
        self.declare_parameter('depth_input_topic', '/zed2/zed_node/depth/depth_registered')
        self.declare_parameter('depth_output_topic', '/zed2_left_camera/depth/image_raw')
        self.declare_parameter('camera_info_input_topic', '/zed2/zed_node/rgb/color/rect/camera_info')
        self.declare_parameter('camera_info_output_topic', '/zed2_left_camera/camera_info')

        rgb_input = self.get_parameter('rgb_input_topic').get_parameter_value().string_value
        rgb_output = self.get_parameter('rgb_output_topic').get_parameter_value().string_value
        depth_input = self.get_parameter('depth_input_topic').get_parameter_value().string_value
        depth_output = self.get_parameter('depth_output_topic').get_parameter_value().string_value
        info_input = self.get_parameter('camera_info_input_topic').get_parameter_value().string_value
        info_output = self.get_parameter('camera_info_output_topic').get_parameter_value().string_value

        self.rgb_pub = self.create_publisher(Image, rgb_output, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_output, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, info_output, qos_profile_sensor_data)

        self.rgb_sub = self.create_subscription(
            Image,
            rgb_input,
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_input,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            info_input,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info('ZED topic restamper started with current ROS time stamping')

    def _rgb_callback(self, msg: Image) -> None:
        rgb_stamp = self.get_clock().now().to_msg()
        msg.header.stamp = rgb_stamp
        self._latest_rgb_stamp = rgb_stamp
        self.rgb_pub.publish(msg)

    def _depth_callback(self, msg: Image) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._latest_rgb_stamp is not None:
            msg.header.stamp = self._latest_rgb_stamp
        else:
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