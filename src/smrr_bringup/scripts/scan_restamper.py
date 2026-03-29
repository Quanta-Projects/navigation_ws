#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__('scan_restamper')

        self.declare_parameter('input_topic', '/scan_filtered_rawstamp')
        self.declare_parameter('output_topic', '/scan')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            10,
        )

        self.get_logger().info(
            f'Restamping LaserScan from {input_topic} to {output_topic} using current ROS time'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRestamper()
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