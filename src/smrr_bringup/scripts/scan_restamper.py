#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


def sanitize_qos(qos: QoSProfile) -> QoSProfile:
    if qos.history == QoSHistoryPolicy.UNKNOWN:
        qos.history = QoSHistoryPolicy.KEEP_LAST
    if qos.depth == 0:
        qos.depth = 10
    return qos


class ScanRestamper(Node):
    def __init__(self) -> None:
        super().__init__('scan_restamper')

        self.declare_parameter('input_topic', '/scan_filtered_rawstamp')
        self.declare_parameter('output_topic', '/scan')

        self._input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.publisher    = None
        self.subscription = None

        self._setup_timer = self.create_timer(1.0, self._try_setup_bridge)
        self.get_logger().info(
            f'Scan restamper initialized: {self._input_topic} -> {self._output_topic}, '
            'waiting for source topic...'
        )

    def _try_setup_bridge(self) -> None:
        if self.subscription is not None:
            self._setup_timer.cancel()
            return

        publishers = self.get_publishers_info_by_topic(self._input_topic)
        if not publishers:
            return

        qos = sanitize_qos(publishers[0].qos_profile)

        self.publisher    = self.create_publisher(LaserScan, self._output_topic, qos)
        self.subscription = self.create_subscription(LaserScan, self._input_topic, self._scan_callback, qos)

        self.get_logger().info('Scan restamper started.')

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
