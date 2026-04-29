#!/usr/bin/env python3
"""
dock_confirm_node.py

Gates the docking server's 'isDocked()' signal behind a real charging check.

The docking server (SimpleChargingDock) considers the robot docked as soon as
battery_topic reports current > charging_threshold.  Without this node that
happens on the very first charging sample — the action completes immediately
and motor commands drop to zero before the robot is fully settled.

This node:
  1. Subscribes to the REAL /battery_state topic.
  2. Requires battery.current > charging_threshold to be TRUE CONTINUOUSLY
     for `settle_time` seconds before declaring success.
  3. Only then publishes BatteryState.current=1.0 on /dock_confirmed, which
     is what docking.yaml points battery_topic at.
  4. Resets immediately if charging is interrupted OR if no battery message
     arrives within `battery_msg_timeout` seconds (watchdog).  This ensures
     that a silent topic (e.g. after undocking) cannot leave `confirmed=True`
     and cause the next docking run to skip the settle window.

If charging is interrupted at any point the timer resets.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class DockConfirmNode(Node):

    def __init__(self):
        super().__init__('dock_confirm_node')

        # Seconds of continuous charging required before declaring docked
        self.declare_parameter('settle_time', 3.0)
        # Current threshold matching charging_threshold in docking.yaml
        self.declare_parameter('charging_threshold', 0.5)
        # Real battery topic to read from
        self.declare_parameter('battery_topic', '/battery_state')
        # Watchdog: reset if no battery message received for this many seconds.
        # Must be longer than the normal battery publish interval but short
        # enough to catch a silent topic after undocking.
        self.declare_parameter('battery_msg_timeout', 2.0)

        self.settle_time = (
            self.get_parameter('settle_time').get_parameter_value().double_value
        )
        self.charging_threshold = (
            self.get_parameter('charging_threshold').get_parameter_value().double_value
        )
        battery_topic = (
            self.get_parameter('battery_topic').get_parameter_value().string_value
        )
        self.battery_msg_timeout = (
            self.get_parameter('battery_msg_timeout').get_parameter_value().double_value
        )

        # State tracking
        self.charging_since = None        # clock time when charging was first detected
        self.confirmed = False
        self.last_battery_msg_time = None  # clock time of last received battery msg

        self.battery_sub = self.create_subscription(
            BatteryState, battery_topic, self._battery_cb, 10
        )

        # Synthetic BatteryState published to /dock_confirmed for the docking server
        self.confirmed_pub = self.create_publisher(BatteryState, '/dock_confirmed', 10)

        # Watchdog timer: fires at 2 Hz to detect a silent /battery_state topic
        self.watchdog_timer = self.create_timer(0.5, self._watchdog_cb)

        self.get_logger().info(
            f'DockConfirmNode started: settle_time={self.settle_time}s, '
            f'charging_threshold={self.charging_threshold}A, '
            f'battery_msg_timeout={self.battery_msg_timeout}s, '
            f'listening on {battery_topic}'
        )

    def _battery_cb(self, msg: BatteryState) -> None:
        self.last_battery_msg_time = self.get_clock().now()
        is_charging = msg.current > self.charging_threshold
        now = self.last_battery_msg_time

        if is_charging:
            if self.charging_since is None:
                self.charging_since = now
                self.get_logger().debug(
                    f'Charging started (current={msg.current:.3f}A), starting settle timer'
                )

            elapsed = (now - self.charging_since).nanoseconds * 1e-9

            if elapsed >= self.settle_time and not self.confirmed:
                self.confirmed = True
                self.get_logger().info(
                    f'Dock confirmed: charging for {elapsed:.1f}s '
                    f'(current={msg.current:.3f}A > {self.charging_threshold}A)'
                )
        else:
            if self.charging_since is not None or self.confirmed:
                self.get_logger().debug(
                    f'Charging lost (current={msg.current:.3f}A), resetting'
                )
            self._reset()

        if self.confirmed:
            out = BatteryState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.current = 1.0   # must exceed charging_threshold in docking.yaml
            self.confirmed_pub.publish(out)

    def _watchdog_cb(self) -> None:
        """Reset state if the battery topic has gone silent."""
        if self.last_battery_msg_time is None:
            # No message ever received — nothing to reset
            return

        age = (self.get_clock().now() - self.last_battery_msg_time).nanoseconds * 1e-9
        if age > self.battery_msg_timeout:
            if self.confirmed or self.charging_since is not None:
                self.get_logger().info(
                    f'Battery topic silent for {age:.1f}s — resetting dock confirmation'
                )
            self._reset()

    def _reset(self) -> None:
        self.charging_since = None
        self.confirmed = False
        # Actively publish current=0.0 so the docking server's internal
        # is_charging_ flag gets explicitly cleared.  Without this, the
        # docking server's subscriber never receives a new message after we
        # stop publishing, leaving is_charging_=True from the previous run
        # and causing isDocked() to return True immediately on the next attempt.
        out = BatteryState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.current = 0.0
        self.confirmed_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DockConfirmNode()
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



def main(args=None):
    rclpy.init(args=args)
    node = DockConfirmNode()
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
