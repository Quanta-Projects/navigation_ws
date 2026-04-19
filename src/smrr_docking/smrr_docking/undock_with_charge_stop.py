"""
Undock wrapper node.

When an UndockRobot action goal is received on /undock_robot_safe:
  1. Publishes /disable_charging = True  (tells base controller to stop charging)
  2. Waits for charging to actually stop (monitors /battery_state)
  3. Holds for a configurable settle time (default 5 s)
  4. Forwards the goal to the real /undock_robot action
  5. On completion, publishes /disable_charging = False

Uses MultiThreadedExecutor + synchronous polling (no async/await) for
reliable operation on ROS2 Humble.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from opennav_docking_msgs.action import UndockRobot
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class UndockWithChargeStop(Node):
    def __init__(self):
        super().__init__('undock_with_charge_stop')

        self.declare_parameter('settle_time', 5.0)
        self.declare_parameter('charge_stop_timeout', 10.0)
        self.settle_time = self.get_parameter('settle_time').value
        self.charge_stop_timeout = self.get_parameter('charge_stop_timeout').value

        cb_group = ReentrantCallbackGroup()

        # Publisher to tell base controller to stop charging
        self.disable_pub = self.create_publisher(Bool, '/disable_charging', 10)

        # Monitor battery state to confirm charging has stopped
        self.is_charging = False
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self._battery_cb, 10,
            callback_group=cb_group)

        # Action client to forward to real docking server
        self.undock_client = ActionClient(
            self, UndockRobot, 'undock_robot', callback_group=cb_group)

        # Action server — synchronous execute callback (not async)
        self.undock_server = ActionServer(
            self, UndockRobot, 'undock_robot_safe',
            self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb_group)

        self.get_logger().info(
            'UndockWithChargeStop ready — call /undock_robot_safe '
            f'(settle={self.settle_time}s, timeout={self.charge_stop_timeout}s)')

    def _battery_cb(self, msg: BatteryState):
        self.is_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING)

    def _execute_cb(self, goal_handle):
        """Synchronous execute callback — uses polling instead of await."""
        self.get_logger().info('Undock requested — stopping charging first')

        try:
            # Step 1: command charging off
            msg = Bool()
            msg.data = True
            self.disable_pub.publish(msg)
            self.get_logger().info('Published /disable_charging = True')

            # Step 2: wait for charging to stop (polling)
            start = time.monotonic()
            while self.is_charging:
                if time.monotonic() - start > self.charge_stop_timeout:
                    self.get_logger().warn(
                        'Timed out waiting for charging to stop — proceeding anyway')
                    break
                time.sleep(0.1)
            elapsed = time.monotonic() - start
            self.get_logger().info(f'Charging stopped after {elapsed:.1f}s')

            # Step 3: settle time
            remaining = max(0.0, self.settle_time - elapsed)
            if remaining > 0:
                self.get_logger().info(f'Settling for {remaining:.1f}s')
                time.sleep(remaining)

            # Step 4: forward to real undock action
            self.get_logger().info('Sending undock goal to docking server')
            if not self.undock_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Undock action server not available')
                goal_handle.abort()
                result = UndockRobot.Result()
                result.success = False
                return result

            # Build goal
            fwd_goal = UndockRobot.Goal()
            fwd_goal.dock_type = goal_handle.request.dock_type
            fwd_goal.max_undocking_time = goal_handle.request.max_undocking_time

            # Send goal and poll until accepted
            goal_future = self.undock_client.send_goal_async(fwd_goal)
            while not goal_future.done():
                time.sleep(0.05)

            client_goal_handle = goal_future.result()
            if not client_goal_handle.accepted:
                self.get_logger().error('Undock goal rejected by docking server')
                goal_handle.abort()
                result = UndockRobot.Result()
                result.success = False
                return result

            self.get_logger().info('Undock goal accepted — waiting for completion')

            # Poll until result is available
            result_future = client_goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.1)

            action_result = result_future.result().result

            if action_result.success:
                self.get_logger().info('Undocking succeeded')
                goal_handle.succeed()
            else:
                self.get_logger().warn(
                    f'Undocking failed (error_code={action_result.error_code})')
                goal_handle.abort()

            return action_result

        except Exception as e:
            self.get_logger().error(f'Undock wrapper error: {e}')
            goal_handle.abort()
            result = UndockRobot.Result()
            result.success = False
            return result

        finally:
            # ALWAYS release the charging override
            self._release_charging()

    def _release_charging(self):
        msg = Bool()
        msg.data = False
        self.disable_pub.publish(msg)
        self.get_logger().info('Published /disable_charging = False')


def main(args=None):
    rclpy.init(args=args)
    node = UndockWithChargeStop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
