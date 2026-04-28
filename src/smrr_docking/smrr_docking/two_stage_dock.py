"""
Two-Stage Docking Orchestrator.

Exposes /dock_robot (DockRobot action) to external callers and internally chains:
  Stage 1 → /stage1/dock_robot  (high k_phi/k_delta, stops at 3/5 total distance)
  Stage 2 → /stage2/dock_robot  (low  k_phi/k_delta, full contact + battery check)

Callers send a DockRobot goal to /dock_robot exactly as they would to the stock
docking server. The orchestrator handles routing to the two namespaced servers.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from opennav_docking_msgs.action import DockRobot


class TwoStageDock(Node):
    def __init__(self):
        super().__init__('two_stage_dock')
        cb = ReentrantCallbackGroup()

        self._stage1_client = ActionClient(
            self, DockRobot, '/stage1/dock_robot', callback_group=cb)
        self._stage2_client = ActionClient(
            self, DockRobot, '/stage2/dock_robot', callback_group=cb)

        self._server = ActionServer(
            self, DockRobot, 'dock_robot',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            'TwoStageDock ready — send DockRobot goals to /dock_robot')

    # ------------------------------------------------------------------
    def _call_stage(self, client, goal, label, goal_handle):
        """
        Send a DockRobot goal to a namespaced server, block until result.
        Returns the Result object on success, or None on failure/cancel.
        """
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{label}: action server unavailable')
            return None

        done = threading.Event()
        result_box = [None]

        def _on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f'{label}: goal rejected')
                done.set()
                return

            def _on_result(res_future):
                result_box[0] = res_future.result().result
                done.set()

            gh.get_result_async().add_done_callback(_on_result)

        client.send_goal_async(goal).add_done_callback(_on_goal)

        # Poll until done, checking for cancellation from the caller
        while not done.wait(timeout=0.2):
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'{label}: cancelled by caller')
                return None

        return result_box[0]

    # ------------------------------------------------------------------
    def _execute(self, goal_handle):
        req = goal_handle.request

        # ---- Stage 1: coarse approach --------------------------------
        self.get_logger().info(
            'Stage 1 starting — coarse approach '
            '(k_phi=3.0, k_delta=2.5); '
            'robot will stop at 3/5 of staging distance (0.64 m from dock)')

        g1 = DockRobot.Goal()
        g1.use_dock_id              = req.use_dock_id
        g1.dock_id                  = req.dock_id
        g1.dock_pose                = req.dock_pose
        g1.dock_type                = req.dock_type
        g1.navigate_to_staging_pose = req.navigate_to_staging_pose
        g1.max_staging_time         = req.max_staging_time

        r1 = self._call_stage(self._stage1_client, g1, 'Stage1', goal_handle)
        if r1 is None or not r1.success:
            error_code = getattr(r1, 'error_code', 'N/A') if r1 is not None else 'N/A'
            self.get_logger().error(
                f'Stage 1 failed (error_code={error_code}) — aborting')
            goal_handle.abort()
            result = DockRobot.Result()
            result.success = False
            return result

        self.get_logger().info(
            'Stage 1 complete — robot is at 3/5 of staging distance (0.64 m from dock)')

        # ---- Stage 2: fine approach + battery confirmation -----------
        self.get_logger().info(
            'Stage 2 starting — fine approach '
            '(k_phi=0.8, k_delta=0.6); '
            'confirms docking via battery charging current')

        g2 = DockRobot.Goal()
        g2.use_dock_id              = req.use_dock_id
        g2.dock_id                  = req.dock_id
        g2.dock_pose                = req.dock_pose
        g2.dock_type                = req.dock_type
        g2.navigate_to_staging_pose = False   # robot already near dock after Stage 1
        g2.max_staging_time         = req.max_staging_time

        r2 = self._call_stage(self._stage2_client, g2, 'Stage2', goal_handle)
        if r2 is None or not r2.success:
            error_code = getattr(r2, 'error_code', 'N/A') if r2 is not None else 'N/A'
            self.get_logger().error(
                f'Stage 2 failed (error_code={error_code}) — aborting')
            goal_handle.abort()
            result = DockRobot.Result()
            result.success = False
            return result

        self.get_logger().info(
            'Two-stage docking complete — robot is charging')
        goal_handle.succeed()
        return r2


def main(args=None):
    rclpy.init(args=args)
    node = TwoStageDock()
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


if __name__ == '__main__':
    main()
