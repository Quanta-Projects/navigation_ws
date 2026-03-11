#!/usr/bin/env python3
"""
AprilTag Manager Server
========================
ROS 2 service server that dynamically starts and stops the `apriltag_ros`
node process to save CPU.

Service:  /toggle_apriltag  (std_srvs/srv/SetBool)
  data=true  → launch apriltag_node in the background
  data=false → terminate the process group cleanly
"""

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


APRILTAG_CMD = [
    'ros2', 'run', 'apriltag_ros', 'apriltag_node',
    '--ros-args',
    '-r', 'image_rect:=/zed2_left_camera/image_raw',
    '-r', 'camera_info:=/zed2_left_camera/camera_info',
    '-p', 'family:=36h11',
    '-p', 'size:=0.15',
]


class AprilTagManagerServer(Node):
    def __init__(self):
        super().__init__('apriltag_manager_server')
        self._process = None

        self._srv = self.create_service(
            SetBool,
            '/toggle_apriltag',
            self._handle_toggle,
        )
        self.get_logger().info(
            'AprilTagManagerServer ready — service /toggle_apriltag active.'
        )

    # ------------------------------------------------------------------
    def _handle_toggle(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            response.success, response.message = self._start_node()
        else:
            response.success, response.message = self._stop_node()
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def _start_node(self):
        if self._process is not None and self._process.poll() is None:
            return True, 'apriltag_node is already running (pid={}).'.format(
                self._process.pid
            )

        try:
            self._process = subprocess.Popen(
                APRILTAG_CMD,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                # Run in its own process-group session so that the entire
                # process tree can be killed with os.killpg later.
                preexec_fn=os.setsid,
            )
            return True, 'apriltag_node started (pid={}).'.format(self._process.pid)
        except Exception as exc:  # noqa: BLE001
            self._process = None
            return False, 'Failed to start apriltag_node: {}'.format(exc)

    # ------------------------------------------------------------------
    def _stop_node(self):
        if self._process is None or self._process.poll() is not None:
            self._process = None
            return True, 'apriltag_node was not running — nothing to stop.'

        try:
            pgid = os.getpgid(self._process.pid)
            os.killpg(pgid, signal.SIGTERM)
            self._process.wait(timeout=5.0)
            self._process = None
            return True, 'apriltag_node terminated cleanly.'
        except ProcessLookupError:
            self._process = None
            return True, 'apriltag_node process was already gone.'
        except subprocess.TimeoutExpired:
            # Force-kill if SIGTERM did not work within 5 s
            try:
                pgid = os.getpgid(self._process.pid)
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._process = None
            return True, 'apriltag_node was SIGKILLed after SIGTERM timeout.'
        except Exception as exc:  # noqa: BLE001
            self._process = None
            return False, 'Error stopping apriltag_node: {}'.format(exc)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagManagerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_node()  # ensure cleanup on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
