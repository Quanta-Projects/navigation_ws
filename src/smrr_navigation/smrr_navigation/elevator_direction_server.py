#!/usr/bin/env python3
"""
ROS 2 Action Server for elevator direction verification via YOLO + OpenCV HSV analysis.

Action: smrr_interfaces/action/CheckElevatorDirection
  Goal:     current_floor (str), target_floor (str)   e.g. "floor0", "floor2"
  Feedback: expected_direction (str), status (str), ratio (f32)
  Result:   correct_direction_confirmed (bool), message (str)

The server monitors the elevator arrow panel through the camera feed.
  - Arrow ON  (illuminated)  → elevator moving in that direction
  - Arrow OFF (extinguished) → direction confirmed (after 100 ms stability)

Resource optimization: the camera subscriber is created on goal start and
destroyed on goal completion/cancellation to save bandwidth and CPU.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
from ultralytics import YOLO

from smrr_interfaces.action import CheckElevatorDirection


# --------------- configuration ---------------
MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'yolo_button_detection.pt',
)

HSV_LOWER           = np.array([5, 50, 150])
HSV_UPPER           = np.array([35, 255, 255])
ON_RATIO_THRESHOLD  = 0.04
OFF_STABLE_DURATION = 0.1    # seconds the arrow must stay OFF
YOLO_CONF           = 0.1
LOOP_SLEEP          = 0.016  # ~60 Hz
# ---------------------------------------------


class ElevatorDirectionServer(Node):

    def __init__(self):
        super().__init__('elevator_direction_server')
        self.cb_group = ReentrantCallbackGroup()

        # ---- camera state (no global subscription — created per-goal) ----
        self.latest_image: Image | None = None
        self.image_sub = None     # created in _execute_cb, destroyed in finally
        self.bridge = CvBridge()

        # ---- YOLO model ----
        self.model = YOLO(MODEL_PATH)

        # ---- debug publisher (always active) ----
        self.debug_pub = self.create_publisher(
            Image, '/floor_vision/debug_image', 1)

        # ---- action server ----
        self._action_server = ActionServer(
            self,
            CheckElevatorDirection,
            'check_elevator_direction',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            'ElevatorDirectionServer ready — action: check_elevator_direction')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        self.latest_image = msg

    @staticmethod
    def _goal_cb(_goal_request):
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_cb(_goal_handle):
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_debug(self, cv_image: np.ndarray) -> None:
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        except Exception:
            pass

    @staticmethod
    def _overlay(img, text, color=(200, 200, 200), y=30):
        cv2.putText(img, text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(img, text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    @staticmethod
    def _parse_floor_index(floor_str: str) -> int:
        """Extract integer index from floor string, e.g. 'floor2' → 2."""
        cleaned = floor_str.strip().lower().replace('floor', '')
        return int(cleaned)

    # ------------------------------------------------------------------
    # Action execution
    # ------------------------------------------------------------------
    def _execute_cb(self, goal_handle):
        raw_current = goal_handle.request.current_floor.strip()
        raw_target  = goal_handle.request.target_floor.strip()

        result   = CheckElevatorDirection.Result()
        feedback = CheckElevatorDirection.Feedback()

        # ── Direction math ─────────────────────────────────────────────
        try:
            current_idx = self._parse_floor_index(raw_current)
            target_idx  = self._parse_floor_index(raw_target)
        except ValueError:
            self.get_logger().error(
                f'[CheckElevatorDirection] Cannot parse floors: '
                f'current="{raw_current}", target="{raw_target}"')
            goal_handle.abort()
            result.correct_direction_confirmed = False
            result.message = (
                f'Invalid floor strings: "{raw_current}", "{raw_target}"')
            return result

        if target_idx > current_idx:
            expected_direction = 'UP'
            target_class       = 'up'
        else:
            expected_direction = 'DOWN'
            target_class       = 'down'

        self.get_logger().info(
            f'[CheckElevatorDirection] {raw_current} → {raw_target}: '
            f'expecting {expected_direction} ({target_class})')

        # ── Dynamic subscription — created here, destroyed in finally ──
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image,
            '/zed2_left_camera/image_raw',
            self._image_cb,
            1,
            callback_group=self.cb_group,
        )

        try:
            # ── Detection loop ──────────────────────────────────────────
            off_start_time = None

            while rclpy.ok() and goal_handle.is_active:
                time.sleep(LOOP_SLEEP)

                # cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.correct_direction_confirmed = False
                    result.message = 'Goal canceled by client.'
                    return result

                # wait for first frame
                if self.latest_image is None:
                    continue

                # convert frame
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(
                        self.latest_image, desired_encoding='bgr8')
                except Exception:
                    continue

                self._overlay(
                    cv_image,
                    f'Target: {target_class}',
                    color=(255, 255, 0))

                # YOLO inference (fresh detections only)
                results = self.model(cv_image, conf=YOLO_CONF, verbose=False)
                working_bbox = None

                for res in results:
                    for box in res.boxes:
                        cls_id   = int(box.cls[0])
                        cls_name = self.model.names.get(cls_id, '')
                        if cls_name == target_class:
                            working_bbox = tuple(
                                map(int, box.xyxy[0].tolist()))
                            break
                    if working_bbox is not None:
                        break

                if working_bbox is None:
                    self._overlay(cv_image,
                                  f'{target_class} not visible',
                                  color=(0, 0, 255), y=60)
                    feedback.expected_direction = expected_direction
                    feedback.status = 'NO_DETECTION'
                    feedback.ratio  = 0.0
                    goal_handle.publish_feedback(feedback)
                    self._publish_debug(cv_image)
                    continue

                # HSV analysis on the detected bounding box
                x1, y1, x2, y2 = working_bbox
                crop = cv_image[y1:y2, x1:x2]

                if crop.size == 0:
                    continue

                hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                mask  = cv2.inRange(hsv_crop, HSV_LOWER, HSV_UPPER)
                ratio = float(np.count_nonzero(mask) / mask.size)

                # PiP mask overlay (top-right corner)
                mask_bgr     = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                mask_resized = cv2.resize(mask_bgr, (150, 150),
                                          interpolation=cv2.INTER_NEAREST)
                h_m, w_m = mask_resized.shape[:2]
                cv_image[0:h_m, cv_image.shape[1]-w_m:] = mask_resized
                cv2.rectangle(cv_image,
                              (cv_image.shape[1]-w_m, 0),
                              (cv_image.shape[1], h_m), (0, 255, 0), 2)
                cv2.putText(cv_image, 'HSV MASK',
                            (cv_image.shape[1]-w_m+10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Decision logic — identical thresholds to FloorArrivalServer
                if ratio >= ON_RATIO_THRESHOLD:
                    state_str      = f'ON (Ratio: {ratio:.2f})'
                    box_color      = (0, 0, 255)
                    off_start_time = None
                    feedback.expected_direction = expected_direction
                    feedback.status = 'ON'
                    feedback.ratio  = ratio
                else:
                    state_str = f'OFF (Ratio: {ratio:.2f})'
                    box_color = (0, 255, 0)

                    if off_start_time is None:
                        off_start_time = (
                            self.get_clock().now().nanoseconds / 1e9)
                        feedback.expected_direction = expected_direction
                        feedback.status = 'OFF_UNSTABLE'
                        feedback.ratio  = ratio
                    else:
                        elapsed = (
                            self.get_clock().now().nanoseconds / 1e9
                            - off_start_time)
                        if elapsed >= OFF_STABLE_DURATION:
                            # ── DIRECTION CONFIRMED ──
                            feedback.expected_direction = expected_direction
                            feedback.status = 'OFF_STABLE'
                            feedback.ratio  = ratio
                            goal_handle.publish_feedback(feedback)

                            cv2.rectangle(cv_image, (x1, y1), (x2, y2),
                                          box_color, 2)
                            cv2.putText(cv_image, state_str,
                                        (x1, max(y1 - 8, 0)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                        box_color, 2)
                            self._publish_debug(cv_image)

                            goal_handle.succeed()
                            result.correct_direction_confirmed = True
                            result.message = (
                                f'{expected_direction} ({target_class}) confirmed '
                                f'({raw_current}→{raw_target}, '
                                f'ratio {ratio:.3f}, stable {elapsed:.2f}s).')
                            return result
                        else:
                            feedback.expected_direction = expected_direction
                            feedback.status = 'OFF_UNSTABLE'
                            feedback.ratio  = ratio

                # annotate and publish every cycle
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(cv_image, state_str, (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
                goal_handle.publish_feedback(feedback)
                self._publish_debug(cv_image)

            # loop exited without success (node shutdown)
            result.correct_direction_confirmed = False
            result.message = 'Loop exited without stable OFF detection.'
            return result

        finally:
            # ── CRITICAL: always destroy the subscription ───────────────
            # Fires whether the action succeeds, is cancelled, aborts,
            # or raises — ensuring the camera stream is released.
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub    = None
                self.latest_image = None
                self.get_logger().info(
                    '[CheckElevatorDirection] Camera subscription destroyed.')


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ElevatorDirectionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
