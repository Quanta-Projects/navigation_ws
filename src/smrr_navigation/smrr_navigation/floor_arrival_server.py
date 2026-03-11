#!/usr/bin/env python3
"""
ROS 2 Action Server for floor arrival verification via YOLO + OpenCV HSV analysis.

Action: smrr_interfaces/action/CheckFloorArrival
  Goal:     target_floor (str)        – e.g. "3", "G"
  Feedback: status (str), ratio (f32) – per-cycle state
  Result:   arrived (bool), message (str)

The server monitors the elevator button panel through the camera feed.
  - Button ON  (illuminated)  → elevator still moving
  - Button OFF (not illuminated) → floor reached (after 100 ms stability)
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

from smrr_interfaces.action import CheckFloorArrival


# --------------- configuration ---------------
MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'yolo_button_detection.pt',
)

HSV_LOWER = np.array([5, 50, 150])
HSV_UPPER = np.array([35, 255, 255])
ON_RATIO_THRESHOLD  = 0.04
OFF_STABLE_DURATION = 0.1          # seconds the button must stay OFF
YOLO_CONF           = 0.1
LOOP_SLEEP          = 0.016        # ~60 Hz
# ---------------------------------------------


class FloorArrivalServer(Node):
    def __init__(self):
        super().__init__('floor_arrival_server')
        self.cb_group = ReentrantCallbackGroup()

        # ---- camera state ----
        self.latest_image: Image | None = None
        self.bridge = CvBridge()

        # ---- YOLO model ----
        self.model = YOLO(MODEL_PATH)

        # ---- subscribers / publishers ----
        self.create_subscription(
            Image, '/zed2_left_camera/image_raw', self._image_cb, 1,
            callback_group=self.cb_group,
        )
        self.debug_pub = self.create_publisher(Image, '/floor_vision/debug_image', 1)

        # ---- action server ----
        self._action_server = ActionServer(
            self,
            CheckFloorArrival,
            'check_floor_arrival',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )

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

    # ------------------------------------------------------------------
    # Action execution
    # ------------------------------------------------------------------
    def _execute_cb(self, goal_handle):
        raw_floor = goal_handle.request.target_floor.strip()

        # Map BT blackboard IDs (e.g. "floor3") to YOLO class labels (e.g. "3", "g")
        floor_mapping = {
            'floor0': 'g',
            'floor1': '1',
            'floor2': '2',
            'floor3': '3',
        }
        mapped_floor = floor_mapping.get(raw_floor, raw_floor)
        target_class = f'button-{mapped_floor}'

        feedback = CheckFloorArrival.Feedback()
        result   = CheckFloorArrival.Result()

        # ---- stability timer ----
        off_start_time = None

        while rclpy.ok() and goal_handle.is_active:
            time.sleep(LOOP_SLEEP)

            # --- cancellation ---
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.arrived = False
                result.message = 'Goal canceled by client.'
                return result

            # --- wait for image ---
            if self.latest_image is None:
                continue

            # --- convert frame ---
            try:
                cv_image = self.bridge.imgmsg_to_cv2(
                    self.latest_image, desired_encoding='bgr8')
            except Exception:
                continue

            self._overlay(cv_image, f'Target: {target_class}',
                          color=(255, 255, 0))

            # --- YOLO inference (fresh detections only) ---
            results = self.model(cv_image, conf=YOLO_CONF, verbose=False)
            working_bbox = None

            for res in results:
                for box in res.boxes:
                    cls_id   = int(box.cls[0])
                    cls_name = self.model.names.get(cls_id, '')
                    if cls_name == target_class:
                        working_bbox = tuple(map(int, box.xyxy[0].tolist()))
                        break

            if working_bbox is None:
                self._overlay(cv_image,
                              'Target button not visible',
                              color=(0, 0, 255), y=60)
                feedback.status = 'NO_DETECTION'
                feedback.ratio  = 0.0
                goal_handle.publish_feedback(feedback)
                self._publish_debug(cv_image)
                continue

            # --- HSV analysis ---
            x1, y1, x2, y2 = working_bbox
            crop = cv_image[y1:y2, x1:x2]

            if crop.size == 0:
                continue

            hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mask  = cv2.inRange(hsv_crop, HSV_LOWER, HSV_UPPER)
            ratio = float(np.count_nonzero(mask) / mask.size)

            # --- PiP mask overlay ---
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

            # --- decision logic ---
            if ratio >= ON_RATIO_THRESHOLD:
                state_str = f'ON (Ratio: {ratio:.2f})'
                box_color = (0, 0, 255)
                off_start_time = None
                feedback.status = 'ON'
                feedback.ratio  = ratio
            else:
                state_str = f'OFF (Ratio: {ratio:.2f})'
                box_color = (0, 255, 0)

                if off_start_time is None:
                    off_start_time = self.get_clock().now().nanoseconds / 1e9
                    feedback.status = 'OFF_UNSTABLE'
                    feedback.ratio  = ratio
                else:
                    elapsed = self.get_clock().now().nanoseconds / 1e9 - off_start_time
                    if elapsed >= OFF_STABLE_DURATION:
                        # ---- FLOOR REACHED ----
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
                        result.arrived = True
                        result.message = (
                            f'Floor {raw_floor} reached '
                            f'(ratio {ratio:.3f}, stable {elapsed:.2f}s).')
                        return result
                    else:
                        feedback.status = 'OFF_UNSTABLE'
                        feedback.ratio  = ratio

            # --- annotate & publish ---
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(cv_image, state_str, (x1, max(y1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
            goal_handle.publish_feedback(feedback)
            self._publish_debug(cv_image)

        # If we exit the loop without succeeding (e.g. node shutdown)
        result.arrived = False
        result.message = 'Loop exited without stable OFF detection.'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FloorArrivalServer()
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
