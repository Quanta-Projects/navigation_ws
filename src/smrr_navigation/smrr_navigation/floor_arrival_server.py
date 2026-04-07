#!/usr/bin/env python3
"""
floor_arrival_server.py
-----------------------
Action server: /check_floor_arrival (smrr_interfaces/action/CheckFloorArrival)

Determines elevator arrival at the target floor using a YOLO model that
classifies button panel buttons in both lit and unlit states.
No HSV / OpenCV colour thresholding is used.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from smrr_interfaces.action import CheckFloorArrival

from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'button_detection.pt',
)

# Map floor ID strings → YOLO class IDs for that button's lit / unlit state
FLOOR_CLASS_MAP = {
    "floor0": {"unlit_class": 2, "lit_class": 3},   # ground / ground_lit
    "floor1": {"unlit_class": 4, "lit_class": 5},   # one / one_lit
    "floor2": {"unlit_class": 8, "lit_class": 9},   # two / two_lit
    "floor3": {"unlit_class": 6, "lit_class": 7},   # three / three_lit
}

INFERENCE_CONF = 0.15
LOOP_HZ = 20                        # target inference rate
LOOP_SLEEP_SEC = 1.0 / LOOP_HZ     # 0.05 s
OFF_STABLE_DURATION_SEC = 0.1      # 100 ms stable-off before declaring arrival


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FloorArrivalServer(Node):

    def __init__(self):
        super().__init__("floor_arrival_server")

        # ── YOLO model ──────────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model from: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded successfully.")

        # ── Shared state ────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_frame = None   # latest BGR image; updated by subscriber

        # ── Callback group allows the inference loop to spin concurrently ───
        self._cb_group = ReentrantCallbackGroup()

        # ── Image subscriber ────────────────────────────────────────────────
        self.create_subscription(
            Image,
            "/zed2_left_camera/image_raw",
            self._image_callback,
            10,
            callback_group=self._cb_group,
        )

        # ── Action server ────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            CheckFloorArrival,
            "/check_floor_arrival",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # ── Debug image publisher (active only during goal execution) ─────────
        self._debug_pub = self.create_publisher(
            Image,
            "/floor_vision/debug_image",
            10,
        )

        self.get_logger().info("FloorArrivalServer is ready.")

    # ── Subscriber callback ─────────────────────────────────────────────────

    def _image_callback(self, msg: Image):
        """Convert incoming ROS image to BGR and cache it."""
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")

    # ── Action server callbacks ─────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        floor = goal_request.target_floor
        if floor not in FLOOR_CLASS_MAP:
            self.get_logger().error(
                f"[CheckFloorArrival] Unknown target_floor '{floor}'. "
                f"Valid values: {list(FLOOR_CLASS_MAP.keys())}"
            )
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[CheckFloorArrival] Accepted goal for floor '{floor}'."
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("[CheckFloorArrival] Cancel requested.")
        return CancelResponse.ACCEPT

    # ── Main execution logic ────────────────────────────────────────────────

    def _execute_callback(self, goal_handle):
        target_floor: str = goal_handle.request.target_floor
        class_ids = FLOOR_CLASS_MAP[target_floor]
        lit_class: int = class_ids["lit_class"]
        unlit_class: int = class_ids["unlit_class"]

        self.get_logger().info(
            f"[CheckFloorArrival] Monitoring floor '{target_floor}' "
            f"(lit_class={lit_class}, unlit_class={unlit_class})."
        )

        feedback_msg = CheckFloorArrival.Feedback()
        result_msg = CheckFloorArrival.Result()

        off_stable_start: float | None = None   # timestamp when OFF state began

        while rclpy.ok():
            # ── Handle cancel ────────────────────────────────────────────────
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.arrived = False
                result_msg.message = "Cancelled by caller."
                self.get_logger().info("[CheckFloorArrival] Goal cancelled.")
                return result_msg

            frame = self._latest_frame

            # ── No frame yet — publish NO_DETECTION and wait ─────────────────
            if frame is None:
                feedback_msg.status = "NO_DETECTION"
                feedback_msg.ratio = 0.0
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(LOOP_SLEEP_SEC)
                continue

            # ── YOLO inference ───────────────────────────────────────────────
            try:
                results = self.model(frame, conf=INFERENCE_CONF, verbose=False)
                # Publish annotated frame showing only the target button's detections
                annotated = frame.copy()
                if results and results[0].boxes is not None:
                    for box in results[0].boxes:
                        cls_id = int(box.cls[0].item())
                        if cls_id not in (lit_class, unlit_class):
                            continue
                        conf = float(box.conf[0].item())
                        cls_name = self.model.names[cls_id] if self.model.names else str(cls_id)
                        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                        color = (0, 255, 0) if cls_id == lit_class else (0, 165, 255)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(
                            annotated,
                            f"{cls_name} {conf:.2f}",
                            (x1, max(y1 - 8, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2,
                        )
                debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = "zed2_left_camera"
                self._debug_pub.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warn(f"[CheckFloorArrival] YOLO error: {exc}")
                time.sleep(LOOP_SLEEP_SEC)
                continue

            detected_lit = False
            detected_unlit = False

            # ── Log all detected classes and confidences for debugging ───────
            if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                debug_detections = []
                for box in results[0].boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    
                    # Try to get the human-readable class name from the model
                    cls_name = self.model.names[cls_id] if self.model.names else str(cls_id)
                    debug_detections.append(f"{cls_name}: {conf:.2f}")

                    if cls_id == lit_class:
                        detected_lit = True
                    elif cls_id == unlit_class:
                        detected_unlit = True
                
                # Print exactly what YOLO is seeing in this frame
                self.get_logger().info(f"[YOLO DEBUG] Detections: {', '.join(debug_detections)}")

            # ── State machine ────────────────────────────────────────────────

            if detected_lit:
                # Elevator still moving — reset stability timer
                off_stable_start = None
                feedback_msg.status = "ON"
                feedback_msg.ratio = 1.0
                goal_handle.publish_feedback(feedback_msg)

            elif detected_unlit and not detected_lit:
                # Button light is off — start / continue stability window
                now = time.monotonic()
                if off_stable_start is None:
                    off_stable_start = now

                elapsed = now - off_stable_start
                feedback_msg.status = "OFF_UNSTABLE"
                feedback_msg.ratio = 0.0
                goal_handle.publish_feedback(feedback_msg)

                if elapsed >= OFF_STABLE_DURATION_SEC:
                    # Stable OFF for 100 ms → elevator has arrived
                    self.get_logger().info(
                        f"[CheckFloorArrival] Floor '{target_floor}' arrived "
                        f"(stable OFF for {elapsed*1000:.0f} ms)."
                    )
                    goal_handle.succeed()
                    result_msg.arrived = True
                    result_msg.message = (
                        f"Arrived at {target_floor} — button unlit for "
                        f"{elapsed*1000:.0f} ms."
                    )
                    return result_msg

            else:
                # Neither class detected
                off_stable_start = None
                feedback_msg.status = "NO_DETECTION"
                feedback_msg.ratio = 0.0
                goal_handle.publish_feedback(feedback_msg)

            time.sleep(LOOP_SLEEP_SEC)

        # rclpy shut down before arrival was confirmed
        result_msg.arrived = False
        result_msg.message = "Node shutdown before arrival was detected."
        return result_msg


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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


if __name__ == "__main__":
    main()