#!/usr/bin/env python3
"""
Standalone ROS 2 test node for YOLO + OpenCV floor verification.
Includes Bounding Box Caching to handle YOLO drop-outs during LED illumination.
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'yolo_button_detection.pt',
)

# Tuned HSV thresholds — broadened to catch halo/glow around illuminated buttons
HSV_LOWER = np.array([5, 50, 150])
HSV_UPPER = np.array([35, 255, 255])
# Lowered threshold: Only 4% of the bounding box needs to be orange to trigger ON
ON_RATIO_THRESHOLD = 0.03 

class TestFloorVision(Node):
    def __init__(self):
        super().__init__("test_floor_vision")

        # ---- state ----
        self.target_floor: str | None = None
        self.latest_image: Image | None = None
        self.bridge = CvBridge()
        
        # ---- Bounding Box Tracker ----
        self.cached_bbox = None  # Stores (x1, y1, x2, y2)
        self.missed_frames = 0   # How many frames since YOLO last saw it
        self.MAX_MISSED_FRAMES = 100 # Drop cache after ~10 seconds at 10Hz camera rate

        # ---- YOLO model ----
        self.get_logger().info(f"Loading YOLO model from {MODEL_PATH} …")
        self.model = YOLO(MODEL_PATH)

        self.debug_pub = self.create_publisher(Image, "/floor_vision/debug_image", 1)
        self.create_subscription(String, "/target_floor", self._target_floor_cb, 10)
        self.create_subscription(Image, "/zed2_left_camera/image_raw", self._image_cb, 1)

        self.create_timer(0.1, self._process)
        self.get_logger().info("test_floor_vision node ready.")

    def _target_floor_cb(self, msg: String):
        new_target = msg.data.strip()
        if new_target != self.target_floor:
            self.target_floor = new_target
            self.cached_bbox = None # Reset cache on new target
            self.get_logger().info(f"Target floor set to: {self.target_floor}")

    def _image_cb(self, msg: Image):
        self.latest_image = msg

    def _publish_debug(self, cv_image: np.ndarray) -> None:
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
        except Exception as e:
            pass

    def _overlay(self, img: np.ndarray, text: str, color=(200, 200, 200), y: int = 30) -> None:
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    def _process(self):
        # Need at least an image to publish anything useful
        if self.latest_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion failed: {e}")
            return

        if self.target_floor is None:
            self._overlay(cv_image, "Waiting for /target_floor ...", color=(0, 200, 255))
            self._publish_debug(cv_image)
            return

        target_class = f"button-{self.target_floor}"
        self._overlay(cv_image, f"Target: {target_class}", color=(255, 255, 0))

        results = self.model(cv_image, conf=0.1)
        target_found_in_yolo = False

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names.get(cls_id, "")

                if cls_name == target_class:
                    target_found_in_yolo = True
                    self.cached_bbox = tuple(map(int, box.xyxy[0].tolist()))
                    self.missed_frames = 0
                    break

        working_bbox = None
        if target_found_in_yolo:
            working_bbox = self.cached_bbox
        elif self.cached_bbox is not None and self.missed_frames < self.MAX_MISSED_FRAMES:
            working_bbox = self.cached_bbox
            self.missed_frames += 1
            self._overlay(cv_image, "YOLO BLIND: Using cached position", color=(0, 165, 255), y=60)

        if working_bbox is None:
            self._overlay(cv_image, "Target button not visible", color=(0, 0, 255), y=60)
            self._publish_debug(cv_image)
            return

        x1, y1, x2, y2 = working_bbox
        crop = cv_image[y1:y2, x1:x2]

        if crop.size > 0:
            hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_crop, HSV_LOWER, HSV_UPPER)
            ratio = np.count_nonzero(mask) / mask.size

            # --- PICTURE-IN-PICTURE MASK DEBUGGING ---
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_resized = cv2.resize(mask_bgr, (150, 150), interpolation=cv2.INTER_NEAREST)
            h_m, w_m = mask_resized.shape[:2]
            cv_image[0:h_m, cv_image.shape[1]-w_m:cv_image.shape[1]] = mask_resized
            cv2.rectangle(cv_image, (cv_image.shape[1]-w_m, 0), (cv_image.shape[1], h_m), (0, 255, 0), 2)
            cv2.putText(cv_image, "HSV MASK", (cv_image.shape[1]-w_m+10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # -----------------------------------------

            if ratio >= ON_RATIO_THRESHOLD:
                state_str = f"ON (Ratio: {ratio:.2f})"
                color = (0, 0, 255)
                self.get_logger().info("Target button is ON - Still riding...")
            else:
                state_str = f"OFF (Ratio: {ratio:.2f})"
                color = (0, 255, 0)
                self.get_logger().info("Target button is OFF - FLOOR REACHED!")

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(cv_image, state_str, (x1, max(y1 - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        self._publish_debug(cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = TestFloorVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()