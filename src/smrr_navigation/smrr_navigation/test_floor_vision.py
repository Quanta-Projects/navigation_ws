#!/usr/bin/env python3
"""
Standalone ROS 2 test node for YOLO floor button detection visualisation.
Publishes an annotated debug image showing all detected buttons with
class name and confidence score to the right of each bounding box.
No target-floor input required — detections start immediately.
"""

import os

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Shared constants — kept in sync with floor_arrival_server.py
# ---------------------------------------------------------------------------

MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'button_detection.pt',
)

INFERENCE_CONF = 0.15


class TestFloorVision(Node):

    def __init__(self):
        super().__init__("test_floor_vision")

        self._bridge = CvBridge()
        self._latest_image = None

        self.get_logger().info(f"Loading YOLO model from {MODEL_PATH} ...")
        self._model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLO model loaded.")

        self._debug_pub = self.create_publisher(Image, "/floor_vision/debug_image", 1)

        self.create_subscription(
            Image,
            "/zed2_left_camera/image_raw",
            self._image_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(0.1, self._process)
        self.get_logger().info("test_floor_vision node ready — publishing to /floor_vision/debug_image")

    # ── callbacks ───────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _process(self):
        if self._latest_image is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(self._latest_image, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"CvBridge conversion failed: {exc}")
            return

        try:
            results = self._model(frame, conf=INFERENCE_CONF, verbose=False)
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return

        annotated = frame.copy()

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                cls_name = self._model.names[cls_id] if self._model.names else str(cls_id)
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                label = f"{cls_name}  {conf:.2f}"
                label_x = x2 + 6
                label_y = (y1 + y2) // 2

                # Keep label inside frame horizontally
                (lw, _lh), _baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
                )
                if label_x + lw > annotated.shape[1]:
                    label_x = x1 - lw - 6

                # Black outline then green foreground for readability
                cv2.putText(
                    annotated, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3,
                )
                cv2.putText(
                    annotated, label, (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                )

        try:
            msg_out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = "zed2_left_camera"
            self._debug_pub.publish(msg_out)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
