#!/usr/bin/env python3
"""
elevator_call_button_server.py
-------------------------------
Action server: detect_call_button  (smrr_interfaces/action/DetectCallButton)

Uses the same button_detection.pt model and the same YOLO class-ID approach as
floor_arrival_server.py — no HSV thresholding.

Given current_floor and target_floor:
  1. Infer direction: UP if target index > current index, else DOWN.
  2. Look up the YOLO unlit_class for that direction from DIRECTION_CLASS_MAP.
  3. Wait until the unlit button is detected with confidence >= INFERENCE_CONF.
  4. Sample depth pixels inside the bounding box, unproject to 3-D using the
     camera intrinsics from /zed2_left_camera/camera_info.
  5. Fit a plane to the 3-D point cloud via SVD.
     The last right-singular vector is the plane normal (least-variance direction).
  6. Guarantee the normal points toward the camera (−Z in optical frame).
  7. Compute:
       press_point    = 3-D centroid of the depth samples   (on button surface)
       approach_point = press_point + 0.03 m × normal       (3 cm outward)
  8. Transform both from left_camera_link_optical → link_0_fake via TF2.
  9. Publish approach_point to /xyz_target (geometry_msgs/Point).
     Return both points in the action result (float64 fields).

NOTE: Verify DIRECTION_CLASS_MAP class IDs against your model by running
      test_floor_vision.py and checking which class IDs appear for the
      up/down call buttons.
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

from smrr_interfaces.action import DetectCallButton

# ---------------------------------------------------------------------------
# Constants — same model as floor_arrival_server.py
# ---------------------------------------------------------------------------

MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'button_detection.pt',
)

# Map direction → YOLO class IDs for the call button's unlit / lit state.
# Pattern mirrors FLOOR_CLASS_MAP in floor_arrival_server.py.
# !! Verify these IDs with test_floor_vision.py for your trained model !!
DIRECTION_CLASS_MAP = {
    "UP":   {"unlit_class": 0,  "lit_class": 1},   # up / up_lit
    "DOWN": {"unlit_class": 10, "lit_class": 11},  # down / down_lit
}

INFERENCE_CONF    = 0.15   # same as floor_arrival_server.py
APPROACH_DIST_M   = 0.03   # 3 cm outward along the button plane normal
DEPTH_STEP        = 2      # pixel stride when sampling depth in bbox
MIN_PLANE_PTS     = 20     # minimum valid depth samples for a plane fit
LOOP_SLEEP        = 0.05   # ~20 Hz poll rate

CAMERA_FRAME = 'left_camera_link_optical'
TARGET_FRAME = 'link_0_fake'
RGB_TOPIC    = '/zed2_left_camera/image_raw'
DEPTH_TOPIC  = '/zed2_left_camera/depth/image_raw'
INFO_TOPIC   = '/zed2_left_camera/camera_info'


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ElevatorCallButtonServer(Node):

    def __init__(self):
        super().__init__('elevator_call_button_server')
        self._cb_group = ReentrantCallbackGroup()
        self._bridge   = CvBridge()

        # TF2
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        # Per-goal state (dynamic subscriptions — same pattern as
        # elevator_direction_server.py)
        self._latest_rgb   = None
        self._latest_depth = None
        self._camera_info  = None   # cached after first receipt
        self._rgb_sub      = None
        self._depth_sub    = None
        self._info_sub     = None

        # YOLO — same model as floor_arrival_server.py
        self.get_logger().info(f'Loading YOLO model: {MODEL_PATH}')
        self._model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded.')

        # Publishers
        self._xyz_pub   = self.create_publisher(Point, '/xyz_target', 10)
        self._debug_pub = self.create_publisher(
            Image, '/floor_vision/debug_image', 1)

        # Action server
        self._action_server = ActionServer(
            self,
            DetectCallButton,
            'detect_call_button',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            'ElevatorCallButtonServer ready — action: detect_call_button')

    # ── Subscriber callbacks ─────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        self._latest_rgb = msg

    def _depth_cb(self, msg: Image):
        self._latest_depth = msg

    def _info_cb(self, msg: CameraInfo):
        self._camera_info = msg
        if self._info_sub is not None:
            self.destroy_subscription(self._info_sub)
            self._info_sub = None

    @staticmethod
    def _goal_cb(_):   return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_cb(_): return CancelResponse.ACCEPT

    # ── Helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _parse_floor(s: str) -> int:
        return int(s.strip().lower().replace('floor', ''))

    def _fit_plane(self, bbox, depth_img: np.ndarray,
                   fx: float, fy: float,
                   cx: float, cy: float):
        """
        Sample depth pixels inside bbox, unproject to 3-D, fit a plane via SVD.

        Plane fitting
        -------------
        For each sampled pixel (u, v) with depth d:
            X = (u - cx) * d / fx
            Y = (v - cy) * d / fy
            Z = d
        giving a 3-D point in the camera optical frame.

        Centred matrix A = points - mean(points)  (N × 3)
        Thin SVD:  A = U S Vt
        Normal = Vt[-1]  — the direction with the smallest singular value,
                            i.e. the direction of least variance in the cloud
                            = normal to the best-fit plane.

        The normal is flipped so it always points toward the camera
        (negative Z component in the optical frame, where objects are at +Z).

        Returns (centroid_3d, unit_normal) or (None, None).
        """
        x1, y1, x2, y2 = bbox
        pts = []
        for v in range(y1, y2, DEPTH_STEP):
            for u in range(x1, x2, DEPTH_STEP):
                if v >= depth_img.shape[0] or u >= depth_img.shape[1]:
                    continue
                d = float(depth_img[v, u])
                if not (np.isfinite(d) and 0.1 < d < 8.0):
                    continue
                pts.append([
                    (u - cx) * d / fx,   # X — right in optical frame
                    (v - cy) * d / fy,   # Y — down  in optical frame
                    d,                   # Z — depth (forward)
                ])

        if len(pts) < MIN_PLANE_PTS:
            return None, None

        pts      = np.asarray(pts, dtype=np.float64)
        centroid = pts.mean(axis=0)

        _, _, Vt = np.linalg.svd(pts - centroid, full_matrices=False)
        normal   = Vt[-1].copy()

        # Ensure normal points toward camera: objects at +Z → toward-camera is -Z
        if normal[2] > 0.0:
            normal = -normal
        normal /= np.linalg.norm(normal)

        return centroid, normal

    def _to_link0_fake(self, point_cam: np.ndarray,
                       cam_frame: str) -> np.ndarray | None:
        """Transform a 3-D point from cam_frame to link_0_fake via TF2."""
        ps = PointStamped()
        ps.header.frame_id = cam_frame
        ps.header.stamp    = rclpy.time.Time().to_msg()
        ps.point.x = float(point_cam[0])
        ps.point.y = float(point_cam[1])
        ps.point.z = float(point_cam[2])
        try:
            tf = self._tf_buffer.lookup_transform(
                TARGET_FRAME, cam_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'TF lookup {cam_frame}→{TARGET_FRAME}: {exc}')
            return None
        out = tf2_geometry_msgs.do_transform_point(ps, tf)
        return np.array([out.point.x, out.point.y, out.point.z])

    def _publish_debug(self, frame: np.ndarray, bbox,
                       label: str, color,
                       approach_lf=None, press_lf=None) -> None:
        annotated = frame.copy()
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            (lw, _), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            lx = x2 + 6 if x2 + 6 + lw < annotated.shape[1] else x1 - lw - 6
            ly = (y1 + y2) // 2
            cv2.putText(annotated, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(annotated, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        h = annotated.shape[0]
        if approach_lf is not None:
            cv2.putText(
                annotated,
                f'approach: ({approach_lf[0]:.4f},{approach_lf[1]:.4f},'
                f'{approach_lf[2]:.4f})',
                (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        if press_lf is not None:
            cv2.putText(
                annotated,
                f'press:    ({press_lf[0]:.4f},{press_lf[1]:.4f},'
                f'{press_lf[2]:.4f})',
                (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
        try:
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        except Exception:
            pass

    # ── Action execution ─────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):
        result   = DetectCallButton.Result()
        feedback = DetectCallButton.Feedback()

        current_floor = goal_handle.request.current_floor.strip()
        target_floor  = goal_handle.request.target_floor.strip()

        try:
            direction = ('UP' if self._parse_floor(target_floor)
                         > self._parse_floor(current_floor) else 'DOWN')
        except ValueError as exc:
            self.get_logger().error(f'Floor parse error: {exc}')
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        class_ids   = DIRECTION_CLASS_MAP[direction]
        unlit_class = class_ids["unlit_class"]
        lit_class   = class_ids["lit_class"]

        self.get_logger().info(
            f'[DetectCallButton] {current_floor}→{target_floor}: '
            f'{direction} button '
            f'(unlit_class={unlit_class}, lit_class={lit_class})')

        # Dynamic subscriptions (same pattern as elevator_direction_server.py)
        self._latest_rgb   = None
        self._latest_depth = None
        self._rgb_sub = self.create_subscription(
            Image, RGB_TOPIC, self._rgb_cb, 1,
            callback_group=self._cb_group)
        self._depth_sub = self.create_subscription(
            Image, DEPTH_TOPIC, self._depth_cb, 1,
            callback_group=self._cb_group)
        if self._camera_info is None:
            self._info_sub = self.create_subscription(
                CameraInfo, INFO_TOPIC, self._info_cb, 1,
                callback_group=self._cb_group)

        try:
            while rclpy.ok() and goal_handle.is_active:

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled.'
                    return result

                if (self._latest_rgb   is None
                        or self._latest_depth is None
                        or self._camera_info  is None):
                    feedback.status    = 'WAITING_FOR_DATA'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    time.sleep(LOOP_SLEEP)
                    continue

                # Convert images
                try:
                    bgr = self._bridge.imgmsg_to_cv2(
                        self._latest_rgb, desired_encoding='bgr8')
                    enc = self._latest_depth.encoding
                    if '32FC1' in enc:
                        depth = self._bridge.imgmsg_to_cv2(
                            self._latest_depth, desired_encoding='32FC1')
                    else:
                        depth = (
                            self._bridge.imgmsg_to_cv2(
                                self._latest_depth, desired_encoding='16UC1')
                            .astype(np.float32) / 1000.0)
                except Exception as exc:
                    self.get_logger().warn(f'Image conversion: {exc}')
                    time.sleep(LOOP_SLEEP)
                    continue

                K          = self._camera_info.k
                fx, fy     = K[0], K[4]
                cx, cy     = K[2], K[5]
                cam_frame  = (self._camera_info.header.frame_id
                              or CAMERA_FRAME)

                # ── YOLO inference — same approach as floor_arrival_server ──
                results = self._model(bgr, conf=INFERENCE_CONF, verbose=False)

                detected_unlit = False
                detected_lit   = False
                unlit_bbox     = None

                if results and results[0].boxes is not None:
                    for box in results[0].boxes:
                        cls_id = int(box.cls[0].item())
                        if cls_id == unlit_class:
                            detected_unlit = True
                            unlit_bbox = tuple(
                                map(int, box.xyxy[0].tolist()))
                        elif cls_id == lit_class:
                            detected_lit = True

                # Annotated debug frame (show only target direction boxes)
                annotated = bgr.copy()
                if results and results[0].boxes is not None:
                    for box in results[0].boxes:
                        cls_id = int(box.cls[0].item())
                        if cls_id not in (unlit_class, lit_class):
                            continue
                        conf  = float(box.conf[0].item())
                        bx1, by1, bx2, by2 = (
                            int(v) for v in box.xyxy[0].tolist())
                        col = (0, 255, 0) if cls_id == unlit_class \
                              else (0, 165, 255)
                        cv2.rectangle(annotated,
                                      (bx1, by1), (bx2, by2), col, 2)
                        lbl = (self._model.names[cls_id]
                               if self._model.names else str(cls_id))
                        cv2.putText(annotated, f'{lbl} {conf:.2f}',
                                    (bx2 + 6, (by1 + by2) // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    col, 2)

                # State logic mirrors floor_arrival_server.py
                if detected_lit:
                    # Button already pressed — wait; don't press again
                    feedback.status    = 'BUTTON_LIT'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    cv2.putText(annotated, f'{direction} LIT — already pressed',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 165, 255), 2)
                    try:
                        self._debug_pub.publish(
                            self._bridge.cv2_to_imgmsg(
                                annotated, encoding='bgr8'))
                    except Exception:
                        pass
                    time.sleep(LOOP_SLEEP)
                    continue

                elif not detected_unlit:
                    feedback.status    = 'NO_DETECTION'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    cv2.putText(annotated, f'{direction} not found',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)
                    try:
                        self._debug_pub.publish(
                            self._bridge.cv2_to_imgmsg(
                                annotated, encoding='bgr8'))
                    except Exception:
                        pass
                    time.sleep(LOOP_SLEEP)
                    continue

                # ── Unlit button found — fit plane from depth ─────────────
                #
                # Unproject each depth sample in the bounding box:
                #   X = (u - cx) * d / fx
                #   Y = (v - cy) * d / fy
                #   Z = d
                # Fit plane via SVD; normal = direction of least variance.
                # approach = centroid + 0.03 * normal  (3 cm toward camera)
                # press    = centroid                  (on button surface)
                # ─────────────────────────────────────────────────────────

                centroid, normal = self._fit_plane(
                    unlit_bbox, depth, fx, fy, cx, cy)

                if centroid is None:
                    self.get_logger().warn(
                        '[DetectCallButton] Too few valid depth samples — '
                        'retrying.')
                    feedback.status    = 'DEPTH_INSUFFICIENT'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    time.sleep(LOOP_SLEEP)
                    continue

                press_cam    = centroid
                approach_cam = centroid + APPROACH_DIST_M * normal

                self.get_logger().info(
                    f'[DetectCallButton] cam-frame — '
                    f'press({press_cam[0]:.4f},{press_cam[1]:.4f},'
                    f'{press_cam[2]:.4f}) '
                    f'normal({normal[0]:.3f},{normal[1]:.3f},'
                    f'{normal[2]:.3f})')

                press_lf    = self._to_link0_fake(press_cam,    cam_frame)
                approach_lf = self._to_link0_fake(approach_cam, cam_frame)

                if press_lf is None or approach_lf is None:
                    feedback.status    = 'TF_ERROR'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    time.sleep(LOOP_SLEEP)
                    continue

                # Publish approach point to /xyz_target
                self._xyz_pub.publish(Point(
                    x=approach_lf[0],
                    y=approach_lf[1],
                    z=approach_lf[2]))

                self.get_logger().info(
                    f'[DetectCallButton] link_0_fake approach '
                    f'({approach_lf[0]:.6f},{approach_lf[1]:.6f},'
                    f'{approach_lf[2]:.6f})')
                self.get_logger().info(
                    f'[DetectCallButton] link_0_fake press    '
                    f'({press_lf[0]:.6f},{press_lf[1]:.6f},'
                    f'{press_lf[2]:.6f})')

                self._publish_debug(annotated, None, '', (0, 255, 0),
                                    approach_lf, press_lf)

                feedback.status    = 'LOCALISED'
                feedback.direction = direction
                goal_handle.publish_feedback(feedback)
                goal_handle.succeed()

                result.success    = True
                result.approach_x = float(approach_lf[0])
                result.approach_y = float(approach_lf[1])
                result.approach_z = float(approach_lf[2])
                result.press_x    = float(press_lf[0])
                result.press_y    = float(press_lf[1])
                result.press_z    = float(press_lf[2])
                result.message    = (
                    f'{direction} unlit button localised. '
                    f'approach=({approach_lf[0]:.4f},{approach_lf[1]:.4f},'
                    f'{approach_lf[2]:.4f}) '
                    f'press=({press_lf[0]:.4f},{press_lf[1]:.4f},'
                    f'{press_lf[2]:.4f})')
                return result

            result.success = False
            result.message = 'Node shutdown before detection.'
            return result

        finally:
            for attr in ('_rgb_sub', '_depth_sub', '_info_sub'):
                sub = getattr(self, attr, None)
                if sub is not None:
                    self.destroy_subscription(sub)
                    setattr(self, attr, None)
            self._latest_rgb   = None
            self._latest_depth = None
            self.get_logger().info(
                '[DetectCallButton] Camera subscriptions released.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ElevatorCallButtonServer()
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
