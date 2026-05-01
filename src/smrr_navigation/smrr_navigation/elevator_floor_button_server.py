#!/usr/bin/env python3
"""
elevator_floor_button_server.py

Action server: press_floor_button  (smrr_interfaces/action/PressFloorButton)

Presses the correct floor button on the elevator panel using YOLO detection
and ZED2 point-cloud RANSAC to localise the button in 3-D, then verifies the
press by checking that the target button becomes lit.

State machine per attempt (up to MAX_PRESS_ATTEMPTS):
  WAITING_FOR_DATA → DETECTING → AWAITING_PRESS → VERIFYING → CONFIRMED

Differences from elevator_call_button_server.py:
  * Goal only carries target_floor (no current_floor, no inside_pose).
  * FLOOR_CLASS_MAP instead of DIRECTION_CLASS_MAP.
  * Verification: BUTTON_LIT only — no door-open check (we are inside elevator).
  * Separate topics: /floor_button_press_goal, /floor_button_detection/...
"""

import enum
import math
import os
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point, PointStamped, Pose, PoseArray, Vector3Stamped,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, String
from example_interfaces.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO

from smrr_interfaces.action import PressFloorButton


MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',
    'button_detection.pt',
)

# YOLO class ids from button_detection.pt
# {0: down, 1: down_lit, 2: ground, 3: ground_lit,
#  4: one,  5: one_lit,  6: three, 7: three_lit,
#  8: two,  9: two_lit, 10: up,   11: up_lit}
FLOOR_CLASS_MAP = {
    'floor0': {'unlit_class': 2, 'lit_class': 3},   # ground
    'floor1': {'unlit_class': 4, 'lit_class': 5},   # one
    'floor2': {'unlit_class': 8, 'lit_class': 9},   # two
    'floor3': {'unlit_class': 6, 'lit_class': 7},   # three
}

INFERENCE_CONF   = 0.15
APPROACH_DIST_M  = 0.1    # 6 cm outward along button normal
PRESS_INSET_M    = -0.06   # 6 cm into button surface
LOOP_SLEEP       = 0.05

# Point-cloud ROI tuning (same as call button server)
BBOX_CENTER_SHRINK    = 0.40
NEAR_DEPTH_PERCENTILE = 15.0
NEAR_DEPTH_BAND_M     = 0.005

# TF frames
CAMERA_FRAME     = 'zed2_left_camera_frame_optical'
TARGET_FRAME     = 'link_0_fake'
PUBLISH_FRAME_ID = 'link_0'

# Topics
RGB_TOPIC        = '/zed2_left_camera/image_raw'
INFO_TOPIC       = '/zed2_left_camera/camera_info'
POINTCLOUD_TOPIC = '/zed2/zed_node/point_cloud/cloud_registered'

# EEF orientation (same as call button server)
EEF_ORIENTATION = (-0.039, 0.691, 0.656, -0.301)  # x, y, z, w

# Per-floor manual offsets in TARGET_FRAME (link_0_fake).
# Tune independently for each floor button position on the panel.
FLOOR_OFFSETS = {
    'floor0': (0.0, -0.07, 0.0),
    'floor1': (0.0, -0.07, 0.0),
    'floor2': (0.0, -0.07, 0.0),
    'floor3': (0.0, 0.03, 0.04),
}

# ── Press-and-verify state machine ───────────────────────────────────────────
PRESS_COMPLETE_TOPIC  = '/press_complete'
MAX_PRESS_ATTEMPTS    = 3
ARM_TIMEOUT_SEC       = 120.0
VERIFY_TIMEOUT_SEC    = 10.0
VERIFY_SETTLE_SEC     = 0.5
VERIFY_LIT_MIN_FRAMES = 2
VERIFY_POLL_SEC       = 0.1     # tighter poll — no path-planner call inside verify


class _PressState(enum.Enum):
    WAITING_FOR_DATA = 'WAITING_FOR_DATA'
    DETECTING        = 'DETECTING'
    AWAITING_PRESS   = 'AWAITING_PRESS'
    VERIFYING        = 'VERIFYING'
    CONFIRMED        = 'CONFIRMED'


class ElevatorFloorButtonServer(Node):

    def __init__(self):
        super().__init__('elevator_floor_button_server')
        self._cb_group = ReentrantCallbackGroup()
        self._bridge   = CvBridge()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        # Per-goal sensor data
        self._latest_rgb   = None
        self._latest_cloud = None
        self._camera_info  = None

        self.get_logger().info(f'Loading YOLO model: {MODEL_PATH}')
        self._model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded.')

        self._debug_pub = self.create_publisher(
            Image, '/floor_vision/debug_image', 1)
        self._button_press_goal_pub = self.create_publisher(
            PoseArray, '/button_press_goal', 10)
        self._joint_command_pub = self.create_publisher(
            Float64MultiArray, '/joint_command', 10)
        self._target_button_pub = self.create_publisher(
            String, '/target_button', QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ))

        self._full_cloud_pub = self.create_publisher(
            PointCloud2, '/button_detection/full_cloud', 1)
        self._plane_cloud_pub = self.create_publisher(
            PointCloud2, '/button_detection/plane_cloud', 1)
        self._plane_marker_pub = self.create_publisher(
            MarkerArray, '/button_detection/markers', 1)

        # Persistent sensor subscribers
        self._rgb_sub = self.create_subscription(
            Image, RGB_TOPIC, self._rgb_cb, 1,
            callback_group=self._cb_group)
        self._cloud_sub = self.create_subscription(
            PointCloud2, POINTCLOUD_TOPIC, self._cloud_cb, 1,
            callback_group=self._cb_group)
        self._info_sub = self.create_subscription(
            CameraInfo, INFO_TOPIC, self._info_cb, 1,
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            PressFloorButton,
            'press_floor_button',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        # ── Press-complete signal from arm controller ─────────────────────────
        self._press_complete_event = threading.Event()
        self._press_complete_sub = self.create_subscription(
            Bool, PRESS_COMPLETE_TOPIC, self._press_complete_cb, 1,
            callback_group=self._cb_group)

        self.get_logger().info(
            'ElevatorFloorButtonServer ready — action: press_floor_button')

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        self._latest_rgb = msg

    def _cloud_cb(self, msg: PointCloud2):
        self._latest_cloud = msg

    def _info_cb(self, msg: CameraInfo):
        self._camera_info = msg

    @staticmethod
    def _goal_cb(_):   return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_cb(_): return CancelResponse.ACCEPT

    # ── Press-complete subscriber ─────────────────────────────────────────────

    def _press_complete_cb(self, msg: Bool):
        """Arm controller publishes True to /press_complete when motion is done."""
        if msg.data:
            self._press_complete_event.set()

    # ── TF2 helper ────────────────────────────────────────────────────────────

    def _to_link0(self, point_cam: np.ndarray,
                  cam_frame: str) -> 'np.ndarray | None':
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
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as exc:
            self.get_logger().warn(
                f'TF lookup {cam_frame}→{TARGET_FRAME}: {exc}')
            return None
        out = tf2_geometry_msgs.do_transform_point(ps, tf)
        return np.array([out.point.x, out.point.y, out.point.z],
                        dtype=np.float64)

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_button_press_goal(self,
                                   approach_lf: np.ndarray,
                                   press_lf: np.ndarray,
                                   target_floor: str,
                                   button_center_lf: 'np.ndarray | None' = None) -> None:
        """
        Publish approach pose and button center to /floor_button_press_goal,
        then publish target floor string to /target_floor.

        Mirrors elevator_call_button_server: pa.poses = [approach_pose, button_center_pose].
        """
        qx, qy, qz, qw = EEF_ORIENTATION
        ox, oy, oz = FLOOR_OFFSETS.get(target_floor, (0.0, 0.0, 0.0))
        offset = np.array([ox, oy, oz], dtype=np.float64)

        approach_pub = approach_lf + offset

        def _make_pose(pt):
            p = Pose()
            p.position.x = float(pt[0])
            p.position.y = float(pt[1])
            p.position.z = float(pt[2])
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            return p

        button_pose = Pose()
        if button_center_lf is not None:
            btn_pt = button_center_lf + offset
            button_pose.position.x = float(btn_pt[0])
            button_pose.position.y = float(btn_pt[1])
            button_pose.position.z = float(btn_pt[2])
        button_pose.orientation.x = qx
        button_pose.orientation.y = qy
        button_pose.orientation.z = qz
        button_pose.orientation.w = qw

        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = PUBLISH_FRAME_ID
        pa.poses           = [_make_pose(approach_pub), button_pose]
        # Publish floor number digit (e.g. 'floor2' → '2') to /target_button FIRST
        # so the arm commander has it before acting on the press goal.
        target_msg = String()
        target_msg.data = target_floor.replace('floor', '')
        self._target_button_pub.publish(target_msg)
        time.sleep(0.1)  # 100 ms gap ensures transient_local delivery before press goal

        self._button_press_goal_pub.publish(pa)

    # ── Debug image ───────────────────────────────────────────────────────────

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

    # ── Visualization helpers ─────────────────────────────────────────────────

    @staticmethod
    def _pack_rgb(r: int, g: int, b: int) -> float:
        packed = struct.pack('BBBB', b, g, r, 0)
        return struct.unpack('f', packed)[0]

    def _publish_plane_visualization(
            self,
            all_crop_pts: np.ndarray,
            inlier_mask: np.ndarray,
            centroid: np.ndarray,
            normal: np.ndarray,
            approach_lf: np.ndarray,
            press_lf: np.ndarray,
            button_center_lf: np.ndarray,
            cloud_frame: str,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        # ── Green inlier PointCloud2 ──────────────────────────────────────────
        inlier_pts = all_crop_pts[inlier_mask]
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
        green = self._pack_rgb(0, 255, 0)
        buf = bytearray(len(inlier_pts) * point_step)
        for i, pt in enumerate(inlier_pts):
            struct.pack_into('ffff', buf, i * point_step,
                             float(pt[0]), float(pt[1]), float(pt[2]), green)

        cloud_viz = PointCloud2()
        cloud_viz.header.stamp    = stamp
        cloud_viz.header.frame_id = cloud_frame
        cloud_viz.height          = 1
        cloud_viz.width           = len(inlier_pts)
        cloud_viz.fields          = fields
        cloud_viz.is_bigendian    = False
        cloud_viz.point_step      = point_step
        cloud_viz.row_step        = point_step * len(inlier_pts)
        cloud_viz.data            = bytes(buf)
        cloud_viz.is_dense        = True
        self._plane_cloud_pub.publish(cloud_viz)

        # ── MarkerArray in TARGET_FRAME ───────────────────────────────────────

        def _base_marker(mid: int, mtype: int) -> Marker:
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = TARGET_FRAME
            m.ns              = 'button_detection'
            m.id              = mid
            m.type            = mtype
            m.action          = Marker.ADD
            m.lifetime        = rclpy.duration.Duration(seconds=0.0).to_msg()
            return m

        markers = MarkerArray()

        # id=0  ARROW: press → approach (normal direction)
        arrow = _base_marker(0, Marker.ARROW)
        arrow.scale.x = 0.005
        arrow.scale.y = 0.010
        arrow.scale.z = 0.015
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        tail = Point(x=float(press_lf[0]),    y=float(press_lf[1]),    z=float(press_lf[2]))
        tip  = Point(x=float(approach_lf[0]), y=float(approach_lf[1]), z=float(approach_lf[2]))
        arrow.points = [tail, tip]
        markers.markers.append(arrow)

        # id=1  SPHERE: button center (magenta, 18mm)
        btn_sph = _base_marker(1, Marker.SPHERE)
        btn_sph.pose.position.x  = float(button_center_lf[0])
        btn_sph.pose.position.y  = float(button_center_lf[1])
        btn_sph.pose.position.z  = float(button_center_lf[2])
        btn_sph.pose.orientation.w = 1.0
        btn_sph.scale.x = btn_sph.scale.y = btn_sph.scale.z = 0.018
        btn_sph.color.r = 1.0
        btn_sph.color.g = 0.0
        btn_sph.color.b = 1.0
        btn_sph.color.a = 1.0
        markers.markers.append(btn_sph)

        # id=2  SPHERE: approach (cyan, 20mm)
        app_sph = _base_marker(2, Marker.SPHERE)
        app_sph.pose.position.x  = float(approach_lf[0])
        app_sph.pose.position.y  = float(approach_lf[1])
        app_sph.pose.position.z  = float(approach_lf[2])
        app_sph.pose.orientation.w = 1.0
        app_sph.scale.x = app_sph.scale.y = app_sph.scale.z = 0.020
        app_sph.color.r = 0.0
        app_sph.color.g = 1.0
        app_sph.color.b = 1.0
        app_sph.color.a = 0.85
        markers.markers.append(app_sph)

        # id=3  SPHERE: press (orange, 12mm)
        prs_sph = _base_marker(3, Marker.SPHERE)
        prs_sph.pose.position.x  = float(press_lf[0])
        prs_sph.pose.position.y  = float(press_lf[1])
        prs_sph.pose.position.z  = float(press_lf[2])
        prs_sph.pose.orientation.w = 1.0
        prs_sph.scale.x = prs_sph.scale.y = prs_sph.scale.z = 0.012
        prs_sph.color.r = 1.0
        prs_sph.color.g = 0.5
        prs_sph.color.b = 0.0
        prs_sph.color.a = 1.0
        markers.markers.append(prs_sph)

        # id=4  TEXT: coordinate label
        txt = _base_marker(4, Marker.TEXT_VIEW_FACING)
        txt.pose.position.x  = float(approach_lf[0])
        txt.pose.position.y  = float(approach_lf[1])
        txt.pose.position.z  = float(approach_lf[2]) + 0.05
        txt.pose.orientation.w = 1.0
        txt.scale.z  = 0.025
        txt.color.r  = txt.color.g = txt.color.b = 1.0
        txt.color.a  = 1.0
        txt.text = (
            f'center   ({button_center_lf[0]:.3f},{button_center_lf[1]:.3f},'
            f'{button_center_lf[2]:.3f})\n'
            f'approach ({approach_lf[0]:.3f},{approach_lf[1]:.3f},'
            f'{approach_lf[2]:.3f})\n'
            f'press    ({press_lf[0]:.3f},{press_lf[1]:.3f},'
            f'{press_lf[2]:.3f})'
        )
        markers.markers.append(txt)

        self._plane_marker_pub.publish(markers)

    # ── PRIMARY: ZED2 Point Cloud + RANSAC ───────────────────────────────────

    def _extract_button_pose_pointcloud(
            self,
            cloud_msg: PointCloud2,
            bbox,
            image_width: int,
            image_height: int,
            fx: float, fy: float, cx: float, cy: float,
            optical_frame: str) -> 'dict | None':
        """
        Identical RANSAC pipeline to elevator_call_button_server.py.

        Steps: scale bbox → center-shrunk crop → NaN/range filter →
               near-depth gate → median outlier filter → RANSAC plane →
               SVD refinement → normal enforcement → bbox-center ray-plane
               intersection → approach/press points → TF to link_0_fake.
        """
        bbox_u_c = (bbox[0] + bbox[2]) / 2.0
        bbox_v_c = (bbox[1] + bbox[3]) / 2.0

        if image_width <= 0 or image_height <= 0:
            return None

        sx = cloud_msg.width  / float(image_width)
        sy = cloud_msg.height / float(image_height)

        x1, y1, x2, y2 = bbox
        x1 = max(0, int(math.floor(x1 * sx)))
        y1 = max(0, int(math.floor(y1 * sy)))
        x2 = min(cloud_msg.width,  int(math.ceil(x2 * sx)))
        y2 = min(cloud_msg.height, int(math.ceil(y2 * sy)))
        if x2 <= x1 or y2 <= y1:
            return None

        roi_w = x2 - x1
        roi_h = y2 - y1
        cx_roi = 0.5 * (x1 + x2)
        cy_roi = 0.5 * (y1 + y2)
        shr_w  = max(2.0, roi_w * BBOX_CENTER_SHRINK)
        shr_h  = max(2.0, roi_h * BBOX_CENTER_SHRINK)

        x1 = max(x1, int(math.floor(cx_roi - 0.5 * shr_w)))
        y1 = max(y1, int(math.floor(cy_roi - 0.5 * shr_h)))
        x2 = min(x2, int(math.ceil(cx_roi  + 0.5 * shr_w)))
        y2 = min(y2, int(math.ceil(cy_roi  + 0.5 * shr_h)))
        if x2 <= x1 or y2 <= y1:
            return None

        # ── Raw byte decode ───────────────────────────────────────────────────
        field_offsets = {f.name: f.offset for f in cloud_msg.fields}
        if not all(n in field_offsets for n in ('x', 'y', 'z')):
            return None

        x_off = field_offsets['x']
        y_off = field_offsets['y']
        z_off = field_offsets['z']
        point_step = cloud_msg.point_step
        row_step   = cloud_msg.row_step

        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        if raw.size != cloud_msg.height * row_step:
            return None

        raw_2d  = raw.reshape(cloud_msg.height, row_step)
        packed  = np.ascontiguousarray(
            raw_2d[:, :cloud_msg.width * point_step]
        ).reshape(cloud_msg.height * cloud_msg.width, point_step)

        f32 = np.dtype('>f4' if cloud_msg.is_bigendian else '<f4')
        x_col = packed[:, x_off:x_off + 4].copy().view(f32).reshape(-1)
        y_col = packed[:, y_off:y_off + 4].copy().view(f32).reshape(-1)
        z_col = packed[:, z_off:z_off + 4].copy().view(f32).reshape(-1)

        pts  = np.column_stack((x_col, y_col, z_col)).reshape(
            cloud_msg.height, cloud_msg.width, 3)
        crop = pts[y1:y2, x1:x2, :].reshape(-1, 3).astype(np.float64)

        # ── Filter ────────────────────────────────────────────────────────────
        valid  = np.isfinite(crop).all(axis=1)
        crop   = crop[valid]
        if len(crop) == 0:
            return None

        depths   = np.linalg.norm(crop, axis=1)
        in_range = (depths > 0.05) & (depths <= 5.0)
        crop     = crop[in_range]
        if len(crop) < 8:
            return None

        depths     = np.linalg.norm(crop, axis=1)
        near_depth = float(np.percentile(depths, NEAR_DEPTH_PERCENTILE))
        crop       = crop[depths <= (near_depth + NEAR_DEPTH_BAND_M)]
        if len(crop) < 8:
            return None

        depths       = np.linalg.norm(crop, axis=1)
        median_depth = float(np.median(depths))
        crop         = crop[np.abs(depths - median_depth) <= 0.02]
        if len(crop) < 8:
            return None

        viz_pts = crop.copy()

        # ── RANSAC ────────────────────────────────────────────────────────────
        DIST_THRESH = 0.003
        MAX_ITER    = 200
        MIN_INLIERS = 8
        npts        = len(crop)
        best_count  = 0
        best_mask   = None
        rng         = np.random.default_rng()

        for _ in range(MAX_ITER):
            idx        = rng.choice(npts, size=3, replace=False)
            p1, p2, p3 = crop[idx]
            normal     = np.cross(p2 - p1, p3 - p1)
            norm_len   = np.linalg.norm(normal)
            if norm_len < 1e-8:
                continue
            normal /= norm_len
            d       = -np.dot(normal, p1)
            mask    = np.abs(crop @ normal + d) < DIST_THRESH
            count   = int(mask.sum())
            if count > best_count:
                best_count = count
                best_mask  = mask

        if best_mask is None or best_count < MIN_INLIERS:
            return None

        # ── SVD refinement ────────────────────────────────────────────────────
        inliers         = crop[best_mask]
        inlier_centroid = inliers.mean(axis=0)
        _, _, Vt        = np.linalg.svd(inliers - inlier_centroid,
                                         full_matrices=False)
        normal = Vt[-1].copy()
        if np.linalg.norm(normal) < 1e-8:
            return None
        normal /= np.linalg.norm(normal)

        # ── Normal direction enforcement ──────────────────────────────────────
        toward_camera = -inlier_centroid
        tc_norm       = np.linalg.norm(toward_camera)
        if tc_norm < 1e-8:
            return None
        toward_camera /= tc_norm
        if np.dot(normal, toward_camera) < 0:
            normal = -normal

        # ── Bbox center ray → plane intersection ──────────────────────────────
        cam_frame = cloud_msg.header.frame_id or CAMERA_FRAME
        D_plane   = -np.dot(normal, inlier_centroid)

        button_centroid = inlier_centroid   # fallback

        dx_opt = (bbox_u_c - cx) / fx
        dy_opt = (bbox_v_c - cy) / fy
        dz_opt = 1.0

        ray_opt = Vector3Stamped()
        ray_opt.header.frame_id = optical_frame
        ray_opt.header.stamp    = rclpy.time.Time().to_msg()
        ray_opt.vector.x = float(dx_opt)
        ray_opt.vector.y = float(dy_opt)
        ray_opt.vector.z = float(dz_opt)

        try:
            tf_opt = self._tf_buffer.lookup_transform(
                cam_frame, optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            ray_cloud_msg = tf2_geometry_msgs.do_transform_vector3(
                ray_opt, tf_opt)
            d_cloud = np.array(
                [ray_cloud_msg.vector.x,
                 ray_cloud_msg.vector.y,
                 ray_cloud_msg.vector.z],
                dtype=np.float64)
            denom = np.dot(normal, d_cloud)
            if abs(denom) >= 1e-6:
                t = -D_plane / denom
                if t > 0.0:
                    button_centroid = t * d_cloud
        except Exception as exc:
            self.get_logger().warn(
                f'[RANSAC] Ray-plane TF failed: {exc} — using inlier centroid.')

        # ── Approach / press in camera frame, then TF to link_0_fake ─────────
        approach_cam     = button_centroid + APPROACH_DIST_M * normal
        press_cam        = button_centroid + PRESS_INSET_M   * normal
        approach_lf      = self._to_link0(approach_cam,    cam_frame)
        press_lf         = self._to_link0(press_cam,       cam_frame)
        button_center_lf = self._to_link0(button_centroid, cam_frame)

        if approach_lf is None or press_lf is None or button_center_lf is None:
            return None

        return {
            'approach_lf':      approach_lf,
            'press_lf':         press_lf,
            'button_center_lf': button_center_lf,
            'normal_cam':       normal,
            'centroid_cam':     button_centroid,
            'inlier_count':     best_count,
            'viz_pts':          viz_pts,
            'inlier_mask':      best_mask,
            'cloud_frame':      cam_frame,
        }

    # ── Post-press verification: BUTTON_LIT only ──────────────────────────────

    def _verify_press(self, lit_class: int, deadline: float,
                      goal_handle) -> str:
        """
        After arm signals press complete, poll YOLO for the target lit class.

        Returns: 'BUTTON_LIT' | 'TIMEOUT' | 'CANCELLED'

        No door-open check — we are inside the elevator; no door transitions
        happen during floor button pressing.
        """
        time.sleep(VERIFY_SETTLE_SEC)
        consecutive_lit = 0

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'

            if self._latest_rgb is not None:
                try:
                    bgr      = self._bridge.imgmsg_to_cv2(
                        self._latest_rgb, desired_encoding='bgr8')
                    yolo_res = self._model(bgr, conf=INFERENCE_CONF, verbose=False)
                    if yolo_res and yolo_res[0].boxes is not None:
                        classes = yolo_res[0].boxes.cls.cpu().numpy().astype(int)
                        confs   = yolo_res[0].boxes.conf.cpu().numpy()
                        if any(int(c) == lit_class and float(cf) > 0.55
                               for c, cf in zip(classes, confs)):
                            consecutive_lit += 1
                            if consecutive_lit >= VERIFY_LIT_MIN_FRAMES:
                                return 'BUTTON_LIT'
                        else:
                            consecutive_lit = 0
                except Exception as exc:
                    self.get_logger().warn(f'[Verify] YOLO error: {exc}')

            time.sleep(VERIFY_POLL_SEC)

        return 'TIMEOUT'

    # ── Action execution (state machine) ──────────────────────────────────────

    def _execute_cb(self, goal_handle):  # noqa: C901
        result   = PressFloorButton.Result()
        feedback = PressFloorButton.Feedback()

        target_floor = goal_handle.request.target_floor.strip().lower()

        if target_floor not in FLOOR_CLASS_MAP:
            self.get_logger().error(
                f'[PressFloorButton] Unknown target_floor: "{target_floor}". '
                f'Valid: {list(FLOOR_CLASS_MAP.keys())}')
            goal_handle.abort()
            result.success = False
            result.message = f'Unknown target_floor: {target_floor}'
            return result

        class_ids   = FLOOR_CLASS_MAP[target_floor]
        unlit_class = class_ids['unlit_class']
        lit_class   = class_ids['lit_class']

        self.get_logger().info(
            f'[PressFloorButton] target={target_floor} '
            f'(unlit={unlit_class}, lit={lit_class})')

        approach_lf = None
        press_lf    = None
        state       = _PressState.WAITING_FOR_DATA

        try:
            for attempt in range(MAX_PRESS_ATTEMPTS):
                if attempt > 0:
                    self.get_logger().info(
                        f'[PressFloorButton] Re-localising — attempt '
                        f'{attempt + 1}/{MAX_PRESS_ATTEMPTS}')

                self._latest_rgb   = None
                self._latest_cloud = None
                state = _PressState.WAITING_FOR_DATA

                # ════════════════════════════════════════════════════════════
                # State: WAITING_FOR_DATA
                # ════════════════════════════════════════════════════════════
                while state == _PressState.WAITING_FOR_DATA:
                    if not (rclpy.ok() and goal_handle.is_active):
                        break
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Cancelled.'
                        return result
                    if (self._latest_rgb   is not None
                            and self._camera_info is not None
                            and self._latest_cloud is not None):
                        state = _PressState.DETECTING
                    else:
                        feedback.status = 'WAITING_FOR_DATA'
                        feedback.floor  = target_floor
                        goal_handle.publish_feedback(feedback)
                        time.sleep(LOOP_SLEEP)

                if not (rclpy.ok() and goal_handle.is_active):
                    break

                # ════════════════════════════════════════════════════════════
                # State: DETECTING
                # ════════════════════════════════════════════════════════════
                while state == _PressState.DETECTING:
                    if not (rclpy.ok() and goal_handle.is_active):
                        break
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Cancelled.'
                        return result

                    try:
                        bgr = self._bridge.imgmsg_to_cv2(
                            self._latest_rgb, desired_encoding='bgr8')
                    except Exception as exc:
                        self.get_logger().warn(f'Image conversion: {exc}')
                        time.sleep(LOOP_SLEEP)
                        continue

                    K         = self._camera_info.k
                    fx, fy    = K[0], K[4]
                    cx_k, cy_k = K[2], K[5]
                    cam_frame = (self._camera_info.header.frame_id or CAMERA_FRAME)

                    # ── YOLO inference ────────────────────────────────────────
                    yolo_results   = self._model(bgr, conf=INFERENCE_CONF, verbose=False)
                    detected_unlit = False
                    detected_lit   = False
                    unlit_bbox     = None

                    if yolo_results and yolo_results[0].boxes is not None:
                        for box in yolo_results[0].boxes:
                            cls_id = int(box.cls[0].item())
                            if cls_id == unlit_class:
                                detected_unlit = True
                                unlit_bbox = tuple(map(int, box.xyxy[0].tolist()))
                            elif cls_id == lit_class:
                                detected_lit = True

                    # Build annotated debug frame
                    annotated = bgr.copy()
                    if yolo_results and yolo_results[0].boxes is not None:
                        for box in yolo_results[0].boxes:
                            cls_id = int(box.cls[0].item())
                            if cls_id not in (unlit_class, lit_class):
                                continue
                            conf = float(box.conf[0].item())
                            bx1, by1, bx2, by2 = (int(v) for v in box.xyxy[0].tolist())
                            col = (0, 255, 0) if cls_id == unlit_class else (0, 165, 255)
                            cv2.rectangle(annotated, (bx1, by1), (bx2, by2), col, 2)
                            lbl = (self._model.names[cls_id]
                                   if self._model.names else str(cls_id))
                            cv2.putText(annotated, f'{lbl} {conf:.2f}',
                                        (bx2 + 6, (by1 + by2) // 2),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

                    # ── Target button already lit — already pressed ────────────
                    if detected_lit:
                        feedback.status = 'BUTTON_LIT'
                        feedback.floor  = target_floor
                        goal_handle.publish_feedback(feedback)
                        cv2.putText(annotated,
                                    f'{target_floor} LIT — already pressed',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 165, 255), 2)
                        try:
                            self._debug_pub.publish(
                                self._bridge.cv2_to_imgmsg(
                                    annotated, encoding='bgr8'))
                        except Exception:
                            pass
                        approach_lf = np.zeros(3)
                        press_lf    = np.zeros(3)
                        result.message = (
                            f'{target_floor} button already lit; already pressed.')
                        arm_down_msg = Float64MultiArray()
                        arm_down_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
                        self._joint_command_pub.publish(arm_down_msg)
                        self.get_logger().info(
                            '[PressFloorButton] Arm-down on lit detection (already lit)')
                        state = _PressState.CONFIRMED
                        break

                    # ── Target button not detected this frame ─────────────────
                    if not detected_unlit or unlit_bbox is None:
                        feedback.status = 'NO_DETECTION'
                        feedback.floor  = target_floor
                        goal_handle.publish_feedback(feedback)
                        cv2.putText(annotated,
                                    f'{target_floor} not found',
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

                    # ── RANSAC localisation ───────────────────────────────────
                    pc_result = self._extract_button_pose_pointcloud(
                        self._latest_cloud, unlit_bbox,
                        int(self._latest_rgb.width), int(self._latest_rgb.height),
                        fx, fy, cx_k, cy_k, cam_frame)

                    if pc_result is None:
                        self.get_logger().warn(
                            '[RANSAC] Failed — insufficient depth points. Retrying.')
                        feedback.status = 'DEPTH_INSUFFICIENT'
                        feedback.floor  = target_floor
                        goal_handle.publish_feedback(feedback)
                        time.sleep(LOOP_SLEEP)
                        continue

                    # Localisation succeeded
                    approach_lf      = pc_result['approach_lf']
                    press_lf         = pc_result['press_lf']
                    button_center_lf = pc_result['button_center_lf']
                    centroid_cam     = pc_result['centroid_cam']
                    inlier_count     = int(pc_result['inlier_count'])

                    self.get_logger().info(
                        '[RANSAC] inliers={}  center_link0=({:.4f},{:.4f},{:.4f})  '
                        'approach=({:.4f},{:.4f},{:.4f})  press=({:.4f},{:.4f},{:.4f})  '
                        '[PUBLISHING /floor_button_press_goal]'.format(
                            inlier_count,
                            button_center_lf[0], button_center_lf[1],
                            button_center_lf[2],
                            approach_lf[0],      approach_lf[1],      approach_lf[2],
                            press_lf[0],         press_lf[1],         press_lf[2],
                        ))

                    self._publish_button_press_goal(approach_lf, press_lf, target_floor, button_center_lf)
                    self._publish_debug(annotated, unlit_bbox, '',
                                        (0, 255, 0), approach_lf, press_lf)
                    self._full_cloud_pub.publish(self._latest_cloud)
                    self._publish_plane_visualization(
                        all_crop_pts     = pc_result['viz_pts'],
                        inlier_mask      = pc_result['inlier_mask'],
                        centroid         = centroid_cam,
                        normal           = pc_result['normal_cam'],
                        approach_lf      = approach_lf,
                        press_lf         = press_lf,
                        button_center_lf = button_center_lf,
                        cloud_frame      = pc_result['cloud_frame'],
                    )

                    feedback.status = 'LOCALISED'
                    feedback.floor  = target_floor
                    goal_handle.publish_feedback(feedback)
                    state = _PressState.AWAITING_PRESS
                    break  # exit DETECTING loop

                if not (rclpy.ok() and goal_handle.is_active):
                    break

                if state == _PressState.CONFIRMED:
                    break

                if state != _PressState.AWAITING_PRESS:
                    continue

                # ════════════════════════════════════════════════════════════
                # State: AWAITING_PRESS
                # ════════════════════════════════════════════════════════════
                self._press_complete_event.clear()
                feedback.status = 'AWAITING_PRESS'
                feedback.floor  = target_floor
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f'[PressFloorButton] Awaiting /press_complete '
                    f'(timeout {ARM_TIMEOUT_SEC:.0f}s, '
                    f'attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')

                arm_deadline = time.time() + ARM_TIMEOUT_SEC
                got_press    = False
                while time.time() < arm_deadline:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Cancelled while awaiting arm press.'
                        return result
                    if self._press_complete_event.wait(timeout=0.2):
                        got_press = True
                        break

                if not got_press:
                    self.get_logger().warn(
                        f'[PressFloorButton] No /press_complete within '
                        f'{ARM_TIMEOUT_SEC:.0f}s '
                        f'(attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')
                    continue

                # ════════════════════════════════════════════════════════════
                # State: VERIFYING — BUTTON_LIT only
                # ════════════════════════════════════════════════════════════
                feedback.status = 'VERIFYING'
                feedback.floor  = target_floor
                goal_handle.publish_feedback(feedback)

                verify_deadline = time.time() + VERIFY_TIMEOUT_SEC
                reason = self._verify_press(lit_class, verify_deadline, goal_handle)

                self.get_logger().info(
                    f'[PressFloorButton] Verification: {reason} '
                    f'(attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')

                if reason == 'CANCELLED':
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled during verification.'
                    return result

                if reason == 'BUTTON_LIT':
                    arm_down_msg = Float64MultiArray()
                    arm_down_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
                    self._joint_command_pub.publish(arm_down_msg)
                    self.get_logger().info(
                        '[PressFloorButton] Arm-down on lit detection (BUTTON_LIT verified)')
                    state = _PressState.CONFIRMED
                    result.message = (
                        f'{target_floor} press confirmed via BUTTON_LIT. '
                        f'approach=({approach_lf[0]:.4f},{approach_lf[1]:.4f},'
                        f'{approach_lf[2]:.4f}) '
                        f'press=({press_lf[0]:.4f},{press_lf[1]:.4f},'
                        f'{press_lf[2]:.4f})')
                    break

                self.get_logger().warn(
                    f'[PressFloorButton] Button not lit after press — '
                    f'retrying (attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')

            # ── Final result ──────────────────────────────────────────────────
            if state == _PressState.CONFIRMED:
                goal_handle.succeed()
                result.success    = True
                result.approach_x = float(approach_lf[0]) if approach_lf is not None else 0.0
                result.approach_y = float(approach_lf[1]) if approach_lf is not None else 0.0
                result.approach_z = float(approach_lf[2]) if approach_lf is not None else 0.0
                result.press_x    = float(press_lf[0])    if press_lf    is not None else 0.0
                result.press_y    = float(press_lf[1])    if press_lf    is not None else 0.0
                result.press_z    = float(press_lf[2])    if press_lf    is not None else 0.0
                # Move arm down after confirmed press
                arm_down_msg = Float64MultiArray()
                arm_down_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
                self._joint_command_pub.publish(arm_down_msg)
                self.get_logger().info('[PressFloorButton] Published arm-down command to /joint_command')
            else:
                if rclpy.ok() and goal_handle.is_active:
                    goal_handle.abort()
                result.success = False
                if not result.message:
                    result.message = (
                        f'Button not confirmed after '
                        f'{MAX_PRESS_ATTEMPTS} attempts.')
            return result

        finally:
            self._latest_rgb   = None
            self._latest_cloud = None
            self.get_logger().info(
                '[PressFloorButton] Goal finished; sensor caches reset.')


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFloorButtonServer()
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
