#!/usr/bin/env python3

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
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point, PointStamped, Pose, PoseArray, PoseStamped, Vector3Stamped,
)
from nav2_msgs.action import ComputePathToPose as Nav2ComputePathToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO

from smrr_interfaces.action import DetectCallButton


MODEL_PATH = os.path.join(
    get_package_share_directory('smrr_navigation'),
    'models',

    'button_detection.pt',
)

DIRECTION_CLASS_MAP = {
    'UP':   {'unlit_class': 10, 'lit_class': 11},
    'DOWN': {'unlit_class': 0,  'lit_class': 1},
}

INFERENCE_CONF   = 0.15
APPROACH_DIST_M  = 0.10    # 10 cm outward along button normal
PRESS_INSET_M    = -0.04  # 7 mm into button surface
LOOP_SLEEP       = 0.05

# Point-cloud ROI tuning
BBOX_CENTER_SHRINK = 0.40  # Use center-focused crop to reduce wall dominance in RANSAC
NEAR_DEPTH_PERCENTILE = 15.0  # Keep points near the closest quartile in ROI
NEAR_DEPTH_BAND_M = 0.005     # Allow 1.5 cm spread beyond near-depth percentile

# TF frames
CAMERA_FRAME     = 'zed2_left_camera_frame_optical'
TARGET_FRAME     = 'link_0_fake'   # coordinates computed in this frame
PUBLISH_FRAME_ID = 'link_0'        # frame_id written into /button_press_goal header
                                    # link_0 and link_0_fake are colocated;
                                    # MoveIt commander uses setPoseReferenceFrame("link_0")

# Topics
RGB_TOPIC        = '/zed2_left_camera/image_raw'
INFO_TOPIC       = '/zed2_left_camera/camera_info'
POINTCLOUD_TOPIC = '/zed2/zed_node/point_cloud/cloud_registered'

# EEF orientation published with button_press_goal.
# Measured from arm at pressing configuration.
EEF_ORIENTATION = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w

# Manual offsets (meters) in TARGET_FRAME (link_0_fake).
# Applied only to points published on /button_press_goal.
# Separate tuning for each button direction.
BUTTON_GOAL_OFFSET_UP_X = 0.0
BUTTON_GOAL_OFFSET_UP_Y = 0.00
BUTTON_GOAL_OFFSET_UP_Z = 0.02

BUTTON_GOAL_OFFSET_DOWN_X = 0.0
BUTTON_GOAL_OFFSET_DOWN_Y = -0.04
BUTTON_GOAL_OFFSET_DOWN_Z = 0.0075

# ── Press-and-verify state machine ───────────────────────────────────────────
PRESS_COMPLETE_TOPIC    = '/press_complete'
MAX_PRESS_ATTEMPTS      = 3        # arm press retries before FAILURE
ARM_TIMEOUT_SEC         = 180.0     # max seconds waiting for /press_complete
VERIFY_TIMEOUT_SEC      = 10.0      # seconds to verify after arm signals done
VERIFY_SETTLE_SEC       = 0.5      # settle delay before polling starts
VERIFY_LIT_MIN_FRAMES   = 2        # consecutive YOLO lit-class frames to confirm
VERIFY_PATH_POLL_SEC    = 0.3      # sleep between verification loop iterations

# Nav2 path-planner call timeouts (used inside _door_is_open)
PATH_SERVER_WAIT_SEC    = 0.5
PATH_GOAL_TIMEOUT_SEC   = 1.5
PATH_RESULT_TIMEOUT_SEC = 1.5


class _PressState(enum.Enum):
    WAITING_FOR_DATA = 'WAITING_FOR_DATA'
    DETECTING        = 'DETECTING'
    AWAITING_PRESS   = 'AWAITING_PRESS'
    VERIFYING        = 'VERIFYING'
    CONFIRMED        = 'CONFIRMED'


class ElevatorCallButtonServer(Node):

    def __init__(self):
        super().__init__('elevator_call_button_server')
        self._cb_group = ReentrantCallbackGroup()
        self._bridge   = CvBridge()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        # Per-goal sensor data
        self._latest_rgb   = None
        self._latest_cloud = None
        self._camera_info  = None   # cached after first receipt

        # Persistent subscribers (avoid destroy/create races with executor)
        self._rgb_sub   = None
        self._cloud_sub = None
        self._info_sub  = None

        self.get_logger().info(f'Loading YOLO model: {MODEL_PATH}')
        self._model = YOLO(MODEL_PATH)
        self.get_logger().info('YOLO model loaded.')

        self._debug_pub = self.create_publisher(
            Image, '/floor_vision/debug_image', 1)
        self._button_press_goal_pub = self.create_publisher(
            PoseArray, '/button_press_goal', 10)
        self._button_press_direction_pub = self.create_publisher(
            String, '/target_button', QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ))

        # ── Visualization publishers ──────────────────────────────────────────
        # /button_detection/full_cloud   — the complete raw ZED2 point cloud
        #                                  re-published so RViz shows the full scene
        # /button_detection/plane_cloud  — RANSAC inlier points only, coloured green
        #                                  overlaid on top of the full cloud in RViz
        # /button_detection/markers      — MarkerArray in TARGET_FRAME (link_0_fake)
        #   ARROW  = plane normal vector from press point to approach point
        #   SPHERE = button center (magenta), approach point (cyan), press point (orange)
        #   TEXT   = coordinate label floating above approach sphere
        self._full_cloud_pub = self.create_publisher(
            PointCloud2, '/button_detection/full_cloud', 1)
        self._plane_cloud_pub = self.create_publisher(
            PointCloud2, '/button_detection/plane_cloud', 1)
        self._plane_marker_pub = self.create_publisher(
            MarkerArray, '/button_detection/markers', 1)
        # ─────────────────────────────────────────────────────────────────────

        # Persistent subscriptions: created once for node lifetime.
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
            DetectCallButton,
            'detect_call_button',
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

        # ── Nav2 path-planner client (door-open detection) ────────────────────
        self._path_client = ActionClient(
            self, Nav2ComputePathToPose, '/compute_path_to_pose',
            callback_group=self._cb_group)

        self.get_logger().info(
            'ElevatorCallButtonServer ready — action: detect_call_button')

    # ── Subscriber callbacks ─────────────────────────────────────────────────

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

    @staticmethod
    def _parse_floor(s: str) -> int:
        return int(s.strip().lower().replace('floor', ''))

    # ── TF2 helper ───────────────────────────────────────────────────────────

    def _to_link0(self, point_cam: np.ndarray,
                  cam_frame: str) -> 'np.ndarray | None':
        """Transform a 3-D point from cam_frame to TARGET_FRAME via TF2."""
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
        return np.array([out.point.x, out.point.y, out.point.z],
                        dtype=np.float64)

    # ── Publisher ────────────────────────────────────────────────────────────

    def _publish_button_press_goal(self,
                                   approach_lf: np.ndarray,
                                   press_lf: np.ndarray,
                                   direction: str,
                                   button_center_lf: 'np.ndarray | None' = None) -> None:
        """
        Publish approach pose and button center to /button_press_goal.

        Coordinates are in TARGET_FRAME (link_0_fake).
        Direction-specific offsets are added in TARGET_FRAME to both
        approach and press coordinates immediately before publishing.
        Header frame_id is PUBLISH_FRAME_ID ('link_0') because
        link_0_fake and link_0 are the same physical frame and the
        MoveIt commander uses setPoseReferenceFrame("link_0").
        """
        qx, qy, qz, qw = EEF_ORIENTATION

        direction_norm = direction.strip().upper()
        if direction_norm == 'UP':
            offset = np.array([
                BUTTON_GOAL_OFFSET_UP_X,
                BUTTON_GOAL_OFFSET_UP_Y,
                BUTTON_GOAL_OFFSET_UP_Z,
            ], dtype=np.float64)
        elif direction_norm == 'DOWN':
            offset = np.array([
                BUTTON_GOAL_OFFSET_DOWN_X,
                BUTTON_GOAL_OFFSET_DOWN_Y,
                BUTTON_GOAL_OFFSET_DOWN_Z,
            ], dtype=np.float64)
        else:
            self.get_logger().warn(
                f'Unknown direction "{direction}" for goal offset. '
                'Using zero offset.')
            offset = np.zeros(3, dtype=np.float64)

        approach_pub_lf = approach_lf + offset
        press_pub_lf = press_lf + offset

        approach_pose = Pose()
        approach_pose.position.x = float(approach_pub_lf[0])
        approach_pose.position.y = float(approach_pub_lf[1])
        approach_pose.position.z = float(approach_pub_lf[2])
        approach_pose.orientation.x = qx
        approach_pose.orientation.y = qy
        approach_pose.orientation.z = qz
        approach_pose.orientation.w = qw

        press_pose = Pose()
        press_pose.position.x = float(press_pub_lf[0])
        press_pose.position.y = float(press_pub_lf[1])
        press_pose.position.z = float(press_pub_lf[2])
        press_pose.orientation.x = qx
        press_pose.orientation.y = qy
        press_pose.orientation.z = qz
        press_pose.orientation.w = qw

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
        pa.header.frame_id = PUBLISH_FRAME_ID   # 'link_0'
        pa.poses           = [approach_pose, button_pose]
        self._button_press_goal_pub.publish(pa)

        dir_msg = String()
        dir_msg.data = direction.strip().upper()
        self._button_press_direction_pub.publish(dir_msg)

    # ── Debug image publisher ─────────────────────────────────────────────────

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
        """Pack R,G,B bytes into a single float32 for PointCloud2 RGB field."""
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
        """
        Publish two RViz-compatible visualizations of the fitted plane.

        1. /button_detection/plane_cloud  (PointCloud2, cloud_frame)
           Publishes ONLY the RANSAC inlier points coloured bright green.
           In RViz, add BOTH /button_detection/full_cloud (the complete raw
           ZED2 scene) AND /button_detection/plane_cloud side-by-side.
           The green inlier cluster will visually overlay the full scene,
           making it immediately clear which surface was fitted as the button
           plane.

        2. /button_detection/markers  (MarkerArray, TARGET_FRAME = link_0_fake)
           id=0  ARROW  : press_lf → approach_lf (shows normal direction, cyan)
           id=1  SPHERE : button center (bbox ray–plane intersection) — MAGENTA, 18mm
                          This is the primary button position result.
                          It differs from the inlier mean when inliers are skewed.
           id=2  SPHERE : approach point  — CYAN,   20mm
           id=3  SPHERE : press point     — ORANGE, 12mm
           id=4  TEXT   : coordinate label floating 5cm above approach sphere
        """
        stamp = self.get_clock().now().to_msg()

        # ── 1. Green inlier-only PointCloud2 in the cloud sensor frame ────────
        inlier_pts = all_crop_pts[inlier_mask]

        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16   # 4 fields × 4 bytes each

        green = self._pack_rgb(0, 255, 0)   # pure bright green for inliers

        buf = bytearray(len(inlier_pts) * point_step)
        for i, pt in enumerate(inlier_pts):
            offset = i * point_step
            struct.pack_into('ffff', buf, offset,
                             float(pt[0]), float(pt[1]),
                             float(pt[2]), green)

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

        # ── 2. MarkerArray in TARGET_FRAME (link_0_fake) ──────────────────────

        def _base_marker(mid: int, mtype: int) -> Marker:
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = TARGET_FRAME
            m.ns              = 'button_detection'
            m.id              = mid
            m.type            = mtype
            m.action          = Marker.ADD
            # Zero lifetime means marker persists until replaced/deleted.
            m.lifetime        = rclpy.duration.Duration(seconds=0.0).to_msg()
            return m

        markers = MarkerArray()

        # id=0  ARROW: tail = press_lf, tip = approach_lf
        # Points outward from wall along the plane normal —
        # immediate visual confirmation that the normal direction is correct.
        arrow = _base_marker(0, Marker.ARROW)
        arrow.scale.x = 0.005   # shaft diameter (m)
        arrow.scale.y = 0.010   # head diameter (m)
        arrow.scale.z = 0.015   # head length (m)
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0     # cyan
        arrow.color.a = 1.0
        tail = Point()
        tail.x = float(press_lf[0])
        tail.y = float(press_lf[1])
        tail.z = float(press_lf[2])
        tip = Point()
        tip.x = float(approach_lf[0])
        tip.y = float(approach_lf[1])
        tip.z = float(approach_lf[2])
        arrow.points = [tail, tip]
        markers.markers.append(arrow)

        # ── CHANGED: id=1 now shows button_center_lf (bbox ray–plane intersection)
        # This is the 3D position of the exact geometric center of the button,
        # computed by projecting the bounding box center pixel onto the fitted plane.
        # Colour: MAGENTA — distinguishes it clearly from approach (cyan) and press (orange).
        # Size: 18mm — larger than press (12mm), slightly smaller than approach (20mm).
        btn_sph = _base_marker(1, Marker.SPHERE)
        btn_sph.pose.position.x  = float(button_center_lf[0])
        btn_sph.pose.position.y  = float(button_center_lf[1])
        btn_sph.pose.position.z  = float(button_center_lf[2])
        btn_sph.pose.orientation.w = 1.0
        btn_sph.scale.x = btn_sph.scale.y = btn_sph.scale.z = 0.018
        btn_sph.color.r = 1.0
        btn_sph.color.g = 0.0
        btn_sph.color.b = 1.0   # magenta — button center (ray-plane intersection)
        btn_sph.color.a = 1.0
        markers.markers.append(btn_sph)

        # id=2  SPHERE: approach point (cyan, 20mm)
        app_sph = _base_marker(2, Marker.SPHERE)
        app_sph.pose.position.x  = float(approach_lf[0])
        app_sph.pose.position.y  = float(approach_lf[1])
        app_sph.pose.position.z  = float(approach_lf[2])
        app_sph.pose.orientation.w = 1.0
        app_sph.scale.x = app_sph.scale.y = app_sph.scale.z = 0.020
        app_sph.color.r = 0.0
        app_sph.color.g = 1.0
        app_sph.color.b = 1.0   # cyan
        app_sph.color.a = 0.85
        markers.markers.append(app_sph)

        # id=3  SPHERE: press point (orange, 12mm)
        prs_sph = _base_marker(3, Marker.SPHERE)
        prs_sph.pose.position.x  = float(press_lf[0])
        prs_sph.pose.position.y  = float(press_lf[1])
        prs_sph.pose.position.z  = float(press_lf[2])
        prs_sph.pose.orientation.w = 1.0
        prs_sph.scale.x = prs_sph.scale.y = prs_sph.scale.z = 0.012
        prs_sph.color.r = 1.0
        prs_sph.color.g = 0.5
        prs_sph.color.b = 0.0   # orange
        prs_sph.color.a = 1.0
        markers.markers.append(prs_sph)

        # id=4  TEXT: coordinate label floating 5 cm above the approach sphere
        txt = _base_marker(4, Marker.TEXT_VIEW_FACING)
        txt.pose.position.x  = float(approach_lf[0])
        txt.pose.position.y  = float(approach_lf[1])
        txt.pose.position.z  = float(approach_lf[2]) + 0.05
        txt.pose.orientation.w = 1.0
        txt.scale.z  = 0.025   # text height in metres
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

    # ─────────────────────────────────────────────────────────────────────────

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
        Extract button plane pose from ZED2 organised point cloud.

        The ZED2 publishes cloud_registered in zed2_left_camera_frame
        (non-optical, X-forward ROS convention). The normal direction
        enforcement uses a frame-agnostic dot-product check against the
        camera-origin vector rather than assuming a fixed forward axis.

        Steps:
          1. Decode raw PointCloud2 bytes directly (fast, no Python list).
             Handles row-padding correctly via np.ascontiguousarray.
             Uses .copy() before .view(float32) to guarantee contiguity.
             2. Scale image-space bbox into cloud pixel space.
             3. Apply center-shrunk crop inside bbox to reduce wall dominance.
             4. Remove NaN/inf and out-of-range points.
             5. Median Euclidean-depth outlier filter (rejects specular spikes).
             6. RANSAC plane fit (5 mm inlier threshold, 200 iterations).
             7. SVD refinement on inlier subset — gives best-fit plane normal
             and a scalar D for the plane equation n·p + D = 0.
             8. Frame-agnostic normal enforcement: dot(normal, -centroid) > 0.
             9. CHANGED: Button center = bbox center ray projected onto fitted plane.
             The bounding box center pixel defines a camera ray in the optical
             frame. That ray is transformed into the cloud frame via TF2 and
             intersected with the RANSAC-fitted plane. This gives the correct
             geometric center of the button regardless of where the RANSAC
             inliers happen to cluster within the bounding box.
             Fallback to inlier centroid if the TF lookup or intersection fails.
             10. Compute approach and press offsets along the plane normal from
             the button center computed in step 7.
             11. Transform both points to TARGET_FRAME (link_0_fake) via TF2.

        Returns dict with keys:
          approach_lf, press_lf, normal_cam, centroid_cam, button_center_lf,
          inlier_count, viz_pts, inlier_mask, cloud_frame
        or None on failure.
        """
        # Save original bbox center BEFORE clipping — this is the pixel that
        # represents the true center of the detected button bounding box.
        bbox_u_c = (bbox[0] + bbox[2]) / 2.0
        bbox_v_c = (bbox[1] + bbox[3]) / 2.0

        if image_width <= 0 or image_height <= 0:
            self.get_logger().warn('[PC-RANSAC] Invalid image resolution for bbox scaling.')
            return None

        # Scale YOLO bbox from image pixel grid to cloud pixel grid.
        sx = cloud_msg.width / float(image_width)
        sy = cloud_msg.height / float(image_height)

        x1, y1, x2, y2 = bbox
        x1 = int(math.floor(x1 * sx))
        y1 = int(math.floor(y1 * sy))
        x2 = int(math.ceil(x2 * sx))
        y2 = int(math.ceil(y2 * sy))

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(cloud_msg.width, x2)
        y2 = min(cloud_msg.height, y2)
        if x2 <= x1 or y2 <= y1:
            return None

        # Use a center-focused crop to reduce wall points around the button.
        roi_w = x2 - x1
        roi_h = y2 - y1
        cx_roi = 0.5 * (x1 + x2)
        cy_roi = 0.5 * (y1 + y2)
        shr_w = max(2.0, roi_w * BBOX_CENTER_SHRINK)
        shr_h = max(2.0, roi_h * BBOX_CENTER_SHRINK)

        x1s = int(math.floor(cx_roi - 0.5 * shr_w))
        y1s = int(math.floor(cy_roi - 0.5 * shr_h))
        x2s = int(math.ceil(cx_roi + 0.5 * shr_w))
        y2s = int(math.ceil(cy_roi + 0.5 * shr_h))

        x1 = max(x1, x1s)
        y1 = max(y1, y1s)
        x2 = min(x2, x2s)
        y2 = min(y2, y2s)
        if x2 <= x1 or y2 <= y1:
            return None

        # ── Step 1: fast raw-byte decode of organised point cloud ─────────────
        #
        # Layout: cloud_msg.data is a flat byte buffer of length H * row_step.
        # Each row contains W points, each of point_step bytes.
        # row_step >= W * point_step (may include trailing padding bytes).
        # x, y, z are float32 values at known byte offsets within each point.
        #
        # Two contiguity guarantees are required for np.view(float32):
        #   (a) The (H, row_step) → crop of width W*point_step → reshape to
        #       (H*W, point_step) may be non-contiguous if row_step > W*point_step.
        #       Fixed by np.ascontiguousarray() before reshape.
        #   (b) Slicing columns from a C-contiguous 2D array always produces
        #       a non-contiguous array. Fixed by .copy() before .view().

        field_offsets = {f.name: f.offset for f in cloud_msg.fields}
        if not all(n in field_offsets for n in ('x', 'y', 'z')):
            self.get_logger().warn('[PC-RANSAC] PointCloud2 missing x/y/z fields.')
            return None

        x_off      = field_offsets['x']
        y_off      = field_offsets['y']
        z_off      = field_offsets['z']
        point_step = cloud_msg.point_step
        row_step   = cloud_msg.row_step

        if point_step <= max(x_off, y_off, z_off) + 3:
            self.get_logger().warn(
                '[PC-RANSAC] point_step too small for x/y/z offsets.')
            return None

        raw = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        expected_bytes = cloud_msg.height * row_step
        if raw.size != expected_bytes:
            self.get_logger().warn(
                f'[PC-RANSAC] Cloud byte-size mismatch: '
                f'got {raw.size}, expected {expected_bytes}')
            return None

        # Reshape to (H, row_step), strip row padding, reshape to (H*W, point_step)
        # np.ascontiguousarray ensures the result is C-contiguous before reshape
        # even when row_step > width * point_step (padding present).
        raw_2d = raw.reshape(cloud_msg.height, row_step)
        packed = np.ascontiguousarray(
            raw_2d[:, :cloud_msg.width * point_step]
        ).reshape(cloud_msg.height * cloud_msg.width, point_step)

        f32 = np.dtype('>f4' if cloud_msg.is_bigendian else '<f4')

        # .copy() makes each 4-byte column slice C-contiguous before .view()
        x_col = packed[:, x_off:x_off + 4].copy().view(f32).reshape(-1)
        y_col = packed[:, y_off:y_off + 4].copy().view(f32).reshape(-1)
        z_col = packed[:, z_off:z_off + 4].copy().view(f32).reshape(-1)

        pts  = np.column_stack((x_col, y_col, z_col)).reshape(
            cloud_msg.height, cloud_msg.width, 3)

        # Crop to YOLO bounding box: pts[row, col] == pts[v, u]
        crop = pts[y1:y2, x1:x2, :].reshape(-1, 3).astype(np.float64)

        # ── Step 2: remove NaN/inf and out-of-range points ────────────────────
        valid  = np.isfinite(crop).all(axis=1)
        crop   = crop[valid]
        if len(crop) == 0:
            return None

        # Euclidean distance from camera origin is frame-agnostic depth proxy
        depths   = np.linalg.norm(crop, axis=1)
        in_range = (depths > 0.05) & (depths <= 5.0)
        crop     = crop[in_range]
        if len(crop) < 8:
            return None

        # ── Step 3: near-depth gate (bias toward button face over wall) ──────
        depths = np.linalg.norm(crop, axis=1)
        near_depth = float(np.percentile(depths, NEAR_DEPTH_PERCENTILE))
        near_mask = depths <= (near_depth + NEAR_DEPTH_BAND_M)
        crop = crop[near_mask]
        if len(crop) < 8:
            return None

        # ── Step 4: median-depth outlier filter (rejects specular spikes) ────
        depths       = np.linalg.norm(crop, axis=1)
        median_depth = float(np.median(depths))
        close_mask   = np.abs(depths - median_depth) <= 0.02
        crop         = crop[close_mask]
        if len(crop) < 8:
            return None

        # ── Keep a copy of all valid crop points for visualization ────────────
        # This is saved here, after all pre-filtering but before RANSAC,
        # so the visualization shows exactly which points RANSAC received.
        viz_pts = crop.copy()

        # ── Step 5: RANSAC plane fit ─────────────────────────────────────────
        DIST_THRESH = 0.003   # 3 mm inlier threshold
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
                continue              # degenerate / collinear sample — skip
            normal /= norm_len
            d       = -np.dot(normal, p1)

            # Vectorised inlier count: point-to-plane distance < threshold
            dists = np.abs(crop @ normal + d)
            mask  = dists < DIST_THRESH
            count = int(mask.sum())
            if count > best_count:
                best_count = count
                best_mask  = mask

        if best_mask is None or best_count < MIN_INLIERS:
            return None

        # ── Step 6: SVD refinement on inlier subset ──────────────────────────
        inliers          = crop[best_mask]
        inlier_centroid  = inliers.mean(axis=0)   # any point on the plane
        _, _, Vt         = np.linalg.svd(inliers - inlier_centroid,
                                          full_matrices=False)
        # Last row of Vt = direction of minimum variance = plane normal
        normal   = Vt[-1].copy()
        if np.linalg.norm(normal) < 1e-8:
            return None
        normal /= np.linalg.norm(normal)

        # ── Step 7: frame-agnostic normal direction enforcement ───────────────
        # Camera origin is [0, 0, 0] in the cloud frame.
        # The normal must point FROM the button surface TOWARD the camera,
        # i.e. dot(normal, camera_origin - inlier_centroid) > 0.
        # This is correct for both optical (Z-forward) and non-optical (X-forward)
        # cloud frames — no axis assumption is made.
        toward_camera      = -inlier_centroid
        toward_camera_norm = np.linalg.norm(toward_camera)
        if toward_camera_norm < 1e-8:
            return None
        toward_camera /= toward_camera_norm
        if np.dot(normal, toward_camera) < 0:
            normal = -normal

        # ── Step 8: button center = bbox center ray projected onto fitted plane ─
        #
        # The RANSAC inlier mean is NOT used as the button center. Instead:
        #   a. The bounding box center pixel (bbox_u_c, bbox_v_c) defines a
        #      camera ray in the optical frame using the pinhole model.
        #   b. The ray direction vector is transformed from the optical frame
        #      into the cloud frame via TF2 (pure rotation, same physical origin).
        #   c. The transformed ray is intersected with the RANSAC-fitted plane
        #      (expressed in cloud frame) to find the exact 3D button center.
        #
        # This decouples the lateral position estimate (bbox center pixel, which
        # is the most reliable measure of where the button center is in the image)
        # from the depth estimate (RANSAC plane, which gives the most reliable
        # measure of how far away the button surface is).
        #
        # Fallback: if the TF lookup or intersection fails, use inlier_centroid.

        cam_frame  = cloud_msg.header.frame_id or CAMERA_FRAME

        # Plane scalar D for the SVD-refined plane: n · p + D = 0
        # Using inlier_centroid as the known point on the plane.
        D_plane = -np.dot(normal, inlier_centroid)

        button_centroid = inlier_centroid   # default fallback

        # Build the pinhole ray direction in the optical frame
        dx_opt = (bbox_u_c - cx) / fx
        dy_opt = (bbox_v_c - cy) / fy
        dz_opt = 1.0   # Z=1 parametric convention

        ray_opt = Vector3Stamped()
        ray_opt.header.frame_id = optical_frame
        ray_opt.header.stamp    = rclpy.time.Time().to_msg()
        ray_opt.vector.x = float(dx_opt)
        ray_opt.vector.y = float(dy_opt)
        ray_opt.vector.z = float(dz_opt)

        try:
            # Transform the ray direction from optical frame to cloud frame.
            # Both frames share the same physical camera origin, so this is
            # purely rotational — no translation needed for a direction vector.
            tf_opt_to_cloud = self._tf_buffer.lookup_transform(
                cam_frame, optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            ray_cloud_msg = tf2_geometry_msgs.do_transform_vector3(
                ray_opt, tf_opt_to_cloud)
            d_cloud = np.array(
                [ray_cloud_msg.vector.x,
                 ray_cloud_msg.vector.y,
                 ray_cloud_msg.vector.z],
                dtype=np.float64)

            # Ray-plane intersection in cloud frame:
            #   P(t) = t * d_cloud  (ray from camera origin [0,0,0])
            #   Plane: n · P + D_plane = 0
            #   → t = -D_plane / dot(n, d_cloud)
            denom = np.dot(normal, d_cloud)
            if abs(denom) < 1e-6:
                self.get_logger().warn(
                    '[PC-RANSAC] Bbox center ray parallel to fitted plane — '
                    'falling back to inlier centroid.')
            else:
                t = -D_plane / denom
                if t <= 0.0:
                    self.get_logger().warn(
                        '[PC-RANSAC] Bbox center ray intersection behind camera — '
                        'falling back to inlier centroid.')
                else:
                    button_centroid = t * d_cloud
                    self.get_logger().info(
                        '[PC-RANSAC] Button center via ray-plane intersection: '
                        f'({button_centroid[0]:.4f},{button_centroid[1]:.4f},'
                        f'{button_centroid[2]:.4f})  '
                        f'inlier_centroid: '
                        f'({inlier_centroid[0]:.4f},{inlier_centroid[1]:.4f},'
                        f'{inlier_centroid[2]:.4f})  '
                        f'lateral_shift={np.linalg.norm(button_centroid - inlier_centroid):.4f}m')

        except Exception as exc:
            self.get_logger().warn(
                f'[PC-RANSAC] TF {optical_frame}→{cam_frame} for ray transform '
                f'failed: {exc} — falling back to inlier centroid.')

        # ── Step 9: approach and press points in cloud frame ──────────────────
        approach_cam = button_centroid + APPROACH_DIST_M * normal   # +5 cm outward
        press_cam    = button_centroid + PRESS_INSET_M   * normal   # -7 mm into button

        # ── Step 10: transform to TARGET_FRAME (link_0_fake) via TF2 ──────────
        # Use the actual frame_id from the cloud header (e.g. 'zed2_left_camera_frame')
        # rather than the optical-frame constant, because the cloud is in the
        # non-optical ROS frame.
        approach_lf      = self._to_link0(approach_cam,    cam_frame)
        press_lf         = self._to_link0(press_cam,       cam_frame)
        button_center_lf = self._to_link0(button_centroid, cam_frame)
        if approach_lf is None or press_lf is None or button_center_lf is None:
            return None

        return {
            'approach_lf':      approach_lf,
            'press_lf':         press_lf,
            'button_center_lf': button_center_lf,   # ray-plane intersection in link_0
            'normal_cam':       normal,
            'centroid_cam':     button_centroid,     # same point, in cloud frame
            'inlier_count':     best_count,
            # ── visualization data ───────────────────────────────────────────
            'viz_pts':     viz_pts,       # (N, 3) all valid points after pre-filtering
            'inlier_mask': best_mask,     # (N,) bool — True = RANSAC inlier
            'cloud_frame': cam_frame,     # frame_id for the coloured cloud publisher
        }

    # ── Arm press-complete subscriber ────────────────────────────────────────

    def _press_complete_cb(self, msg: Bool):
        """Arm controller publishes True to /press_complete when motion is done."""
        if msg.data:
            self._press_complete_event.set()

    # ── Door-open check via Nav2 path planner ────────────────────────────────

    def _door_is_open(self, inside_pose: PoseStamped) -> bool:
        """
        Return True if Nav2 can plan a path to inside_pose (door is open).

        Polls /compute_path_to_pose with GridBased planner.
        Returns False immediately if inside_pose has no frame_id set.
        Uses tight timeouts so the verification loop stays responsive.
        The MultiThreadedExecutor resolves the action client futures in its
        own threads while this method busy-waits.
        """
        if not inside_pose.header.frame_id:
            return False
        if not self._path_client.wait_for_server(
                timeout_sec=PATH_SERVER_WAIT_SEC):
            return False

        goal = Nav2ComputePathToPose.Goal()
        goal.goal       = inside_pose
        goal.planner_id = 'GridBased'

        future   = self._path_client.send_goal_async(goal)
        deadline = time.time() + PATH_GOAL_TIMEOUT_SEC
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False

        gh = future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        deadline2 = time.time() + PATH_RESULT_TIMEOUT_SEC
        while not result_future.done() and time.time() < deadline2:
            time.sleep(0.05)
        if not result_future.done():
            return False

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    # ── Post-press verification ───────────────────────────────────────────────

    def _verify_press(self, lit_class: int, inside_pose: PoseStamped,
                      deadline: float, goal_handle) -> str:
        """
        After the arm signals press complete, confirm via either:
          BUTTON_LIT : YOLO detects the lit class for VERIFY_LIT_MIN_FRAMES
                       consecutive frames (elevator on different floor — button
                       stays lit while travelling).
          DOOR_OPEN  : Nav2 ComputePathToPose to inside_pose succeeds
                       (elevator was on same floor — door opens almost immediately
                       after press; button lit window is only ~0.5 s and unreliable).

        Returns: 'BUTTON_LIT' | 'DOOR_OPEN' | 'TIMEOUT' | 'CANCELLED'
        """
        time.sleep(VERIFY_SETTLE_SEC)   # let door/button state settle after press
        consecutive_lit = 0

        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                return 'CANCELLED'

            # ── Door-open check (catches same-floor case) ─────────────────
            if self._door_is_open(inside_pose):
                return 'DOOR_OPEN'

            # ── YOLO lit-button check (catches different-floor case) ──────
            if self._latest_rgb is not None:
                try:
                    bgr = self._bridge.imgmsg_to_cv2(
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
                    self.get_logger().warn(f'[Verify] YOLO check error: {exc}')

            time.sleep(VERIFY_PATH_POLL_SEC)

        return 'TIMEOUT'

    # ── Action execution ──────────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):  # noqa: C901
        result   = DetectCallButton.Result()
        feedback = DetectCallButton.Feedback()

        current_floor = goal_handle.request.current_floor.strip()
        target_floor  = goal_handle.request.target_floor.strip()
        inside_pose   = goal_handle.request.inside_pose   # PoseStamped for door check

        try:
            direction = (
                'UP' if self._parse_floor(target_floor)
                      > self._parse_floor(current_floor)
                else 'DOWN'
            )
        except ValueError as exc:
            self.get_logger().error(f'Floor parse error: {exc}')
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        class_ids   = DIRECTION_CLASS_MAP[direction]
        unlit_class = class_ids['unlit_class']
        lit_class   = class_ids['lit_class']

        self.get_logger().info(
            f'[DetectCallButton] {current_floor}→{target_floor}: '
            f'{direction} (unlit={unlit_class}, lit={lit_class})')

        approach_lf = None
        press_lf    = None
        state       = _PressState.WAITING_FOR_DATA

        try:
            for attempt in range(MAX_PRESS_ATTEMPTS):
                if attempt > 0:
                    self.get_logger().info(
                        f'[DetectCallButton] Re-localising — press attempt '
                        f'{attempt + 1}/{MAX_PRESS_ATTEMPTS}')

                # Fresh sensor data each attempt
                self._latest_rgb   = None
                self._latest_cloud = None
                state = _PressState.WAITING_FOR_DATA

                # ══════════════════════════════════════════════════════════════
                # State: WAITING_FOR_DATA
                # ══════════════════════════════════════════════════════════════
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
                        feedback.status    = 'WAITING_FOR_DATA'
                        feedback.direction = direction
                        goal_handle.publish_feedback(feedback)
                        time.sleep(LOOP_SLEEP)

                if not (rclpy.ok() and goal_handle.is_active):
                    break

                # ══════════════════════════════════════════════════════════════
                # State: DETECTING
                # YOLO detection + RANSAC localisation loop.
                # Exits when button localised (→ AWAITING_PRESS) or
                # already lit (→ CONFIRMED).
                # ══════════════════════════════════════════════════════════════
                while state == _PressState.DETECTING:
                    if not (rclpy.ok() and goal_handle.is_active):
                        break
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.message = 'Cancelled.'
                        return result

                    # Convert RGB image
                    try:
                        bgr = self._bridge.imgmsg_to_cv2(
                            self._latest_rgb, desired_encoding='bgr8')
                    except Exception as exc:
                        self.get_logger().warn(f'Image conversion: {exc}')
                        time.sleep(LOOP_SLEEP)
                        continue

                    # Camera intrinsics
                    K        = self._camera_info.k
                    fx, fy   = K[0], K[4]
                    cx, cy   = K[2], K[5]
                    cam_frame = (self._camera_info.header.frame_id or CAMERA_FRAME)

                    # ── YOLO inference ────────────────────────────────────────
                    yolo_results  = self._model(bgr, conf=INFERENCE_CONF, verbose=False)
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
                            col = ((0, 255, 0) if cls_id == unlit_class
                                   else (0, 165, 255))
                            cv2.rectangle(annotated, (bx1, by1), (bx2, by2), col, 2)
                            lbl = (self._model.names[cls_id]
                                   if self._model.names else str(cls_id))
                            cv2.putText(annotated, f'{lbl} {conf:.2f}',
                                        (bx2 + 6, (by1 + by2) // 2),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

                    # ── Button already lit — call already active ──────────────
                    if detected_lit:
                        feedback.status    = 'BUTTON_LIT'
                        feedback.direction = direction
                        goal_handle.publish_feedback(feedback)
                        cv2.putText(annotated,
                                    f'{direction} LIT — already pressed',
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 165, 255), 2)
                        try:
                            self._debug_pub.publish(
                                self._bridge.cv2_to_imgmsg(
                                    annotated, encoding='bgr8'))
                        except Exception:
                            pass
                        # Already pressed — no arm action needed
                        approach_lf    = np.zeros(3)
                        press_lf       = np.zeros(3)
                        result.message = (
                            f'{direction} button already lit; call already active.')
                        state = _PressState.CONFIRMED
                        break

                    # ── No unlit button detected this frame ───────────────────
                    if not detected_unlit or unlit_bbox is None:
                        feedback.status    = 'NO_DETECTION'
                        feedback.direction = direction
                        goal_handle.publish_feedback(feedback)
                        cv2.putText(annotated,
                                    f'{direction} not found',
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

                    # ── ZED2 Point Cloud + RANSAC localisation ────────────────
                    pc_result = self._extract_button_pose_pointcloud(
                        self._latest_cloud, unlit_bbox,
                        int(self._latest_rgb.width), int(self._latest_rgb.height),
                        fx, fy, cx, cy, cam_frame)

                    if pc_result is None:
                        self.get_logger().warn(
                            '[PC-RANSAC] Failed — insufficient valid depth '
                            'points in YOLO bbox. Retrying.')
                        feedback.status    = 'DEPTH_INSUFFICIENT'
                        feedback.direction = direction
                        goal_handle.publish_feedback(feedback)
                        time.sleep(LOOP_SLEEP)
                        continue

                    # Localisation succeeded — cache waypoints and publish goal
                    approach_lf      = pc_result['approach_lf']
                    press_lf         = pc_result['press_lf']
                    button_center_lf = pc_result['button_center_lf']
                    centroid_cam     = pc_result['centroid_cam']
                    inlier_count     = int(pc_result['inlier_count'])

                    self.get_logger().info(
                        '[PC-RANSAC] inliers={}  '
                        'center_link0_fake=({:.4f},{:.4f},{:.4f})  '
                        'approach=({:.4f},{:.4f},{:.4f})  '
                        'press=({:.4f},{:.4f},{:.4f})  '
                        '[PUBLISHING /button_press_goal]'.format(
                            inlier_count,
                            button_center_lf[0], button_center_lf[1],
                            button_center_lf[2],
                            approach_lf[0],      approach_lf[1],      approach_lf[2],
                            press_lf[0],         press_lf[1],         press_lf[2],
                        ))

                    self._publish_button_press_goal(approach_lf, press_lf, direction, button_center_lf)
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

                    feedback.status    = 'LOCALISED'
                    feedback.direction = direction
                    goal_handle.publish_feedback(feedback)
                    state = _PressState.AWAITING_PRESS
                    break  # exit DETECTING loop

                # Guard: node/goal still alive?
                if not (rclpy.ok() and goal_handle.is_active):
                    break

                if state == _PressState.CONFIRMED:
                    break  # already-lit fast path

                if state != _PressState.AWAITING_PRESS:
                    continue  # DETECTING loop ended without localising — retry

                # ══════════════════════════════════════════════════════════════
                # State: AWAITING_PRESS
                # Block until arm controller publishes True to /press_complete
                # or ARM_TIMEOUT_SEC elapses.
                # ══════════════════════════════════════════════════════════════
                self._press_complete_event.clear()
                feedback.status    = 'AWAITING_PRESS'
                feedback.direction = direction
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f'[DetectCallButton] Awaiting /press_complete '
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
                        f'[DetectCallButton] No /press_complete within '
                        f'{ARM_TIMEOUT_SEC:.0f}s '
                        f'(attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')
                    continue  # next attempt — re-localise and re-publish

                # ══════════════════════════════════════════════════════════════
                # State: VERIFYING
                # Check BUTTON_LIT (different floor) or DOOR_OPEN (same floor).
                # ══════════════════════════════════════════════════════════════
                feedback.status    = 'VERIFYING'
                feedback.direction = direction
                goal_handle.publish_feedback(feedback)

                verify_deadline = time.time() + VERIFY_TIMEOUT_SEC
                reason = self._verify_press(
                    lit_class, inside_pose, verify_deadline, goal_handle)

                self.get_logger().info(
                    f'[DetectCallButton] Verification: {reason} '
                    f'(attempt {attempt + 1}/{MAX_PRESS_ATTEMPTS})')

                if reason == 'CANCELLED':
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled during press verification.'
                    return result

                if reason in ('BUTTON_LIT', 'DOOR_OPEN'):
                    state = _PressState.CONFIRMED
                    result.message = (
                        f'{direction} press confirmed via {reason}. '
                        f'approach=({approach_lf[0]:.4f},{approach_lf[1]:.4f},'
                        f'{approach_lf[2]:.4f}) '
                        f'press=({press_lf[0]:.4f},{press_lf[1]:.4f},'
                        f'{press_lf[2]:.4f})')
                    break

                self.get_logger().warn(
                    f'[DetectCallButton] Press not confirmed — '
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
            else:
                if rclpy.ok() and goal_handle.is_active:
                    goal_handle.abort()
                result.success = False
                if not result.message:
                    result.message = (
                        f'Press not confirmed after '
                        f'{MAX_PRESS_ATTEMPTS} attempts.')
            return result

        finally:
            # Keep subscriptions alive; only clear per-goal cached messages.
            self._latest_rgb   = None
            self._latest_cloud = None
            self.get_logger().info(
                '[DetectCallButton] Goal finished; sensor caches reset.')


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