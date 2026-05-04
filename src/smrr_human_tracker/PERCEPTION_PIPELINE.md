# SMRR Human Tracker — Perception Pipeline Architecture

## Overview

The SMRR Human Tracker implements a multi-sensor human detection and tracking pipeline designed for a mobile service robot operating in crowded indoor environments. The system fuses data from a 2D LiDAR scanner (360° coverage) and a ZED2 stereo camera (RGB + depth) to achieve robust, full-environment awareness of nearby humans. The pipeline is built on three cooperating ROS2 nodes, launched together by `human_fusion_kf.launch.py`.

The fundamental design philosophy is **sensor complementarity**: each sensor covers the other's weaknesses. The LiDAR provides 360° spatial coverage but cannot distinguish humans from non-human obstacles reliably in complex environments. The camera provides high-confidence human identification via deep learning but is limited to a narrow field of view and struggles with depth accuracy at range. Together, they produce a stable, spatially-accurate, wide-coverage human tracking output.

---

## High-Level System Block Diagram

```
╔══════════════════════════════════════════════════════════════════════════════════════╗
║                      MULTI-SENSOR HUMAN PERCEPTION PIPELINE                         ║
║                       (human_fusion_kf.launch.py)                                   ║
╚══════════════════════════════════════════════════════════════════════════════════════╝

  ┌────────────────────────────┐          ┌──────────────────────────────────────┐
  │       2D LiDAR Sensor      │          │         ZED2 Stereo Camera           │
  │  /scan (LaserScan, 360°)   │          │  /zed2_left_camera/image_raw (RGB)   │
  │                            │          │  /zed2_left_camera/depth/image_raw   │
  └────────────┬───────────────┘          │  /zed2_left_camera/camera_info       │
               │                          └────────────────────┬─────────────────┘
               │                                               │
               ▼                                               ▼
  ┌────────────────────────────┐          ┌──────────────────────────────────────┐
  │   STAGE 1: LiDAR Masking   │          │  STAGE 2: YOLO26n Human Detection    │
  │  (Static Obstacle Filter)  │          │       (Instance Segmentation)        │
  │                            │          │                                      │
  │  Input: /map (Occupancy    │          │  Input: RGB + Depth + CameraInfo     │
  │    Grid, latched)          │          │                                      │
  │                            │          │  YOLO26n-seg.engine (TensorRT FP16) │
  │  Method:                   │          │  + ByteTrack/BoTSORT tracking        │
  │  - Binary map (free/wall)  │          │                                      │
  │  - Morphological dilation  │          │  Output:                             │
  │    (elliptical, 0.35m)     │          │  - Bounding boxes + segmentation     │
  │  - Vectorized O(1) lookup  │          │    masks per detected human          │
  │  - Wall hits → 29.99m      │          │  - Persistent track IDs              │
  │                            │          │  - 3D positions via depth unproject  │
  │  Output: Filtered scan     │          │  - Confidence scores                 │
  │  (humans + dynamic objs    │          │  - Camera-frame → map-frame via TF   │
  │   only, walls removed)     │          │                                      │
  └────────────┬───────────────┘          │  → tracked_humans/poses (PoseArray) │
               │                          └────────────────────┬─────────────────┘
               ▼                                               │
  ┌────────────────────────────┐                               │
  │  STAGE 3: DR-SPAAM Human   │                               │
  │  Detection (2D LiDAR DNN)  │                               │
  │                            │                               │
  │  Input: Filtered scan      │                               │
  │                            │                               │
  │  Method:                   │                               │
  │  - DR-SPAAM neural net     │                               │
  │    (ONNX Runtime, ~15 FPS) │                               │
  │  - Spatial attention       │                               │
  │  - NMS* centroid averaging │                               │
  │    (cluster radius 0.5m)   │                               │
  │  - Confidence thresholding │                               │
  │    (default: 0.3)          │                               │
  │  - Lidar → map-frame TF    │                               │
  │                            │                               │
  │  Output: 2D human          │                               │
  │  positions + confidence    │                               │
  │  → detected_people         │                               │
  │    (PoseArray)             │                               │
  └────────────┬───────────────┘                               │
               │                                               │
               └──────────────────────┬────────────────────────┘
                                      │
                                      ▼
  ╔══════════════════════════════════════════════════════════════╗
  ║         STAGE 4: SENSOR FUSION DECISION LOGIC               ║
  ║         (HumanFusionKFNode — human_fusion_kf_node.py)       ║
  ╚══════════════════════════════════════════════════════════════╝

  ┌───────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                   │
  │   Inputs time-synchronized via ApproximateTimeSynchronizer (slop = 150 ms):      │
  │     • tracked_humans/poses   ← YOLO26n detections (map frame, PoseArray)         │
  │     • detected_people        ← DR-SPAAM detections (map frame, PoseArray)        │
  │                                                                                   │
  │   ┌─────────────────────────────────────────────────────────────────────────┐    │
  │   │  STEP 4.1 — FOV Partitioning: Classify each detection by sensor zone    │    │
  │   │                                                                         │    │
  │   │  Camera FOV = ±55° (110° total) around robot heading (base_link)        │    │
  │   │                                                                         │    │
  │   │  Every LiDAR detection is tested:                                       │    │
  │   │    - angle to robot heading (via TF to base_link)                       │    │
  │   │    - if |angle| ≤ 55°  →  "IN camera FOV"                              │    │
  │   │    - if |angle| > 55°  →  "OUT of camera FOV (blind spot)"             │    │
  │   └────────────────────────────────┬────────────────────────────────────────┘    │
  │                                    │                                              │
  │   ┌────────────────────────────────▼────────────────────────────────────────┐    │
  │   │  STEP 4.2 — Hungarian Matching: YOLO ↔ LiDAR within camera FOV         │    │
  │   │                                                                         │    │
  │   │  Build cost matrix: Euclidean distance between all YOLO ↔ LiDAR pairs  │    │
  │   │  Apply Hungarian algorithm for globally optimal 1-to-1 assignment      │    │
  │   │  Match threshold: 1.0 m (fusion_distance_threshold)                    │    │
  │   │                                                                         │    │
  │   │  Results:                                                               │    │
  │   │    ① MATCHED pairs   (both YOLO + LiDAR agree on same human)           │    │
  │   │    ② YOLO-only       (YOLO detects human, no nearby LiDAR)             │    │
  │   │    ③ LiDAR-only (IN) (LiDAR detects something, no YOLO match)          │    │
  │   │    ④ LiDAR-only (OUT)(LiDAR detection outside camera FOV)              │    │
  │   └────────────────────────────────┬────────────────────────────────────────┘    │
  │                                    │                                              │
  │   ┌────────────────────────────────▼────────────────────────────────────────┐    │
  │   │  STEP 4.3 — Fusion Decision Logic (per detection)                       │    │
  │   │                                                                         │    │
  │   │  CASE ①: MATCHED (YOLO + LiDAR within 1.0m)                           │    │
  │   │  ───────────────────────────────────────────                           │    │
  │   │  → ACCEPT as high-confidence detection                                 │    │
  │   │  → Spatial position: taken from LiDAR (trusted for accuracy)           │    │
  │   │  → Confidence: min(yolo_conf + lidar_conf, 1.0)  [boosted]             │    │
  │   │  → Rationale: both sensors agree → almost certainly a real human        │    │
  │   │                                                                         │    │
  │   │  CASE ②: YOLO-only (inside camera FOV, no LiDAR match)                │    │
  │   │  ─────────────────────────────────────────────────────                 │    │
  │   │  → ACCEPT, using camera depth for position                             │    │
  │   │  → Position: computed via ZED2 depth image + camera intrinsics         │    │
  │   │    - Primary: segmentation mask weighted depth average                 │    │
  │   │    - Fallback: center-region 20% padded bounding box average           │    │
  │   │  → Confidence: YOLO score (unmodified)                                 │    │
  │   │  → Rationale: person visible to camera, may be out of LiDAR range     │    │
  │   │    or occluded in 2D plane (e.g., legs behind cart)                    │    │
  │   │                                                                         │    │
  │   │  CASE ③: LiDAR-only inside camera FOV (no YOLO match)                 │    │
  │   │  ──────────────────────────────────────────────────────                │    │
  │   │  → DISCARD (false positive suppression)                                │    │
  │   │  → Rationale: camera is looking at this region and did NOT detect      │    │
  │   │    a human there — LiDAR must be detecting a non-human obstacle        │    │
  │   │    (e.g., chair leg, reflective surface, narrow post). Camera          │    │
  │   │    acts as a veto within its FOV.                                       │    │
  │   │                                                                         │    │
  │   │  CASE ④: LiDAR-only outside camera FOV (blind spot)                   │    │
  │   │  ─────────────────────────────────────────────────────                 │    │
  │   │  → ACCEPT unconditionally (no camera veto possible)                    │    │
  │   │  → Position: taken from LiDAR (sole sensor)                            │    │
  │   │  → Confidence: DR-SPAAM score (unmodified)                             │    │
  │   │  → Rationale: provides 360° awareness. People behind/beside the        │    │
  │   │    robot must be tracked for safe navigation even without visual        │    │
  │   │    confirmation. False positive rate accepted for blind-spot safety.   │    │
  │   └────────────────────────────────┬────────────────────────────────────────┘    │
  │                                    │                                              │
  └────────────────────────────────────┼──────────────────────────────────────────── ┘
                                       │
                                       ▼
  ╔══════════════════════════════════════════════════════════════╗
  ║       STAGE 5: IMM KALMAN FILTER TRACKING (IMMFilter)       ║
  ╚══════════════════════════════════════════════════════════════╝

  ┌───────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                   │
  │  Input: Fused detection list from Stage 4                                        │
  │  Output: Smoothed, persistent track estimates with velocity                      │
  │                                                                                   │
  │  ┌──────────────────────────────────────────────────────────────────────────┐    │
  │  │  STEP 5.1 — Track-to-Detection Association                               │    │
  │  │                                                                          │    │
  │  │  For every existing track, compute Mahalanobis distance to each          │    │
  │  │  incoming fused detection. Hungarian algorithm finds optimal assignment. │    │
  │  │  Gating: Mahalanobis < 3.0 σ  AND  physical distance < 1.0 m            │    │
  │  │  Unmatched tracks → COASTING; unmatched detections → new track init     │    │
  │  └──────────────────────────────────────────────────────────────────────────┘    │
  │                                                                                   │
  │  ┌──────────────────────────────────────────────────────────────────────────┐    │
  │  │  STEP 5.2 — IMM Prediction Step (3-model ensemble)                       │    │
  │  │                                                                          │    │
  │  │  State vector: [x, y, vx, vy, ω]  (position, velocity, turn rate)       │    │
  │  │                                                                          │    │
  │  │  Model 1 — Constant Velocity (CV):                                       │    │
  │  │    Assumes steady linear motion. x += vx·dt, y += vy·dt.                │    │
  │  │    Best for people walking in straight lines.                            │    │
  │  │                                                                          │    │
  │  │  Model 2 — Coordinated Turn (CT):                                        │    │
  │  │    Assumes curved path with turn rate ω. Uses nonlinear kinematic        │    │
  │  │    equations (sinc functions). Best for people turning corners.          │    │
  │  │                                                                          │    │
  │  │  Model 3 — Brownian Motion (BM):                                         │    │
  │  │    Assumes near-stationary. Velocity decays toward zero.                 │    │
  │  │    Best for people standing still or slowly milling around.              │    │
  │  │                                                                          │    │
  │  │  SPENCER Markov transition matrix governs probability of switching       │    │
  │  │  between models. Each model maintains its own covariance. Mixed          │    │
  │  │  output is a probability-weighted combination of all three.              │    │
  │  └──────────────────────────────────────────────────────────────────────────┘    │
  │                                                                                   │
  │  ┌──────────────────────────────────────────────────────────────────────────┐    │
  │  │  STEP 5.3 — IMM Update Step                                              │    │
  │  │                                                                          │    │
  │  │  When a matched detection is available:                                  │    │
  │  │    - Each model independently updates via EKF (H measures x, y only)    │    │
  │  │    - Innovation clamping: max physical pull = 0.4 m/frame (~4 m/s)      │    │
  │  │    - Likelihood computed per model (Gaussian measurement model)          │    │
  │  │    - Bayesian mode probability update: p(model | z) ∝ L · p_prior       │    │
  │  │    - Output state = Σ p(model) · state(model)                           │    │
  │  │                                                                          │    │
  │  │  When coasting (no match):                                               │    │
  │  │    - Prediction only, no measurement update                              │    │
  │  │    - Covariance expands (uncertainty grows)                              │    │
  │  └──────────────────────────────────────────────────────────────────────────┘    │
  │                                                                                   │
  │  ┌──────────────────────────────────────────────────────────────────────────┐    │
  │  │  STEP 5.4 — Track Lifecycle (SPENCER State Machine)                      │    │
  │  │                                                                          │    │
  │  │                      ┌──── TENTATIVE ─────┐                             │    │
  │  │                      │  (unconfirmed,      │                             │    │
  │  │                      │   not published)    │                             │    │
  │  │                      └──┬──────────────────┘                            │    │
  │  │      3 hits (or 1 if    │                 3 misses                       │    │
  │  │      visually confirmed)│                    │                           │    │
  │  │                         ▼                    ▼                           │    │
  │  │                       ┌─────┐          ┌─────────┐                      │    │
  │  │             match ──▶ │ NEW │          │ DELETED │                      │    │
  │  │                       └──┬──┘          └─────────┘                      │    │
  │  │                          │                                               │    │
  │  │                        match                                             │    │
  │  │                          │                                               │    │
  │  │                          ▼                                               │    │
  │  │                      ┌─────────┐    no match    ┌────────┐              │    │
  │  │                      │ MATCHED │ ─────────────▶ │ MISSED │              │    │
  │  │                      └────▲────┘                └───┬────┘              │    │
  │  │                           │                         │                   │    │
  │  │                           └────── match ────────────┘                   │    │
  │  │                                          miss > limit → DELETED         │    │
  │  │                                          (30 frames if mature ≥20 hits) │    │
  │  │                                          (5 frames if fresh < 20 hits)  │    │
  │  └──────────────────────────────────────────────────────────────────────────┘    │
  │                                                                                   │
  │  ┌──────────────────────────────────────────────────────────────────────────┐    │
  │  │  STEP 5.5 — Static False Positive Gating                                 │    │
  │  │                                                                          │    │
  │  │  If a track's velocity remains < 0.1 m/s for > 2.0 seconds:             │    │
  │  │    - Confidence capped at 0.2 (suppressed from downstream use)          │    │
  │  │  Exception: if track was confirmed visually (has YOLO history),          │    │
  │  │    this gate is bypassed (person may just be standing still)             │    │
  │  └──────────────────────────────────────────────────────────────────────────┘    │
  │                                                                                   │
  └────────────────────────────────────────┬──────────────────────────────────────── ┘
                                           │
                                           ▼
  ╔══════════════════════════════════════════════════════════════╗
  ║                   STAGE 6: OUTPUT PUBLISHING                ║
  ╚══════════════════════════════════════════════════════════════╝

  ┌───────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                   │
  │  Publish threshold: confidence ≥ 0.3                                             │
  │                                                                                   │
  │  → fused_humans_kf/poses  (geometry_msgs/PoseArray, map frame)                  │
  │      pose.position.x/y    = KF-estimated position (metres)                       │
  │      pose.position.z      = confidence score [0.0–1.0]                           │
  │      pose.orientation.x   = vx (velocity x-component, m/s)                      │
  │      pose.orientation.y   = vy (velocity y-component, m/s)                      │
  │      pose.orientation.w   = |v| velocity magnitude, capped at 5.0 m/s           │
  │                                                                                   │
  │  → fused_humans_kf/markers  (visualization_msgs/MarkerArray)                    │
  │      Orange cylinders at estimated positions                                     │
  │      Velocity arrows indicating direction and speed                              │
  │      Text labels: track ID, speed, confidence                                   │
  │                                                                                   │
  └───────────────────────────────────────────────────────────────────────────────────┘
```

---

## Stage 1: LiDAR Masking (Static Obstacle Filtering)

### Purpose

The raw 2D LiDAR scan contains returns from all surfaces in the environment: walls, furniture, door frames, columns, and humans. If this unfiltered scan is fed directly into the DR-SPAAM human detector, the network sees reflections from static infrastructure as potential human leg clusters, producing a high rate of false positives. The masking stage removes all returns that correspond to known static obstacles before the scan reaches the neural network.

### Input

- `/scan` — `sensor_msgs/LaserScan`: the raw 360° laser scan at robot update rate.
- `/map` — `nav_msgs/OccupancyGrid`: the occupancy grid produced by the SLAM system (latched topic, received once and reused). Cell values: free (0–50), occupied/wall (>50), unknown (−1).

### Implementation

On first receipt of the map, the node converts it to a binary image:

- Free cells (value 0–50) → 0 (passable).
- Occupied and unknown cells (value >50 or −1) → 255 (blocked).

A morphological dilation is applied with an elliptical structuring element sized to correspond to a 0.35 m safety margin around each obstacle. This inflation serves two purposes: it absorbs the physical width of thin obstacles such as chair legs (which might not be fully represented in the occupancy grid), and it provides a tolerance buffer for small pose errors between the LiDAR frame and the map frame at the moment of the lookup.

The result is a binary NumPy boolean array (`inflated_map`) stored in memory. For every incoming scan, each beam's endpoint is converted from polar coordinates to Cartesian, transformed to the map frame using a 3×3 homogeneous matrix (vectorised across all beams simultaneously), and the corresponding cell in `inflated_map` is looked up in O(1). Any beam whose endpoint falls inside an inflated obstacle cell has its range value overwritten with 29.99 m — the maximum range sentinel — effectively making it invisible to the downstream detector. The resulting filtered scan is also published on `/filtered_scan` for debugging.

### Why This Matters

Without this step, DR-SPAAM regularly reports spurious detections at wall corners, chair clusters, and narrow pillars. By applying the map knowledge, the pipeline constrains DR-SPAAM to only see dynamic, uncharted objects — greatly improving precision in static environments.

---

## Stage 2: YOLO26n Human Detection (Camera Branch)

### Purpose

The camera branch runs a deep learning-based instance segmentation model on the RGB image stream from the ZED2 left camera. It extracts precise bounding boxes and per-pixel segmentation masks for every detected human, pairs those with depth measurements from the ZED2 depth channel, and reprojects each detection into a 3D position in the map frame using the camera intrinsic matrix.

### Input

- `/zed2_left_camera/image_raw` — `sensor_msgs/Image`: rectified RGB image.
- `/zed2_left_camera/depth/image_raw` — `sensor_msgs/Image`: per-pixel depth map in metres (32-bit float).
- `/zed2_left_camera/camera_info` — `sensor_msgs/CameraInfo`: intrinsic matrix K (stored once at startup).

### YOLO26n-seg Model

The model is YOLOv26n-seg — a nano-scale instance segmentation model derived from the YOLOv8 family. The `.engine` (TensorRT FP16) variant is used by default on the Jetson Orin, achieving 20–30 FPS. A `.pt` (PyTorch) fallback is available for CPU testing.

The model outputs, for each detected human (class 0 — person):
- Bounding box (x1, y1, x2, y2) in pixels.
- Segmentation mask (binary, per-pixel, same resolution as input).
- Class confidence score.

### ByteTrack / BoTSORT Integration

Frame-by-frame detections are associated across time using ByteTrack (default) or BoTSORT. These algorithms assign persistent integer track IDs to each detected human, enabling downstream filtering to maintain identity continuity. The tracking runs inside the YOLO `track()` call and is GPU-accelerated on the Orin.

### 3D Deprojection

For each tracked human:

1. The segmentation mask is applied to the depth image, extracting only depth values within the human's silhouette (not background).
2. Invalid depth values (0.0, inf, NaN) are masked out.
3. The median depth of the valid pixels in the mask is taken as the representative distance Z to avoid outliers from reflective clothing or hair.
4. If the mask is unavailable or yields too few valid pixels, the fallback uses the centre 20% padded region of the bounding box.
5. The 3D position in camera frame is:
   - `X = (u − cx) · Z / fx`
   - `Y = (v − cy) · Z / fy`
   - `Z = depth`
   where (u, v) is the bounding box centroid and (cx, cy, fx, fy) are from the camera intrinsic matrix K.
6. Detections beyond `max_detection_distance` (15 m) are discarded.

### Frame Transformation (Adaptive 2-Step)

The ZED2 camera frame position is transformed to the map frame using an adaptive 2-step TF approach designed to work reliably with SLAM systems that update the map→odom transform asynchronously:

1. **Exact-time lookup**: `camera_frame → odom` at the exact image timestamp. This step is strict to avoid temporal desynchronisation with the ZED's odometry.
2. **Latest-time lookup**: `odom → map` at the latest available transform. This step is relaxed because the SLAM-produced `map→odom` correction updates at low frequency (typically <5 Hz) and there is no reason to demand temporal precision here.

This approach eliminates the "extrapolation into the future" TF errors that would otherwise occur when a fast camera feed (30 Hz) tries to look up SLAM transforms that have not yet been broadcast for that exact timestamp.

### Output

- `tracked_humans/poses` — `geometry_msgs/PoseArray`: one pose per tracked human in map frame. `position.z` encodes the confidence score. `orientation` is identity.
- `tracked_humans/markers` — `visualization_msgs/MarkerArray`: cylinder and text markers for RViz.

---

## Stage 3: DR-SPAAM Human Detection (LiDAR Branch)

### Purpose

DR-SPAAM (Distance-aware Real-time Spatio-temporal Attention-based People Mover) is a neural network designed specifically for detecting humans from 2D laser scans. It processes the filtered scan from Stage 1 and outputs a list of 2D positions and confidence scores for each detected human-shaped cluster in the scan.

### Input

- Filtered `sensor_msgs/LaserScan` from Stage 1 (static obstacles removed).

### DR-SPAAM Neural Network

DR-SPAAM operates by sliding a context window along the angular sequence of range measurements. It uses a spatial attention mechanism to learn the characteristic pattern of two human legs appearing at various distances and orientations in the scan. The model was pre-trained on the JRDB dataset (large-scale crowd scenarios) and fine-tuned for 20 epochs (`ckpt_jrdb_ann_ft_dr_spaam_e20`).

Two inference backends are supported:
- **PyTorch** (`.pth`): Uses the full DR-SPAAM model including spatial attention layers. ~10–12 FPS.
- **ONNX Runtime** (`.onnx`): A stateless CNN backbone with spatial attention removed (incompatible with static export). ~13–15 FPS (+27%). Selected automatically if the `.onnx` sibling file exists alongside the `.pth` file.

The ONNX runtime is wrapped in an `ONNXModelWrapper` class that provides a `__call__` interface identical to the PyTorch model, making the swap transparent to the rest of the node. For CUDA-accelerated ONNX inference on Jetson, the node automatically injects the cuDNN library path into `LD_LIBRARY_PATH` at startup.

### NMS* (Centroid Averaging)

Standard non-maximum suppression (NMS) simply picks the highest-confidence detection and suppresses nearby ones. DR-SPAAM uses a modified NMS* algorithm from ETH Zurich's pedestrian detection research:

1. Cluster all raw detections that fall within 0.5 m of each other.
2. For each cluster, compute the weighted centroid (confidence-weighted average of positions) and the average confidence.
3. Output one detection per cluster.

This is more robust than winner-takes-all NMS in crowded scenes where two high-confidence detections may both be partially overlapping a single human.

### Output

- `detected_people` — `geometry_msgs/PoseArray`: one pose per detected human in map frame. `position.z` encodes the DR-SPAAM confidence score.
- `detected_people_markers` — `visualization_msgs/Marker`: red circle line-list markers for RViz.

---

## Stage 4: Sensor Fusion Decision Logic

### Purpose

This is the core algorithmic stage where detections from the camera (YOLO26n) and the LiDAR (DR-SPAAM) are combined into a single, higher-confidence detection list. The logic is designed around the camera's reliable identification capability within its FOV and the LiDAR's 360° spatial coverage.

### Input

- `tracked_humans/poses` — YOLO26n detections from Stage 2.
- `detected_people` — DR-SPAAM detections from Stage 3.

Both topics are received through an `ApproximateTimeSynchronizer` with a 150 ms time tolerance, ensuring fusion always operates on temporally-matched data.

### 4.1 FOV Partitioning

Each LiDAR detection is classified by computing the bearing angle from the robot's `base_link` frame:

```
angle_to_detection = atan2(dy, dx)   (in robot frame)
in_fov = |angle_to_detection| ≤ 55°   (for 110° camera FOV)
```

This partitions LiDAR detections into:
- **In-FOV**: the camera can see this region → YOLO results are authoritative here.
- **Out-of-FOV (blind spot)**: the camera cannot see this region → LiDAR is the sole sensor.

### 4.2 Hungarian Matching (YOLO ↔ LiDAR within FOV)

A Euclidean distance cost matrix is built between all YOLO detections and all in-FOV LiDAR detections. The Hungarian algorithm (scipy `linear_sum_assignment`) finds the globally-optimal one-to-one assignment that minimises total matching distance. Pairs with distance > 1.0 m (the `fusion_distance_threshold`) are rejected — those sensor observations are too far apart to plausibly be the same physical person.

### 4.3 Fusion Decision (Four Cases)

**Case 1 — Matched Pair (both sensors agree):**

The most reliable outcome. Both sensors have independently identified a human at approximately the same position. The fused position is taken from LiDAR (because LiDAR provides better absolute spatial accuracy in the 2D plane — depth cameras have range-dependent error that grows quadratically, while LiDAR range error is roughly constant). The confidence is boosted: `conf_fused = min(yolo_conf + lidar_conf, 1.0)`. Matching boosts confidence above what either sensor alone would provide.

**Case 2 — YOLO-only (camera sees human, LiDAR does not):**

Possible causes include: the human's legs are occluded by another object in the scan plane (e.g., a shopping cart), the human is within the LiDAR's minimum range, or the human is positioned such that DR-SPAAM's confidence threshold was not met. In these cases, the YOLO detection is accepted and its depth-derived 3D position is used. Confidence is the raw YOLO score.

**Case 3 — LiDAR-only inside camera FOV (LiDAR detects, camera does not):**

This is treated as a **false positive and discarded**. The camera's FOV covers this region, and a well-tuned YOLO26n model running at 0.5 confidence threshold has a low miss rate for clearly-visible humans. If the camera reports nothing, the LiDAR detection is likely a non-human object (a chair leg at leg-separation distance, a beverage cart, a reflective post) that happens to look like a human leg pair to the 2D detector. The camera acts as a veto authority within its field of view.

**Case 4 — LiDAR-only outside camera FOV (blind spot detection):**

There is no camera data for this region — the YOLO veto cannot be applied. The LiDAR detection is accepted unconditionally at its DR-SPAAM confidence. This ensures the robot is aware of humans approaching from the side or rear, which is essential for safe navigation in corridors and crowded spaces. The inherently higher false-positive rate in the blind spot is managed by the downstream Kalman filter's track initiation requirements (3 hits before a TENTATIVE track becomes NEW).

---

## Stage 5: IMM Kalman Filter Tracking

### Purpose

A single frame of fused detections is noisy. Sensor measurements have random error. People briefly disappear from view when occluded. The fusion logic may occasionally drop a detection or produce a spurious one. The Kalman filter stage converts these noisy, intermittent per-frame detections into smooth, continuous track estimates with associated velocities, enabling the navigation stack to plan around predicted future positions.

### Interacting Multiple Model (IMM) Filter

A standard Kalman filter assumes a fixed motion model (e.g., constant velocity). Humans exhibit diverse motion patterns: walking straight, turning, standing still. Choosing the wrong single model degrades tracking performance during behaviour transitions. The IMM filter addresses this by running three models simultaneously and combining their outputs with dynamically-adjusted probability weights.

**State vector**: `[x, y, vx, vy, ω]` — 2D position, 2D velocity, turn rate.

**Model 1 — Constant Velocity (CV)**:
Assumes `x += vx·dt`, `y += vy·dt`, `ω = 0`. Optimised for straight-line walking. Process noise is assigned to both position and velocity.

**Model 2 — Coordinated Turn (CT)**:
Uses the kinematic equations for circular arc motion. When a non-zero turn rate ω is present, the velocity vector rotates through the turn. The state transition Jacobian includes sinc and cosc terms. Process noise on ω allows smooth adaptation to changing turn rate. Handles cornering behaviour.

**Model 3 — Brownian Motion (BM)**:
Assumes near-zero velocity (velocity decays toward 0 at each step, only position noise is present). Handles stationary or slow-milling behaviour without the BM model, a stopped person would cause the CV model to accumulate large velocity error.

**SPENCER Markov Transition Matrix**:
The probability of switching between models from one timestep to the next is encoded in a 3×3 Markov matrix (tuned following the SPENCER crowd navigation project). These probabilities bias the IMM toward realistic human motion transitions (e.g., CV→CT is more likely than CV→BM in a single step).

### Track-to-Detection Association

Before updating any track, each incoming fused detection must be assigned to an existing track (or declared a new track). The association criterion is the Mahalanobis distance between the KF-predicted position (with its uncertainty ellipse) and the measurement:

```
d_mahal = sqrt( (z - H·x̂)ᵀ · (H·P·Hᵀ + R)⁻¹ · (z - H·x̂) )
```

This distance accounts for the track's current uncertainty — a track with high covariance will accept detections from a larger physical radius. Hard gating requires both:
- `d_mahal < 3.0 σ` (Mahalanobis gate), AND
- `physical_distance < 1.0 m` (prevents cross-assignment in dense crowds).

The Hungarian algorithm is again used for globally-optimal assignment across all track-detection pairs.

### Innovation Clamping

A well-known failure mode of Kalman filters in robotics is "filter hijacking": a single outlier measurement (e.g., a spurious LiDAR return co-located with an existing track) causes the filter to snap to the wrong position. To prevent this, the innovation (difference between measurement and prediction) is clamped to a maximum physical displacement of 0.4 m per frame. At 10 Hz, this corresponds to a maximum implied velocity of 4 m/s — beyond the walking speed of any human — so no legitimate measurement is ever clipped. Outlier measurements simply have reduced influence.

### Two-Point Velocity Initialisation

When a new track is created, its velocity is unknown. Rather than initialising velocity to zero (which causes the BM model to dominate early), the track waits for a second detection hit and uses the displacement between the first two confirmed positions divided by elapsed time as the initial velocity estimate. This is capped at 2.0 m/s to prevent noise spikes from producing unphysical initial velocities.

### SPENCER Track Lifecycle

Tracks progress through a state machine modelled after the SPENCER project's crowd tracking system:

- **TENTATIVE**: A new detection cluster that has not yet been confirmed. Not published. The track accumulates hits. Required: 3 hits for standard LiDAR-only tracks, 1 hit for visually confirmed (YOLO-matched) tracks.
- **NEW**: Freshly promoted from TENTATIVE. Published once.
- **MATCHED**: Actively updated with sensor measurements this frame.
- **MISSED**: No measurement matched this frame. Track coasts via KF prediction only.
- **DELETED**: Removed from tracking pool and cleaned up.

The miss tolerance before deletion differs by track age:
- Mature tracks (≥ 20 total hits): survive up to 30 missed frames (~3 seconds at 10 Hz). These represent well-established tracks of people who may have stepped into a doorway or behind a brief occlusion.
- Fresh tracks (< 20 hits): deleted after only 5 missed frames. An unconfirmed track that immediately disappears is likely a false positive.

### Static False Positive Gating

After filtering and after KF update, a confidence suppressor checks each track. If the track's estimated speed (`|v|`) has remained below 0.1 m/s for more than 2.0 continuous seconds, the track confidence is capped at 0.2. This prevents persistent static detections (e.g., a chair that survived Stage 1 masking due to map age, or a newly-placed obstacle) from being reported as humans indefinitely. Tracks with a YOLO visual confirmation history are exempt from this gate — a human standing still is still a human.

---

## Stage 6: Output

### fused_humans_kf/poses (PoseArray)

Published at the Kalman filter update rate for all tracks with confidence ≥ 0.3.

The standard `Pose` message fields are repurposed to carry richer data without introducing custom message types:

| Field | Meaning |
|---|---|
| `position.x` | KF-estimated X position in map frame (metres) |
| `position.y` | KF-estimated Y position in map frame (metres) |
| `position.z` | Track confidence score [0.0–1.0] |
| `orientation.x` | Velocity component vx (m/s) |
| `orientation.y` | Velocity component vy (m/s) |
| `orientation.z` | 0.0 |
| `orientation.w` | Velocity magnitude `|v|`, capped at 5.0 m/s |

### fused_humans_kf/markers (MarkerArray)

Orange cylinders (0.5 m diameter, 1.8 m height) at each estimated position, velocity arrows scaled by speed, and floating text labels showing track ID, speed in m/s, and confidence score. Used for real-time debugging in RViz.

---

## Transform Hierarchy

All sensor detections are expressed in the global `map` frame before fusion, enabling the fusion node to compare positions across sensors without coordinate frame confusion. The adaptive 2-step TF lookup used by all three nodes is described in Stage 2.

```
map (global SLAM frame)
  └── odom (local odometry, continuous)
        ├── base_link (robot body centre)
        │     ├── laser_frame (LiDAR origin)
        │     └── camera_frame (ZED2 left lens origin)
        └── [all detections are expressed here after transformation]
```

---

## ROS2 Node and Topic Summary

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            NODE TOPOLOGY                                         │
│                                                                                 │
│                                                                                 │
│  /scan ──────────────────────────▶ lidar_human_detection                        │
│  /map ───────────────────────────▶        │                                     │
│                                           │ detected_people ──────────────┐    │
│                                           │ detected_people_markers        │    │
│                                           │ /filtered_scan (debug)         │    │
│                                                                             │    │
│  /zed2_left_camera/image_raw ──▶  human_tracker                             │    │
│  /zed2_left_camera/depth/... ──▶        │                                   │    │
│  /zed2_left_camera/camera_info ▶        │ tracked_humans/poses ──────────┐ │    │
│                                          │ tracked_humans/markers         │ │    │
│                                          │ tracked_humans/visualization   │ │    │
│                                                                            │ │    │
│                                                                            ▼ ▼    │
│                                                              human_fusion_kf     │
│                                                                     │            │
│                                                                     │            │
│                                               fused_humans_kf/poses ▼           │
│                                               fused_humans_kf/markers            │
│                                                                                  │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### All Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/scan` | `LaserScan` | Input | Raw 360° LiDAR scan |
| `/map` | `OccupancyGrid` | Input | SLAM occupancy grid (latched) |
| `/zed2_left_camera/image_raw` | `Image` | Input | ZED2 RGB |
| `/zed2_left_camera/depth/image_raw` | `Image` | Input | ZED2 depth |
| `/zed2_left_camera/camera_info` | `CameraInfo` | Input | Camera intrinsics |
| `tracked_humans/poses` | `PoseArray` | Internal | YOLO26n detections (map frame) |
| `tracked_humans/markers` | `MarkerArray` | Internal/Viz | YOLO RViz markers |
| `tracked_humans/visualization` | `Image` | Internal/Debug | Annotated camera image |
| `detected_people` | `PoseArray` | Internal | DR-SPAAM detections (map frame) |
| `detected_people_markers` | `Marker` | Internal/Viz | DR-SPAAM RViz circles |
| `/filtered_scan` | `LaserScan` | Debug | Post-masking scan |
| `fused_humans_kf/poses` | `PoseArray` | Output | KF-fused human tracks |
| `fused_humans_kf/markers` | `MarkerArray` | Output/Viz | Fused tracks RViz visualization |

---

## Launch Parameters Reference

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Set `true` for Gazebo simulation |
| `yolo_model_path` | `yolo26n-seg.engine` | YOLO model (`.engine` for TensorRT, `.pt` for CPU) |
| `yolo_confidence_threshold` | `0.5` | Minimum YOLO detection confidence |
| `rgb_topic` | `/zed2_left_camera/image_raw` | RGB camera topic |
| `depth_topic` | `/zed2_left_camera/depth/image_raw` | Depth topic |
| `camera_info_topic` | `/zed2_left_camera/camera_info` | Camera info topic |
| `max_detection_distance` | `15.0` | Max YOLO depth range (metres) |
| `tracker_type` | `bytetrack.yaml` | YOLO tracker (`bytetrack` or `botsort`) |
| `iou_threshold` | `0.5` | YOLO tracker IOU threshold |
| `drspaam_model_path` | `ckpt_jrdb_ann_ft_dr_spaam_e20.pth` | DR-SPAAM weights |
| `lidar_conf_thresh` | `0.3` | DR-SPAAM confidence threshold |
| `scan_topic` | `/scan` | LiDAR scan topic |
| `detector_model` | `DR-SPAAM` | Detector type (`DR-SPAAM` or `DROW3`) |
| `stride` | `1` | LiDAR scan downsampling stride |
| `panoramic_scan` | `true` | Enable 360° panoramic mode |
| `camera_fov_degrees` | `110.0` | Camera horizontal FOV (degrees) |
| `fusion_distance_threshold` | `1.0` | YOLO↔LiDAR match distance (metres) |
| `track_timeout_sec` | `1.0` | Track coast timeout (seconds) |
| `max_track_distance` | `2.0` | Mahalanobis gating distance (metres) |
| `process_noise_pos` | `0.1` | KF position process noise |
| `process_noise_vel` | `0.5` | KF velocity process noise |
| `measurement_noise` | `0.3` | KF measurement covariance |
| `base_frame` | `base_link` | Robot base frame |
| `map_frame` | `map` | Global reference frame |

---

## Performance Summary

| Component | Hardware | Framerate |
|---|---|---|
| YOLO26n-seg (TensorRT FP16) | Jetson Orin | 20–30 FPS |
| DR-SPAAM (ONNX Runtime + cuDNN) | Jetson Orin | 13–15 FPS |
| DR-SPAAM (PyTorch) | Jetson Orin | 10–12 FPS |
| Kalman Fusion Node | CPU | ~10 Hz (sync rate) |
| End-to-end latency | — | ~50 ms |

---

## Design Rationale Summary

The pipeline is built on the principle that **no single sensor should be trusted unconditionally**. LiDAR provides spatial accuracy and 360° coverage but cannot reliably classify humans in cluttered environments. The camera provides strong human classification evidence but is restricted to a forward FOV and has range-dependent depth error. By applying the camera as a **veto** inside its FOV and as an **uncertainty reducer** through confidence boosting, the fusion achieves a false-positive rate much lower than either sensor alone, while the Kalman filter converts the noisy frame-by-frame detections into smooth, motion-model-constrained trajectories suitable for real-time crowd navigation.
