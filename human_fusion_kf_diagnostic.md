# Human Tracking Pipeline — Post-Fix Verification & Diagnostic Audit (v2)

> **Audit Date:** 2026-02-14  
> **Audited Files:** `human_tracker.py`, `human_lidar_matcher.py`, `human_fusion_kf_node.py`, `human_fusion_kf.launch.py`  
> **Original Complaint:** Severe velocity estimation spikes corrupting RL observation space  
> **Audit Purpose:** Verify 8 applied fixes; identify remaining failure modes

---

## 1. Verification of Phase 1: Core Math Fixes

### 1.1 Double-Predict Bug & $\Delta t$ Guard — **[VERIFIED]** ✅

**Double-predict removed.** `HumanTrackKF.update()` no longer calls `self.kf.predict()`:

```python
# human_fusion_kf_node.py — HumanTrackKF.update()
def update(self, measurement, timestamp, confidence=None):
    # Measurement update only — no prediction step
    self.kf.update(measurement, confidence)
    self.last_seen = timestamp
    self.kf.last_update = timestamp
```

**$\Delta t$ guard present.** `HumanTrackKF.predict()` skips prediction when `dt < 1e-4`:

```python
# human_fusion_kf_node.py — HumanTrackKF.predict()
if dt < 1e-4:
    return
```

The `kf.predict(dt)` and `self.kf.last_update = current_time` only execute when the guard passes.

---

### 1.2 Dynamic $Q$ Matrix — **[VERIFIED]** ✅

`HumanTrackKF.predict()` dynamically computes $Q$ using the discrete white-noise acceleration model before every prediction:

```python
# human_fusion_kf_node.py — HumanTrackKF.predict()
noise_accel = 0.5
q_pos = (dt**3) / 3.0 * noise_accel
q_vel = dt * noise_accel
q_cov = (dt**2) / 2.0 * noise_accel
self.kf.Q = np.array([
    [q_pos, 0.0,   q_cov, 0.0  ],
    [0.0,   q_pos, 0.0,   q_cov],
    [q_cov, 0.0,   q_vel, 0.0  ],
    [0.0,   q_cov, 0.0,   q_vel]
])
```

This correctly encodes:

$$
Q = \sigma_a^2 \begin{bmatrix}
\frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} & 0 \\
0 & \frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} \\
\frac{\Delta t^2}{2} & 0 & \Delta t & 0 \\
0 & \frac{\Delta t^2}{2} & 0 & \Delta t
\end{bmatrix}, \quad \sigma_a^2 = 0.5 \; \text{m}^2/\text{s}^4
$$

---

### 1.3 Confidence Score Corruption — **[VERIFIED]** ✅

**YOLO Tracker (`human_tracker.py`):** Identity quaternion set before TF transform; confidence packed into `position.z` *after* transform:

```python
pose_camera.orientation.x = 0.0
pose_camera.orientation.y = 0.0
pose_camera.orientation.z = 0.0
pose_camera.orientation.w = 1.0
# ...after transform...
pose_map.position.z = float(human['confidence'])
```

**LiDAR Matcher (`human_lidar_matcher.py`):** Identity quaternion in `_detections_to_pose_array`; confidence packed *after* frame transform:

```python
p.orientation.x = 0.0
p.orientation.y = 0.0
p.orientation.z = 0.0
p.orientation.w = 1.0
# ...after _transform_pose_array_to_target_frame()...
for pose, d_cls in zip(dets_msg.poses, dets_cls):
    pose.position.z = float(d_cls)
```

**Fusion Node (`human_fusion_kf_node.py`):** Extracts confidence from `position.z`:

```python
yolo_conf = np.array([
    p.position.z if (p.position.z > 0.0) else 0.7
    for p in yolo_msg.poses
])
```

Output also uses `position.z` with identity quaternion:

```python
pose.position.z = track.confidence
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
pose.orientation.w = min(vel_mag, 5.0)
```

> **⚠️ Minor Note:** The output `orientation.w` is overloaded with velocity magnitude (clamped to 5.0), not `1.0`. This is not a unit quaternion, but since the fused output is not re-transformed, it does not cause corruption. Downstream consumers must be aware of this encoding.

---

## 2. Verification of Phase 2: Pipeline Architecture

### 2.1 Ego-Motion Latency — **[VERIFIED]** ✅

**YOLO Tracker (`human_tracker.py → transform_pose_to_map`):** Uses the sensor message timestamp:

```python
pose_stamped.header.stamp = header.stamp  # ← sensor capture time
transformed_pose = self.tf_buffer.transform(
    pose_stamped, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.5)
)
```

**LiDAR Matcher (`human_lidar_matcher.py → _transform_pose_array_to_target_frame`):** Converts to `rclpy.time.Time`:

```python
transform = self.tf_buffer.lookup_transform(
    self.target_frame, pose_array.header.frame_id,
    rclpy.time.Time.from_msg(pose_array.header.stamp),  # ← sensor capture time
    timeout=rclpy.duration.Duration(seconds=0.1)
)
```

**LiDAR Matcher (`_transform_marker_to_target_frame`):** Also uses message timestamp:

```python
rclpy.time.Time.from_msg(marker.header.stamp)
```

> **Residual:** The fusion node's `get_robot_pose_and_yaw()` still uses `rclpy.time.Time()` (latest) for the FOV-check transform. This is acceptable — the FOV check is a coarse geometric filter, not a precision transform. The error is bounded by robot angular velocity × latency and does not feed the KF directly.

---

### 2.2 ApproximateTimeSynchronizer — **[VERIFIED]** ✅

**Timer removed.** No `create_timer` in `HumanFusionKFNode.__init__()`.  
**Caches removed.** No `self.latest_yolo` or `self.latest_lidar`.  
**Callbacks removed.** No `yolo_callback` or `lidar_callback` stubs.

**Synchronizer implemented:**

```python
self.yolo_sub = Subscriber(self, PoseArray, 'tracked_humans/poses')
self.lidar_sub = Subscriber(self, PoseArray, 'detected_people')
self.ts = ApproximateTimeSynchronizer(
    [self.yolo_sub, self.lidar_sub],
    queue_size=10,
    slop=0.1  # 100 ms tolerance
)
self.ts.registerCallback(self.fusion_callback)
```

**Callback signature updated:**

```python
def fusion_callback(self, yolo_msg, lidar_msg):
```

**Timestamp anchored to sensor time:**

```python
current_time = rclpy.time.Time.from_msg(yolo_msg.header.stamp)
```

---

### 2.3 Hungarian Algorithm for YOLO↔LiDAR — **[VERIFIED]** ✅

**Cost matrix built via vectorized Euclidean distance:**

```python
cost_matrix = np.linalg.norm(
    yolo_points[:, np.newaxis, :] - lidar_points[np.newaxis, :, :], axis=2
)
```

**Optimal assignment via `linear_sum_assignment`:**

```python
from scipy.optimize import linear_sum_assignment
row_ind, col_ind = linear_sum_assignment(cost_matrix)
```

**Distance threshold gate applied:**

```python
for r, c in zip(row_ind, col_ind):
    if cost_matrix[r, c] < self.fusion_threshold:  # default 1.0 m
        # Confidence-weighted fusion
```

Unmatched YOLO → kept as camera-only.  
Unmatched LiDAR → kept only if outside camera FOV.

---

## 3. Verification of Phase 3: Final Polish

### 3.1 Dynamic Depth Window — **[VERIFIED]** ✅

In `human_tracker.py → calculate_3d_position`:

```python
box_width = x2 - x1
box_height = y2 - y1
region_size = max(1, int(min(box_width, box_height) * 0.1))
```

Scales the sampling window to 10% of the smaller bounding-box dimension, minimum 1 pixel. A 200×400 bbox → ±20 px; a 30×60 bbox → ±3 px.

---

### 3.2 `use_sim_time` in Launch File — **[MISSING]** ❌

The launch file `human_fusion_kf.launch.py` does **not** declare or pass `use_sim_time` to any node. All three nodes will use wall-clock time when running in Gazebo, while TF operates on sim time. This causes $\Delta t$ mismatches in the KF predict step and potential TF extrapolation errors.

**Evidence:** No `use_sim_time` string appears anywhere in the launch file. No global parameter block sets it.

---

## 4. Secondary Failure Analysis — Remaining Velocity Noise Sources

### 4.1 Track Initialization — **[CONCERN]** 🟡

When a new `HumanTrackKF` is created:

| Parameter | Value | Source |
|---|---|---|
| Initial velocity $(v_x, v_y)$ | $(0.0, 0.0)$ | `KalmanFilter.__init__`: `self.x = [..., 0.0, 0.0]` |
| Initial $P_{vx}, P_{vy}$ | $1.0 \; \text{m}^2/\text{s}^2$ | `KalmanFilter.__init__`: `np.diag([..., 1.0, 1.0])` |

**Problem:** When a pedestrian walks into frame at 1.4 m/s, the filter is initialized at $v = 0$ with $P_v = 1.0$. The KF needs several measurement cycles to converge on the true velocity. During convergence the velocity estimate ramps from 0 → 1.4 m/s, appearing as an acceleration that doesn't exist. If a downstream RL policy reads velocity during this transient, it sees a "velocity spike from zero."

**Recommendation:** Increase $P_v^0$ to $4.0$ or higher (representing ±2 m/s uncertainty) so the filter converges in fewer steps, or delay publishing velocity until the track is at least 3–5 updates old.

---

### 4.2 Track Lifecycle & Ghost Coasting — **[CONCERN]** 🟡

```python
# Track timeout: default 1.0 s
self.track_timeout = self.get_parameter('track_timeout_sec').value  # 1.0
```

**Coasting behavior:** When a track loses association, it continues to be predicted forward at its last estimated velocity with no damping. Over 1.0 second at 1.4 m/s, the predicted position drifts ~1.4 m.

**Snap-back risk:** If the human reappears (e.g., after brief occlusion) and is re-associated to the drifted coast position, the KF update yanks the position back. This produces:
- A large innovation $\mathbf{y}$, generating a velocity correction spike.
- Several frames of oscillation before convergence.

**No velocity damping during coast:** The CV model maintains velocity indefinitely. A decaying velocity model or reducing $P_v$ ceiling during coast would help.

---

### 4.3 Static Measurement Noise $R$ — **[CONCERN]** 🟡

```python
# KalmanFilter.__init__
self.R = np.eye(2) * measurement_noise  # 0.3 * I
```

$R$ is fixed at $0.3 \; \text{m}^2$ for all measurements regardless of:

| Factor | Impact on True Noise |
|---|---|
| **Sensor source** | ZED2i stereo depth: ~0.01–0.5 m error depending on range; RPLiDAR: ~0.03 m |
| **Detection confidence** | A 0.5-confidence detection is noisier than a 0.95-confidence detection |
| **Range** | Depth error scales quadratically with distance for stereo cameras |

**Effect:** At close range the filter over-trusts (too much noise assumed), at long range it under-trusts (too little noise assumed). For low-confidence detections, the filter weighs them equally with high-confidence ones — injecting more noise into the velocity estimate.

**Recommendation:** Scale $R$ dynamically, e.g.:

```python
R_scale = max(0.1, 1.0 - confidence)  # Lower confidence → higher R
self.R = np.eye(2) * measurement_noise * (1.0 + R_scale * range_m)
```

---

### 4.4 No Innovation Gating — **[CONCERN]** 🟡

The KF update accepts **any** measurement that passes the track-to-detection Hungarian assignment ($< 2.0$ m). There is no Mahalanobis distance / chi-squared innovation gate. A single GPS-like outlier measurement (e.g., a stereo depth glitch at 15 m instead of 3 m) will directly corrupt the state because:

$$
\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + K \cdot (\mathbf{z}_{\text{outlier}} - H\hat{\mathbf{x}}^-)
$$

The innovation $\mathbf{y}$ can be enormous, and the full Kalman gain $K$ is applied without clamping.

**Recommendation:** Add a chi-squared gate before `kf.update()`:

```python
S = H @ P @ H.T + R
y = z - H @ x
mahal_sq = y.T @ np.linalg.inv(S) @ y
if mahal_sq > chi2_threshold:  # e.g. 9.21 for 2-DOF at 99%
    return  # Reject outlier
```

---

### 4.5 `noise_accel` Is Hardcoded — **[MINOR]** 🟢

The acceleration variance $\sigma_a^2 = 0.5$ used in the dynamic $Q$ matrix is hardcoded inside `HumanTrackKF.predict()`, not exposed as a ROS parameter. The `process_noise_pos` and `process_noise_vel` launch parameters (still declared and passed) are used only for the initial diagonal $Q$ in `KalmanFilter.__init__()`, which is immediately overwritten on the first `predict()` call. These two parameters are now effectively dead code.

**Recommendation:** Either remove the dead parameters or replace `noise_accel = 0.5` with a configurable ROS parameter.

---

### 4.6 Velocity Output Encoding — **[MINOR]** 🟢

The published `fused_humans_kf/poses` encodes velocity magnitude in `orientation.w`:

```python
pose.orientation.w = min(vel_mag, 5.0)
```

This clamps at 5.0 m/s. Downstream consumers cannot distinguish a human at 5 m/s from one at 10 m/s, and `orientation.w` is not `1.0`, so the quaternion is invalid. Any future code that transforms this pose will produce incorrect results.

---

## 5. Complete Fix Verification Summary

| # | Fix | Status | Evidence |
|---|---|---|---|
| 1 | Double-predict bug removed | ✅ **VERIFIED** | `update()` has no `kf.predict()` call |
| 2 | $\Delta t < 10^{-4}$ guard | ✅ **VERIFIED** | `if dt < 1e-4: return` in `predict()` |
| 3 | Dynamic $Q$ (white-noise accel.) | ✅ **VERIFIED** | $Q(\Delta t)$ computed before every `kf.predict(dt)` |
| 4 | Confidence in `position.z`, identity quat | ✅ **VERIFIED** | All 3 nodes use `position.z`; quat `(0,0,0,1)` |
| 5 | Ego-motion latency (sensor timestamps) | ✅ **VERIFIED** | `header.stamp` / `Time.from_msg()` in both TF lookups |
| 6 | `ApproximateTimeSynchronizer` | ✅ **VERIFIED** | `slop=0.1`, timer removed, caches removed |
| 7 | Hungarian YOLO↔LiDAR association | ✅ **VERIFIED** | `linear_sum_assignment(cost_matrix)` with threshold gate |
| 8 | Dynamic depth window | ✅ **VERIFIED** | `region_size = max(1, int(min(w,h)*0.1))` |
| 9 | `use_sim_time` in launch file | ❌ **MISSING** | Not declared or passed to any node |

---

## 6. Remaining Velocity-Noise Risk Matrix

| # | Severity | Issue | Expected Impact |
|---|---|---|---|
| 1 | 🟡 Medium | Track init at $v=0$ with $P_v=1.0$ | Velocity ramp-up transient on new tracks (~3–5 frames) |
| 2 | 🟡 Medium | 1.0 s coast with no velocity damping | Position drift → snap-back velocity spike on re-detection |
| 3 | 🟡 Medium | Static $R=0.3 I$ regardless of confidence/range | Low-confidence detections inject excessive noise |
| 4 | 🟡 Medium | No innovation gating (Mahalanobis) | Single depth outlier directly corrupts velocity |
| 5 | 🟢 Minor | `noise_accel` hardcoded; launch params dead code | Not tunable at launch; confusing parameter interface |
| 6 | 🟢 Minor | Velocity clamped at 5.0 in `orientation.w` | Downstream saturation; invalid quaternion |
| 7 | 🟢 Minor | `use_sim_time` missing in launch | $\Delta t$ wrong in simulation |

---

## 7. Recommended Next Steps (Priority Order)

1. **Add innovation gating** — reject outlier measurements via Mahalanobis $\chi^2$ test before `kf.update()`.
2. **Make $R$ confidence-adaptive** — scale measurement noise inversely with detection confidence.
3. **Increase initial $P_v$** — set to `4.0` or higher so velocity converges faster for new tracks.
4. **Add velocity damping during coast** — decay velocity toward zero when no measurements arrive for > 0.3 s.
5. **Expose `noise_accel`** as a ROS parameter; remove dead `process_noise_pos` / `process_noise_vel` params.
6. **Add `use_sim_time`** to the launch file.
7. **Consider a track maturity gate** — only publish velocity after a track has received ≥ 3 updates.
