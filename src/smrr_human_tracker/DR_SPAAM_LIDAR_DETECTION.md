# DR-SPAAM 2D LiDAR Human Detection Implementation

## Overview

This document provides a comprehensive explanation of the DR-SPAAM based 2D LiDAR human detection system implemented in the `smrr_human_tracker` package. The system uses deep learning to detect pedestrians from 2D laser scans in real-time, with advanced frame transformation and performance optimizations.

## Architecture

### System Components

```
┌────────────────┐
│  /scan Topic   │ (LaserScan)
└────────┬───────┘
         │
         ▼
┌─────────────────────────────────────┐
│   DrSpaamNode                       │
│   (human_lidar_matcher.py)          │
│                                     │
│  ┌───────────────────────────────┐ │
│  │  DR-SPAAM Detector            │ │
│  │  - Deep Learning Model        │ │
│  │  - GPU Acceleration           │ │
│  └───────────────────────────────┘ │
│                                     │
│  ┌───────────────────────────────┐ │
│  │  TF2 Transformation           │ │
│  │  - Zero-Wait Strategy         │ │
│  │  - Ego-Motion Compensation    │ │
│  └───────────────────────────────┘ │
└─────────┬───────────────────────────┘
          │
          ├─────────────┬───────────────┐
          ▼             ▼               ▼
  ┌───────────────┐  ┌────────────┐  ┌─────────────┐
  │ PoseArray     │  │  Markers   │  │ Performance │
  │ (Detections)  │  │  (RViz)    │  │   Metrics   │
  └───────────────┘  └────────────┘  └─────────────┘
```

### File Structure

- **Node Implementation**: `src/smrr_human_tracker/smrr_human_tracker/human_lidar_matcher.py`
- **Launch File**: `src/smrr_human_tracker/launch/lidar_human_detector.launch.py`
- **Model Weights**: `models/ckpt_jrdb_ann_ft_dr_spaam_e20.pth`

## DR-SPAAM Detector

### What is DR-SPAAM?

**DR-SPAAM** (Deep Rotation-equivariant Self-Attention for Point cloud Analysis and Multiple pedestrian detection) is a deep learning-based 2D LiDAR person detector that:

- Uses rotation-equivariant self-attention mechanisms
- Processes raw laser scan data directly (no feature engineering)
- Operates in real-time with GPU acceleration
- Works with panoramic (360°) laser scans
- Trained on JRDB (JackRabbot Dataset and Benchmark) for robust pedestrian detection

### Supported Models

The implementation supports two detector architectures:

1. **DR-SPAAM** (Default) - State-of-the-art, rotation-equivariant
2. **DROW3** - Earlier architecture, still effective

## Node Implementation Details

### DrSpaamNode Class

```python
class DrSpaamNode(Node):
    """ROS 2 node to detect pedestrians using DROW3 or DR-SPAAM."""
```

### Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `weight_file` | string | `ckpt_jrdb_ann_ft_dr_spaam_e20.pth` | Pre-trained model checkpoint path |
| `detector_model` | string | `DR-SPAAM` | Model architecture (DR-SPAAM/DROW3) |
| `conf_thresh` | float | `0.3` | Detection confidence threshold (0.0-1.0) |
| `stride` | int | `1` | Scan downsampling stride (1=no downsampling) |
| `panoramic_scan` | bool | `True` | Enable for 360° scans |
| `scan_topic` | string | `/scan` | Input laser scan topic |
| `target_frame` | string | `map` | Output frame (map/odom/base_link) |
| `detections_topic` | string | `detected_people` | Output PoseArray topic |
| `marker_topic` | string | `detected_people_markers` | Output RViz marker topic |

### Detection Pipeline

#### 1. Scan Preprocessing

```python
scan = np.array(msg.ranges)
scan[scan == 0.0] = 29.99      # Replace zeros with max range
scan[np.isinf(scan)] = 29.99   # Handle infinite values
scan[np.isnan(scan)] = 29.99   # Handle NaN values
```

**Why?** The detector expects valid range values. Invalid readings are set to a large value (29.99m) to be filtered out.

#### 2. Detection Inference

```python
dets_xy, dets_cls, _ = self._detector(scan)
```

**Outputs:**
- `dets_xy`: Detection positions in polar coordinates (x, y relative to laser frame)
- `dets_cls`: Confidence scores (0.0-1.0)

#### 3. Confidence Filtering

```python
conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
dets_xy = dets_xy[conf_mask]
dets_cls = dets_cls[conf_mask]
```

Only detections above the confidence threshold are kept.

#### 4. Message Conversion

Detections are converted to:
- **PoseArray**: For fusion with other detection sources
- **Marker**: For visualization in RViz

#### 5. Frame Transformation

Detections are transformed from the laser frame to the target frame (typically `map`).

## Frame Transformation Strategy

### The Zero-Wait Ego-Motion Fix

This is a **critical optimization** that prevents blocking while maintaining accuracy.

#### Problem

Traditional TF lookups with timeouts block the node, reducing FPS:
```python
# DON'T DO THIS - Blocks for up to 1 second!
transform = self.tf_buffer.lookup_transform(
    target_frame, source_frame, timestamp,
    timeout=rclpy.duration.Duration(seconds=1.0)  # ❌ BLOCKS!
)
```

#### Solution: Two-Stage Instant Lookup

```python
try:
    # Stage 1: Try exact timestamp (instant, non-blocking)
    transform = self.tf_buffer.lookup_transform(
        self.target_frame,
        pose_array.header.frame_id,
        rclpy.time.Time.from_msg(pose_array.header.stamp),
        timeout=rclpy.duration.Duration(seconds=0.0)  # ✅ INSTANT!
    )
except TransformException:
    # Stage 2: Fallback to latest available (instant, non-blocking)
    transform = self.tf_buffer.lookup_transform(
        self.target_frame,
        pose_array.header.frame_id,
        rclpy.time.Time(),  # Latest available
        timeout=rclpy.duration.Duration(seconds=0.0)  # ✅ INSTANT!
    )
```

**Benefits:**
- ✅ **Zero blocking** - Maintains high FPS (>30 Hz)
- ✅ **Ego-motion compensation** - Uses latest transform if exact time unavailable
- ✅ **Graceful degradation** - Falls back to latest instead of failing

### Confidence Injection Strategy

**Critical Detail**: Confidence scores are injected into `position.z` **AFTER** frame transformation:

```python
# 1. Create PoseArray with identity quaternions
dets_msg = self._detections_to_pose_array(dets_xy, dets_cls)

# 2. Transform to target frame
if self.target_frame and self.target_frame != msg.header.frame_id:
    dets_msg = self._transform_pose_array_to_target_frame(dets_msg)

# 3. THEN inject confidence (AFTER transformation)
for pose, d_cls in zip(dets_msg.poses, dets_cls):
    pose.position.z = float(d_cls)  # Confidence stored here
```

**Why?** Injecting confidence before transformation would corrupt the quaternion during TF operations. By using clean identity quaternions during transformation and injecting confidence afterward, we maintain numerical stability.

### Clean Quaternion Initialization

```python
def _detections_to_pose_array(self, dets_xy, dets_cls):
    """Convert detections to PoseArray with pure identity quaternions"""
    pose_array = PoseArray()
    for d_xy in dets_xy:
        p = Pose()
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0  # Clean identity quaternion!
        p.orientation.w = 1.0
        pose_array.poses.append(p)
    return pose_array
```

## Performance Optimizations

### 1. Lazy Publisher Check

```python
if (self._dets_pub.get_subscription_count() == 0 and 
    self._rviz_pub.get_subscription_count() == 0):
    return  # Skip processing if no subscribers
```

Avoids unnecessary computation when no node is listening.

### 2. GPU Acceleration

```python
self._detector = Detector(
    weight_file,
    model=detector_model,
    gpu=True,  # GPU acceleration enabled
    stride=stride,
    panoramic_scan=panoramic_scan,
)
```

Utilizes GPU for neural network inference.

### 3. Performance Metrics Tracking

```python
# Rolling window of last 30 frame times
self.frame_times = deque(maxlen=30)

# Log metrics every 5 seconds
if current_time - self.last_metrics_log >= 5.0:
    avg_time = np.mean(self.frame_times)
    fps = 1.0 / avg_time if avg_time > 0 else 0
    self.get_logger().info(
        f'[DR-SPAAM Performance] FPS: {fps:.2f} | Avg: {avg_time*1000:.1f}ms'
    )
```

**Output Example:**
```
[INFO] [dr_spaam_node]: [DR-SPAAM Performance] FPS: 32.45 | Avg: 30.8ms
```

### 4. Dynamic FOV Configuration

```python
if not self._detector.is_ready():
    fov_deg = np.rad2deg(msg.angle_increment * len(msg.ranges))
    self._detector.set_laser_fov(fov_deg)
```

Automatically configures the detector based on the actual laser FOV from the first scan message.

## RViz Visualization

### Marker Generation

Detections are visualized as **red circles** around detected people:

```python
def _detections_to_rviz_marker(self, dets_xy, dets_cls):
    """Convert detections to RViz marker circles"""
    msg = Marker()
    msg.type = Marker.LINE_LIST  # Circle approximated by line segments
    msg.color.r = 1.0  # Red
    msg.color.a = 1.0  # Fully opaque
    msg.scale.x = 0.05  # Line width
    
    # Circle parameters
    r = 0.4  # radius (meters)
    ang = np.linspace(0, 2 * np.pi, 20)  # 20 segments per circle
```

Each detection is represented as a 0.4m radius circle with 20 line segments.

## Launch Configuration

### Launch File: `lidar_human_detector.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('smrr_human_tracker')
    default_weight_file = os.path.join(
        package_share_dir, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth'
    )
    
    detector_node = Node(
        package='smrr_human_tracker',
        executable='human_lidar_matcher',
        name='dr_spaam_detector',
        output='screen',
        parameters=[{...}]
    )
```

### Usage Examples

#### Basic Launch
```bash
ros2 launch smrr_human_tracker lidar_human_detector.launch.py
```

#### Custom Confidence Threshold
```bash
ros2 launch smrr_human_tracker lidar_human_detector.launch.py conf_thresh:=0.5
```

#### Change Detection Frame
```bash
ros2 launch smrr_human_tracker lidar_human_detector.launch.py target_frame:=odom
```

#### Custom Scan Topic
```bash
ros2 launch smrr_human_tracker lidar_human_detector.launch.py scan_topic:=/front_scan
```

#### Use DROW3 Model
```bash
ros2 launch smrr_human_tracker lidar_human_detector.launch.py \
    detector_model:=DROW3 \
    weight_file:=/path/to/drow3_model.pth
```

## Output Topics

### 1. Detections Topic (PoseArray)

**Topic**: `detected_people` (default)  
**Type**: `geometry_msgs/msg/PoseArray`

**Structure**:
```yaml
header:
  stamp: <timestamp>
  frame_id: "map"  # or configured target_frame
poses:
  - position:
      x: 2.5    # Detection X position
      y: 1.3    # Detection Y position
      z: 0.85   # ⚠️ Confidence score (0.0-1.0)
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

**Important**: The confidence score is stored in `position.z`, not in orientation!

### 2. Marker Topic (Visualization)

**Topic**: `detected_people_markers` (default)  
**Type**: `visualization_msgs/msg/Marker`

Red circles displayed in RViz at detection positions.

## Technical Details

### Scan Processing Specs

- **Input**: 360° 2D laser scan
- **Range Handling**: 0-30m (invalid values set to 29.99m)
- **Stride**: Configurable downsampling (default: 1 = no downsampling)
- **FOV**: Auto-detected from first scan message

### Model Information

- **Architecture**: DR-SPAAM (rotation-equivariant self-attention)
- **Training Dataset**: JRDB (JackRabbot Dataset and Benchmark)
- **Model File**: `ckpt_jrdb_ann_ft_dr_spaam_e20.pth` (20 epochs fine-tuned)
- **Inference**: GPU-accelerated (CUDA required)

### Performance Characteristics

- **FPS**: 30-40 Hz (with GPU, depends on scan density)
- **Latency**: ~30ms per frame
- **Detection Range**: 0-30m (practical: 0-15m for reliable detection)
- **Confidence Threshold**: 0.3 (default, adjustable)

## Integration with Human Tracker

The LiDAR detections integrate with the multi-modal human tracking system:

```
DR-SPAAM Detections (LiDAR) ──┐
                              ├──> Human Fusion Node ──> Kalman Filter
YOLO Detections (Camera) ─────┘
```

- **LiDAR Detections**: Provide accurate 2D positions immune to lighting
- **Camera Detections**: Provide visual classification and depth
- **Fusion**: Combines both modalities for robust tracking
- **Kalman Filter**: Tracks state (position, velocity) over time

## Debugging and Monitoring

### Enable Debug Logging

```bash
ros2 run smrr_human_tracker human_lidar_matcher --ros-args --log-level debug
```

### Monitor Performance

```bash
# Watch detection rate
ros2 topic hz /detected_people

# View detection messages
ros2 topic echo /detected_people

# Check node info
ros2 node info /dr_spaam_detector
```

### RViz Visualization

Add the following displays in RViz:
1. **LaserScan**: Raw laser data (`/scan`)
2. **Marker**: Detection circles (`/detected_people_markers`)
3. **PoseArray**: Detection poses (optional, for debugging)

## Common Issues and Solutions

### Issue: Low FPS

**Symptoms**: FPS < 20 Hz  
**Solutions**:
- Ensure GPU is available and CUDA drivers installed
- Increase `stride` parameter to downsample scan
- Check TF tree is properly configured (zero-wait transform ensures this isn't the bottleneck)

### Issue: No Detections

**Symptoms**: Empty PoseArray messages  
**Solutions**:
- Lower `conf_thresh` parameter (try 0.2)
- Verify laser scan has valid data (`ros2 topic echo /scan`)
- Check people are within 0-15m range
- Ensure model file loaded correctly (check logs)

### Issue: Transform Errors

**Symptoms**: Warnings about failed transforms  
**Solutions**:
- Verify TF tree: `ros2 run tf2_tools view_frames`
- Check `target_frame` exists and is connected to laser frame
- Ensure timestamp synchronization is working

### Issue: Detections in Wrong Frame

**Symptoms**: Markers appear at incorrect locations  
**Solutions**:
- Verify `target_frame` parameter is correct
- Check TF tree connectivity
- Confirm laser frame_id matches TF tree

## Dependencies

### ROS 2 Packages
- `rclpy` - ROS 2 Python client
- `sensor_msgs` - LaserScan messages
- `geometry_msgs` - Pose messages
- `visualization_msgs` - RViz markers
- `tf2_ros` - Frame transformations
- `tf2_geometry_msgs` - Transform utilities

### Python Packages
- `numpy` - Numerical operations
- `dr_spaam` - DR-SPAAM detector library

### Hardware Requirements
- **GPU**: NVIDIA GPU with CUDA support (required for real-time performance)
- **VRAM**: 1GB+ recommended
- **CPU**: Multi-core processor (fallback if no GPU)

## References

1. **DR-SPAAM Paper**: [arXiv:2004.14979](https://arxiv.org/abs/2004.14979)
2. **JRDB Dataset**: [jrdb.erc.monash.edu](https://jrdb.erc.monash.edu/)
3. **DR-SPAAM GitHub**: [github.com/VisualComputingInstitute/DR-SPAAM-Detector](https://github.com/VisualComputingInstitute/DR-SPAAM-Detector)

## License

This implementation is part of the `smrr_human_tracker` package under Apache-2.0 license.

---

**Last Updated**: February 19, 2026  
**Package Version**: 0.0.0  
**Maintainer**: achiraubuntu (achirahansindu52@gmail.com)
