#!/usr/bin/env python3
"""
Door Classifier Node

A ROS 2 node that continuously runs ONNX inference on depth images
to classify elevator door state as OPEN or CLOSED.

Usage:
  # Build:
  colcon build --symlink-install --packages-select smrr_navigation
  source install/setup.bash

  # Run with defaults:
  ros2 run smrr_navigation door_classifier_node.py

  # Run with parameter overrides:
  ros2 run smrr_navigation door_classifier_node.py --ros-args \
      -p threshold:=0.6 \
      -p print_rate_hz:=10.0 \
      -p depth_topic:=/my_depth_topic

  # Using launch file:
  ros2 launch smrr_navigation door_classifier.launch.py

Parameters:
  - depth_topic (str): Depth image topic [default: /zed2_left_camera/depth/image_raw]
  - model_path (str): ONNX model path [default: packaged model]
  - open_index (int): Class index for OPEN [default: 1]
  - threshold (float): Confidence threshold [default: 0.5]
  - clip_min_m (float): Min depth clip in meters [default: 0.0]
  - clip_max_m (float): Max depth clip in meters [default: 5.0]
  - print_rate_hz (float): Print rate in Hz [default: 5.0]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pathlib import Path
import numpy as np
import time

# Import canonical preprocessing spec (matches training exactly)
from smrr_navigation.depth_preprocess_spec import (
    preprocess_depth_training_spec,
    compute_preprocessing_stats
)

# Try to import cv2 for resize, fallback to numpy
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Import onnxruntime
try:
    import onnxruntime as ort
except ImportError:
    ort = None


class DoorClassifierNode(Node):
    """
    ROS 2 node for real-time door state classification using ONNX model.
    """

    def __init__(self):
        super().__init__('door_classifier_node')

        # Declare parameters
        self.declare_parameter('depth_topic', '/zed2_left_camera/depth/image_raw')
        self.declare_parameter('model_path', '')
        self.declare_parameter('open_index', 1)  # Model: 0=CLOSED, 1=OPEN
        self.declare_parameter('threshold', 0.7)
        self.declare_parameter('clip_min_m', 0.2)  # TRAINING SPEC: min clip = 0.2m
        self.declare_parameter('clip_max_m', 5.0)   # TRAINING SPEC: max clip = 5.0m
        self.declare_parameter('print_rate_hz', 5.0)
        self.declare_parameter('debug_preprocess', False)  # Enable preprocessing debug logs

        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        model_path_param = self.get_parameter('model_path').value
        self.open_index = self.get_parameter('open_index').value
        self.threshold = self.get_parameter('threshold').value
        self.clip_min_m = self.get_parameter('clip_min_m').value
        self.clip_max_m = self.get_parameter('clip_max_m').value
        self.print_rate_hz = self.get_parameter('print_rate_hz').value
        self.debug_preprocess = self.get_parameter('debug_preprocess').value
        self.debug_preprocess = self.get_parameter('debug_preprocess').value

        # Validate onnxruntime is available
        if ort is None:
            self.get_logger().fatal('onnxruntime not installed! Run: pip install onnxruntime')
            raise RuntimeError('onnxruntime not available')

        # Resolve model path
        if model_path_param and model_path_param.strip():
            self.model_path = Path(model_path_param)
        else:
            # Default: look in several possible locations
            self.model_path = self._find_default_model_path()

        if not self.model_path.exists():
            self.get_logger().fatal(f'Model file not found: {self.model_path}')
            raise FileNotFoundError(f'Model not found: {self.model_path}')

        # Load ONNX model
        self.get_logger().info(f'Loading ONNX model: {self.model_path}')
        try:
            self.session = ort.InferenceSession(
                str(self.model_path),
                providers=['CPUExecutionProvider']
            )
            self.get_logger().info(f'Model loaded. Providers: {self.session.get_providers()}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to load ONNX model: {e}')
            raise

        # Verify input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.get_logger().info(f'Input: {self.input_name}, Output: {self.output_name}')

        # State tracking
        self.last_prediction = None
        self.last_print_time = 0.0
        self.print_period = 1.0 / max(self.print_rate_hz, 0.1)
        self.frames_received = 0
        self.last_frame_time = None
        self.last_debug_log_time = 0.0  # For throttled preprocessing debug logs

        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('Door Classifier Node Configuration')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Depth topic:    {self.depth_topic}')
        self.get_logger().info(f'  Model path:     {self.model_path}')
        self.get_logger().info(f'  Open index:     {self.open_index}')
        self.get_logger().info(f'  Threshold:      {self.threshold}')
        self.get_logger().info(f'  Clip range:     [{self.clip_min_m}, {self.clip_max_m}] m (TRAINING SPEC)')
        self.get_logger().info(f'  Print rate:     {self.print_rate_hz} Hz')
        self.get_logger().info(f'  Debug preproc:  {self.debug_preprocess}')
        self.get_logger().info(f'  Using cv2:      {HAS_CV2}')
        self.get_logger().info('=' * 60)

        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # Timer to warn if no frames received
        self.watchdog_timer = self.create_timer(5.0, self.watchdog_callback)

        self.get_logger().info(f'Subscribed to: {self.depth_topic}')
        self.get_logger().info('Waiting for depth images...')

    def _find_default_model_path(self) -> Path:
        """
        Find the default model path by checking several possible locations.
        
        Search order:
        1. Relative to the actual Python source file (resolving symlinks)
        2. Relative to __file__ directly
        3. In package share directory
        """
        model_name = 'door_classifier_3.onnx'
        
        # Candidate paths to try
        candidates = []
        
        # 1. Resolve symlinks and look relative to actual source (package root)
        try:
            real_path = Path(__file__).resolve()
            # Go up from smrr_navigation/door_classifier_node.py to package root
            candidates.append(real_path.parent.parent / 'models' / model_name)
        except Exception:
            pass
        
        # 2. Relative to __file__ (may be symlink) - package root
        candidates.append(Path(__file__).parent.parent / 'models' / model_name)
        
        # 3. Try using ament_index to find package share
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('smrr_navigation')
            candidates.append(Path(share_dir) / 'models' / model_name)
        except Exception:
            pass
        
        # Find first existing candidate
        for candidate in candidates:
            if candidate.exists():
                return candidate
        
        # Return best guess (first candidate) even if doesn't exist
        # This allows the error message to be informative
        return candidates[0] if candidates else Path('models') / model_name

    def watchdog_callback(self):
        """Warn if no frames received."""
        if self.frames_received == 0:
            self.get_logger().warn(f'No depth images received on {self.depth_topic}')
        elif self.last_frame_time is not None:
            elapsed = time.time() - self.last_frame_time
            if elapsed > 2.0:
                self.get_logger().warn(f'No depth images for {elapsed:.1f}s')

    def depth_callback(self, msg: Image):
        """Process incoming depth image."""
        self.frames_received += 1
        self.last_frame_time = time.time()

        try:
            # Convert ROS Image to numpy array
            depth_m = self.ros_image_to_depth_meters(msg)
            if depth_m is None:
                return

            # Preprocess
            tensor = self.preprocess(depth_m)

            # Run inference
            logits = self.run_inference(tensor)
            if logits is None:
                return

            # Postprocess
            label, confidence, open_prob = self.postprocess(logits)

            # Print result (rate-limited)
            self.print_result(label, confidence, open_prob)

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}', throttle_duration_sec=2.0)

    def ros_image_to_depth_meters(self, msg: Image) -> np.ndarray:
        """
        Convert ROS Image to depth in meters as 2D numpy array.
        Supports 32FC1 (float32 meters) and 16UC1 (uint16 millimeters).
        Does NOT use cv_bridge.
        """
        encoding = msg.encoding
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data

        if encoding == '32FC1':
            # float32, depth in meters
            expected_step = width * 4
            if step != expected_step:
                self.get_logger().warn(
                    f'Step mismatch for 32FC1: expected {expected_step}, got {step}',
                    throttle_duration_sec=5.0
                )
            depth_m = np.frombuffer(data, dtype=np.float32).reshape((height, width))

        elif encoding == '16UC1':
            # uint16, depth in millimeters
            expected_step = width * 2
            if step != expected_step:
                self.get_logger().warn(
                    f'Step mismatch for 16UC1: expected {expected_step}, got {step}',
                    throttle_duration_sec=5.0
                )
            depth_mm = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
            depth_m = depth_mm.astype(np.float32) * 0.001  # mm to m

        else:
            self.get_logger().error(
                f'Unsupported depth encoding: {encoding}. Expected 32FC1 or 16UC1.',
                throttle_duration_sec=5.0
            )
            return None

        return depth_m.copy()  # Copy to ensure contiguous and writable

    def preprocess(self, depth_m: np.ndarray) -> np.ndarray:
        """
        Preprocess depth image using EXACT training spec.
        
        Training preprocessing steps:
        1. Replace invalid (NaN/inf/<=0) with clip_max_m (5.0 = FAR, not 0!)
        2. Clip to [clip_min_m=0.2, clip_max_m=5.0]
        3. Normalize to [0, 1]
        4. Resize to 96x96 using cv2.INTER_AREA (area-based downsampling)
        5. Return float32 tensor [1, 1, 96, 96]
        
        Returns:
            Preprocessed tensor matching training spec exactly
        """
        # Compute stats before preprocessing (for debug logging)
        if self.debug_preprocess:
            current_time = time.time()
            # Throttle to ~1 Hz
            if current_time - self.last_debug_log_time >= 1.0:
                stats = compute_preprocessing_stats(depth_m, self.clip_min_m, self.clip_max_m)
                self.get_logger().info(
                    f'[PREPROCESS] Invalid: {stats["invalid_fraction"]*100:.1f}%, '
                    f'Valid depth: min={stats.get("valid_min", np.nan):.2f}m, '
                    f'mean={stats.get("valid_mean", np.nan):.2f}m, '
                    f'max={stats.get("valid_max", np.nan):.2f}m, '
                    f'Below {self.clip_min_m}m: {stats["below_clip_min"]}, '
                    f'Above {self.clip_max_m}m: {stats["above_clip_max"]}'
                )
                self.last_debug_log_time = current_time
        
        # Use canonical training spec preprocessing
        tensor = preprocess_depth_training_spec(
            depth_m,
            clip_min_m=self.clip_min_m,
            clip_max_m=self.clip_max_m,
            out_size=96
        )
        
        # Debug log normalized tensor range (throttled)
        if self.debug_preprocess:
            current_time = time.time()
            if current_time - self.last_debug_log_time >= 1.0:
                self.get_logger().info(
                    f'[PREPROCESS] Normalized tensor: '
                    f'min={tensor.min():.4f}, max={tensor.max():.4f}, '
                    f'shape={tensor.shape}, dtype={tensor.dtype}'
                )
        
        return tensor

    def run_inference(self, tensor: np.ndarray) -> np.ndarray:
        """Run ONNX inference."""
        try:
            outputs = self.session.run(
                [self.output_name],
                {self.input_name: tensor}
            )

            logits = outputs[0]

            # Validate output shape
            if logits.shape != (1, 2):
                self.get_logger().error(
                    f'Unexpected output shape: {logits.shape}, expected (1, 2)',
                    throttle_duration_sec=5.0
                )
                return None

            return logits

        except Exception as e:
            self.get_logger().error(f'ONNX inference failed: {e}', throttle_duration_sec=2.0)
            return None

    def postprocess(self, logits: np.ndarray) -> tuple:
        """
        Apply softmax and determine prediction.
        Returns: (label_str, confidence, open_probability)
        """
        # Softmax
        logits_shifted = logits - np.max(logits, axis=1, keepdims=True)
        exp_logits = np.exp(logits_shifted)
        probs = exp_logits / np.sum(exp_logits, axis=1, keepdims=True)

        # Get open probability
        open_prob = probs[0, self.open_index]
        closed_prob = 1.0 - open_prob

        # Determine label
        door_open = open_prob >= self.threshold
        label = 'OPEN' if door_open else 'CLOSED'
        confidence = open_prob if door_open else closed_prob

        return label, confidence, open_prob

    def print_result(self, label: str, confidence: float, open_prob: float):
        """Print result to terminal with rate limiting."""
        current_time = time.time()
        time_elapsed = (current_time - self.last_print_time) >= self.print_period
        label_changed = (label != self.last_prediction)

        if time_elapsed or label_changed:
            # Format output
            prob_bar = '█' * int(open_prob * 20) + '░' * (20 - int(open_prob * 20))

            print(
                f'Door: {label:6s} | '
                f'Open prob: [{prob_bar}] {open_prob*100:5.1f}%'
            )

            self.last_print_time = current_time
            self.last_prediction = label


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DoorClassifierNode()
        rclpy.spin(node)
    except (RuntimeError, FileNotFoundError) as e:
        print(f'ERROR: {e}')
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
