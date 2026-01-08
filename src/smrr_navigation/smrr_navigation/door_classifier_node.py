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
        self.declare_parameter('open_index', 0)  # Model: 0=OPEN, 1=CLOSED
        self.declare_parameter('threshold', 0.8)
        self.declare_parameter('clip_min_m', 0.0)
        self.declare_parameter('clip_max_m', 5.0)
        self.declare_parameter('print_rate_hz', 5.0)

        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        model_path_param = self.get_parameter('model_path').value
        self.open_index = self.get_parameter('open_index').value
        self.threshold = self.get_parameter('threshold').value
        self.clip_min_m = self.get_parameter('clip_min_m').value
        self.clip_max_m = self.get_parameter('clip_max_m').value
        self.print_rate_hz = self.get_parameter('print_rate_hz').value

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

        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('Door Classifier Node Configuration')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Depth topic:    {self.depth_topic}')
        self.get_logger().info(f'  Model path:     {self.model_path}')
        self.get_logger().info(f'  Open index:     {self.open_index}')
        self.get_logger().info(f'  Threshold:      {self.threshold}')
        self.get_logger().info(f'  Clip range:     [{self.clip_min_m}, {self.clip_max_m}] m')
        self.get_logger().info(f'  Print rate:     {self.print_rate_hz} Hz')
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
        model_name = 'door_classifier.onnx'
        
        # Candidate paths to try
        candidates = []
        
        # 1. Resolve symlinks and look relative to actual source
        try:
            real_path = Path(__file__).resolve()
            candidates.append(real_path.parent / 'models' / model_name)
        except Exception:
            pass
        
        # 2. Relative to __file__ (may be symlink)
        candidates.append(Path(__file__).parent / 'models' / model_name)
        
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
        Preprocess depth image for model input.
        - Replace invalid values with 0.0
        - Resize to 96x96
        - Clip and normalize to [0, 1]
        - Return tensor shape [1, 1, 96, 96], dtype float32
        """
        # Replace invalid values (NaN, inf, <= 0) with 0.0
        valid_mask = np.isfinite(depth_m) & (depth_m > 0)
        depth_m = np.where(valid_mask, depth_m, 0.0)

        # Resize to 96x96
        target_size = (96, 96)
        if HAS_CV2:
            # Use OpenCV with nearest neighbor interpolation
            depth_resized = cv2.resize(
                depth_m.astype(np.float32),
                target_size,
                interpolation=cv2.INTER_NEAREST
            )
        else:
            # Fallback: numpy nearest-neighbor resize
            depth_resized = self.numpy_resize_nearest(depth_m, target_size)

        # Clip to valid range
        depth_clipped = np.clip(depth_resized, self.clip_min_m, self.clip_max_m)

        # Normalize to [0, 1]
        range_val = self.clip_max_m - self.clip_min_m
        eps = 1e-6
        depth_normalized = (depth_clipped - self.clip_min_m) / (range_val + eps)

        # Form tensor: [1, 1, 96, 96]
        tensor = depth_normalized.astype(np.float32).reshape(1, 1, 96, 96)

        return tensor

    def numpy_resize_nearest(self, img: np.ndarray, target_size: tuple) -> np.ndarray:
        """
        Simple nearest-neighbor resize using numpy indexing.
        target_size: (width, height)
        """
        target_w, target_h = target_size
        src_h, src_w = img.shape[:2]

        # Compute source indices for each target pixel
        x_indices = (np.arange(target_w) * src_w / target_w).astype(np.int32)
        y_indices = (np.arange(target_h) * src_h / target_h).astype(np.int32)

        # Clip to valid range
        x_indices = np.clip(x_indices, 0, src_w - 1)
        y_indices = np.clip(y_indices, 0, src_h - 1)

        # Use advanced indexing
        resized = img[np.ix_(y_indices, x_indices)]

        return resized.astype(np.float32)

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
            state_icon = '🚪' if label == 'OPEN' else '🔒'
            prob_bar = '█' * int(open_prob * 20) + '░' * (20 - int(open_prob * 20))

            print(
                f'{state_icon} Door: {label:6s} | '
                f'Confidence: {confidence*100:5.1f}% | '
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
