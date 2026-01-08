/**
 * @file wait_for_door_open_model_action.cpp
 * @brief ONNX-based door classifier BT action node implementation.
 *
 * This node uses a trained depth classifier (TinyCNN) to detect when
 * an elevator door has opened. The model was trained on full-frame
 * depth images resized to 96x96.
 *
 * Model Details:
 * - Architecture: TinyCNN (~15K params)
 * - Input: depth_input [1, 1, 96, 96] float32, normalized [0,1]
 * - Output: logits [1, 2] float32
 * - Classes: Configurable via open_index (default: 0=OPEN, 1=CLOSED)
 *
 * Depth Preprocessing:
 * 1. Convert from 32FC1 (meters) or 16UC1 (mm) to float32 meters
 * 2. Replace invalid pixels (NaN, inf, <=0) with 0.0
 * 3. Resize full image to 96x96 using nearest-neighbor
 * 4. Clip to [clip_min_m, clip_max_m] range
 * 5. Normalize to [0, 1]
 * 6. Reshape to [1, 1, 96, 96] NCHW tensor
 */

#include "smrr_navigation/bt_nodes/wait_for_door_open_model_action.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace smrr_navigation
{

WaitForDoorOpenModelAction::WaitForDoorOpenModelAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  onnx_initialized_(false),
  currently_open_(false),
  started_(false)
{
  // Get ROS node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitForDoorOpenModelAction"),
                 "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

void WaitForDoorOpenModelAction::loadParameters()
{
  // Defaults
  depth_topic_ = "/zed2_left_camera/depth/image_raw";
  model_path_ = "";
  timeout_sec_ = 30.0;
  poll_rate_hz_ = 10.0;
  max_depth_stale_sec_ = 1.0;
  stable_time_sec_ = 1.0;
  clip_min_m_ = 0.0;
  clip_max_m_ = 5.0;
  open_index_ = 0;
  threshold_ = 0.5;
  debug_log_ = false;

  // Load from ports
  auto depth_topic_input = getInput<std::string>("depth_topic");
  if (depth_topic_input.has_value()) depth_topic_ = depth_topic_input.value();

  auto model_path_input = getInput<std::string>("model_path");
  if (model_path_input.has_value()) model_path_ = model_path_input.value();

  auto timeout_input = getInput<double>("timeout_sec");
  if (timeout_input.has_value()) timeout_sec_ = timeout_input.value();

  auto poll_input = getInput<double>("poll_rate_hz");
  if (poll_input.has_value()) poll_rate_hz_ = poll_input.value();

  auto stale_input = getInput<double>("max_depth_stale_sec");
  if (stale_input.has_value()) max_depth_stale_sec_ = stale_input.value();

  auto stable_input = getInput<double>("stable_time_sec");
  if (stable_input.has_value()) stable_time_sec_ = stable_input.value();

  auto clip_min_input = getInput<double>("clip_min_m");
  if (clip_min_input.has_value()) clip_min_m_ = clip_min_input.value();

  auto clip_max_input = getInput<double>("clip_max_m");
  if (clip_max_input.has_value()) clip_max_m_ = clip_max_input.value();

  auto open_index_input = getInput<int>("open_index");
  if (open_index_input.has_value()) open_index_ = open_index_input.value();

  auto threshold_input = getInput<double>("threshold");
  if (threshold_input.has_value()) threshold_ = threshold_input.value();

  auto debug_input = getInput<bool>("debug_log");
  if (debug_input.has_value()) debug_log_ = debug_input.value();

  // Resolve default model path if empty
  if (model_path_.empty()) {
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("smrr_navigation");
      model_path_ = pkg_share + "/models/door_classifier.onnx";
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(),
                  "Could not find package share dir, using hardcoded model path");
      model_path_ = "/home/achiraubuntu/navigation_ws/ml_door_classifier/outputs/onnx/door_classifier.onnx";
    }
  }
}

bool WaitForDoorOpenModelAction::initOnnxSession()
{
  if (onnx_initialized_) {
    return true;
  }

  try {
    RCLCPP_INFO(node_->get_logger(), "Loading ONNX model: %s", model_path_.c_str());

    // Create ONNX Runtime environment
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "WaitForDoorOpenModel");

    // Session options
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create session
    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path_.c_str(), session_options);

    // Create memory info for CPU
    ort_memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    onnx_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "ONNX model loaded successfully");

    return true;

  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "ONNX Runtime error: %s", e.what());
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load ONNX model: %s", e.what());
    return false;
  }
}

void WaitForDoorOpenModelAction::depthCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = msg;
  latest_depth_time_ = node_->now();
}

BT::NodeStatus WaitForDoorOpenModelAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "WaitForDoorOpenModel: Starting door detection");

  // Load parameters
  loadParameters();

  // Initialize ONNX session
  if (!initOnnxSession()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize ONNX session");
    return BT::NodeStatus::FAILURE;
  }

  // Reset state
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_.reset();
  }
  currently_open_ = false;
  started_ = true;
  start_time_ = node_->now();
  last_poll_time_ = start_time_;
  stable_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Subscribe to depth topic
  depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::SensorDataQoS(),
    std::bind(&WaitForDoorOpenModelAction::depthCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "Subscribed to %s, waiting for door to open (timeout=%.1fs, stable=%.1fs)",
              depth_topic_.c_str(), timeout_sec_, stable_time_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDoorOpenModelAction::onRunning()
{
  rclcpp::Time now = node_->now();

  // Check timeout
  double elapsed = (now - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(),
                "WaitForDoorOpenModel: Timeout after %.1f seconds", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  // Rate limiting
  double poll_period = 1.0 / std::max(poll_rate_hz_, 1.0);
  if ((now - last_poll_time_).seconds() < poll_period) {
    return BT::NodeStatus::RUNNING;
  }
  last_poll_time_ = now;

  // Get latest depth frame
  sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
  rclcpp::Time depth_time;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!latest_depth_) {
      if (debug_log_) {
        RCLCPP_DEBUG(node_->get_logger(), "No depth frame received yet");
      }
      return BT::NodeStatus::RUNNING;
    }
    depth_msg = latest_depth_;
    depth_time = latest_depth_time_;
  }

  // Check stale frame
  double frame_age = (now - depth_time).seconds();
  if (frame_age > max_depth_stale_sec_) {
    RCLCPP_WARN(node_->get_logger(),
                "Depth frame stale (age=%.2fs > max=%.2fs)",
                frame_age, max_depth_stale_sec_);
    return BT::NodeStatus::FAILURE;
  }

  // Convert depth to meters
  std::vector<float> depth_m;
  int width, height;
  if (!convertToDepthMeters(*depth_msg, depth_m, width, height)) {
    return BT::NodeStatus::FAILURE;
  }

  // Preprocess to model input tensor
  std::vector<float> input_tensor;
  preprocessDepth(depth_m, width, height, input_tensor);

  // Run inference
  float p_open;
  if (!runInference(input_tensor, p_open)) {
    RCLCPP_ERROR(node_->get_logger(), "Inference failed");
    return BT::NodeStatus::RUNNING;  // Continue trying
  }

  // Decision
  bool model_says_open = (p_open >= threshold_);

  // Always log probability and prediction for debugging
  RCLCPP_INFO(node_->get_logger(),
              "[WaitForDoorOpenModel] P(OPEN)=%.3f, P(CLOSED)=%.3f, Threshold=%.2f -> Predicted: %s",
              p_open, 1.0f - p_open, threshold_, model_says_open ? "OPEN" : "CLOSED");

  if (debug_log_) {
    RCLCPP_INFO(node_->get_logger(),
                "p_open=%.3f threshold=%.3f -> %s",
                p_open, threshold_, model_says_open ? "OPEN" : "CLOSED");
  }

  // Temporal stability logic
  if (model_says_open) {
    if (!currently_open_) {
      // Transition to open - start stable timer
      stable_start_time_ = now;
      currently_open_ = true;
      if (debug_log_) {
        RCLCPP_INFO(node_->get_logger(), "Door detected OPEN, starting stability timer");
      }
    }

    // Check if stable long enough
    double stable_duration = (now - stable_start_time_).seconds();
    if (stable_duration >= stable_time_sec_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Door OPEN stable for %.2fs - SUCCESS! (p_open=%.3f)",
                  stable_duration, p_open);
      return BT::NodeStatus::SUCCESS;
    }

    if (debug_log_) {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Door OPEN for %.2f/%.2fs", stable_duration, stable_time_sec_);
    }

  } else {
    // Door closed - reset stable timer
    if (currently_open_ && debug_log_) {
      RCLCPP_INFO(node_->get_logger(), "Door detected CLOSED, resetting stability timer");
    }
    currently_open_ = false;
    stable_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForDoorOpenModelAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "WaitForDoorOpenModel: Halted");

  // Unsubscribe to avoid dangling callbacks
  depth_sub_.reset();

  // Clear state
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_.reset();
  }
  currently_open_ = false;
  started_ = false;
}

bool WaitForDoorOpenModelAction::convertToDepthMeters(
  const sensor_msgs::msg::Image & msg,
  std::vector<float> & depth_out,
  int & width_out, int & height_out)
{
  const std::string & encoding = msg.encoding;
  int height = static_cast<int>(msg.height);
  int width = static_cast<int>(msg.width);
  int step = static_cast<int>(msg.step);

  depth_out.resize(height * width);
  width_out = width;
  height_out = height;

  if (encoding == "32FC1") {
    // Float32 meters
    if (step != width * 4) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "32FC1 step mismatch: expected %d, got %d", width * 4, step);
    }
    const float * src = reinterpret_cast<const float *>(msg.data.data());
    for (int i = 0; i < height * width; ++i) {
      float d = src[i];
      // Replace invalid values
      if (!std::isfinite(d) || d <= 0.0f) {
        depth_out[i] = 0.0f;
      } else {
        depth_out[i] = d;
      }
    }
    return true;

  } else if (encoding == "16UC1") {
    // Uint16 millimeters
    if (step != width * 2) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "16UC1 step mismatch: expected %d, got %d", width * 2, step);
    }
    const uint16_t * src = reinterpret_cast<const uint16_t *>(msg.data.data());
    for (int i = 0; i < height * width; ++i) {
      uint16_t mm = src[i];
      if (mm == 0) {
        depth_out[i] = 0.0f;
      } else {
        depth_out[i] = static_cast<float>(mm) * 0.001f;  // mm to meters
      }
    }
    return true;

  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                          "Unsupported depth encoding: %s (expected 32FC1 or 16UC1)",
                          encoding.c_str());
    return false;
  }
}

void WaitForDoorOpenModelAction::resizeNearestNeighbor(
  const std::vector<float> & src, int src_w, int src_h,
  std::vector<float> & dst, int dst_w, int dst_h)
{
  dst.resize(dst_w * dst_h);

  float x_ratio = static_cast<float>(src_w) / dst_w;
  float y_ratio = static_cast<float>(src_h) / dst_h;

  for (int y = 0; y < dst_h; ++y) {
    int src_y = std::min(static_cast<int>(y * y_ratio), src_h - 1);
    for (int x = 0; x < dst_w; ++x) {
      int src_x = std::min(static_cast<int>(x * x_ratio), src_w - 1);
      dst[y * dst_w + x] = src[src_y * src_w + src_x];
    }
  }
}

void WaitForDoorOpenModelAction::preprocessDepth(
  const std::vector<float> & depth_m, int width, int height,
  std::vector<float> & tensor_out)
{
  // Resize to 96x96
  std::vector<float> resized;
  resizeNearestNeighbor(depth_m, width, height, resized,
                        MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);

  // Clip and normalize
  tensor_out.resize(MODEL_INPUT_SIZE * MODEL_INPUT_SIZE);
  float clip_min_f = static_cast<float>(clip_min_m_);
  float clip_max_f = static_cast<float>(clip_max_m_);
  float range = clip_max_f - clip_min_f;
  float eps = 1e-6f;

  for (size_t i = 0; i < resized.size(); ++i) {
    float d = resized[i];
    // Clip
    d = std::max(clip_min_f, std::min(d, clip_max_f));
    // Normalize to [0, 1]
    tensor_out[i] = (d - clip_min_f) / (range + eps);
  }
}

bool WaitForDoorOpenModelAction::runInference(
  const std::vector<float> & input_tensor,
  float & p_open_out)
{
  if (!onnx_initialized_ || !ort_session_) {
    return false;
  }

  try {
    // Input shape: [1, 1, 96, 96]
    std::array<int64_t, 4> input_shape = {1, 1, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE};
    size_t input_size = input_tensor.size();

    // Create input tensor
    Ort::Value input_ort = Ort::Value::CreateTensor<float>(
      *ort_memory_info_,
      const_cast<float *>(input_tensor.data()),
      input_size,
      input_shape.data(),
      input_shape.size());

    // Input/output names
    const char * input_names[] = {"depth_input"};
    const char * output_names[] = {"logits"};

    // Run inference
    auto output_tensors = ort_session_->Run(
      Ort::RunOptions{nullptr},
      input_names, &input_ort, 1,
      output_names, 1);

    // Get output
    float * logits = output_tensors[0].GetTensorMutableData<float>();

    // Validate output shape
    auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() != 2 || shape[0] != 1 || shape[1] != MODEL_NUM_CLASSES) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Unexpected output shape: [%ld, %ld], expected [1, 2]",
                   shape[0], shape[1]);
      return false;
    }

    // Compute softmax
    float max_logit = std::max(logits[0], logits[1]);
    float exp0 = std::exp(logits[0] - max_logit);
    float exp1 = std::exp(logits[1] - max_logit);
    float sum_exp = exp0 + exp1;

    float prob0 = exp0 / sum_exp;
    float prob1 = exp1 / sum_exp;

    // Get probability for OPEN class
    if (open_index_ == 0) {
      p_open_out = prob0;
    } else {
      p_open_out = prob1;
    }

    return true;

  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "ONNX inference error: %s", e.what());
    return false;
  }
}

}  // namespace smrr_navigation
