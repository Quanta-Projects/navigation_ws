#ifndef SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_MODEL_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_MODEL_ACTION_HPP_

/**
 * @file wait_for_door_open_model_action.hpp
 * @brief BT Action Node that uses ONNX depth classifier to detect door open state.
 *
 * Model I/O Specification:
 * ========================
 * - Input:  "depth_input" - float32 tensor [1, 1, 96, 96] (NCHW)
 *           Normalized depth in [0,1] range after clipping to [clip_min_m, clip_max_m]
 * - Output: "logits" - float32 tensor [1, 2]
 *           Raw logits for [class0, class1]. Apply softmax to get probabilities.
 *           open_index parameter specifies which class corresponds to OPEN.
 *
 * Expected Depth Encoding:
 * ========================
 * - "32FC1": float32 meters (direct use)
 * - "16UC1": uint16 millimeters (converted to meters via * 0.001)
 *
 * Why StatefulActionNode:
 * =======================
 * - Non-blocking: Returns RUNNING while waiting for door to open
 * - Temporal stability: Requires OPEN prediction for stable_time_sec before SUCCESS
 * - Lazy subscription: Subscribes onStart, unsubscribes onHalted to avoid always-on
 * - Reusable: Can be used multiple times in BT (elevator entry and exit)
 *
 * Stability Logic:
 * ================
 * - Each tick checks if model predicts OPEN with confidence >= threshold
 * - If OPEN: start/continue stable timer
 * - If CLOSED: reset stable timer
 * - If stable timer >= stable_time_sec: return SUCCESS
 * - If total time >= timeout_sec: return FAILURE
 */

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// ONNX Runtime headers
#include <onnxruntime_cxx_api.h>

namespace smrr_navigation
{

class WaitForDoorOpenModelAction : public BT::StatefulActionNode
{
public:
  WaitForDoorOpenModelAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  ~WaitForDoorOpenModelAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      // Topic and model configuration
      BT::InputPort<std::string>("depth_topic", "/zed2_left_camera/depth/image_raw",
        "Depth image topic"),
      BT::InputPort<std::string>("model_path", "",
        "ONNX model path (empty = use package default)"),

      // Timing parameters
      BT::InputPort<double>("timeout_sec", 30.0,
        "Timeout for door open detection"),
      BT::InputPort<double>("poll_rate_hz", 10.0,
        "Rate limit for inference (Hz)"),
      BT::InputPort<double>("max_depth_stale_sec", 1.0,
        "Max age for depth data before failure"),
      BT::InputPort<double>("stable_time_sec", 1.0,
        "Time door must remain OPEN for SUCCESS"),

      // Depth normalization
      BT::InputPort<double>("clip_min_m", 0.0,
        "Minimum depth clip in meters"),
      BT::InputPort<double>("clip_max_m", 5.0,
        "Maximum depth clip in meters"),

      // Classification parameters
      BT::InputPort<int>("open_index", 0,
        "Class index for OPEN (0 or 1)"),
      BT::InputPort<double>("threshold", 0.5,
        "Confidence threshold for OPEN classification"),

      // Debug
      BT::InputPort<bool>("debug_log", false,
        "Enable verbose debug logging")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Load parameters from BT ports
  void loadParameters();

  // Depth image callback
  void depthCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Convert ROS Image to float depth in meters
  bool convertToDepthMeters(const sensor_msgs::msg::Image & msg,
                            std::vector<float> & depth_out,
                            int & width_out, int & height_out);

  // Area-based downsampling to match cv2.INTER_AREA (training spec)
  void resizeAreaDownsample(const std::vector<float> & src, int src_w, int src_h,
                            std::vector<float> & dst, int dst_w, int dst_h);

  // Preprocess depth to model input tensor (matches training preprocessing exactly)
  void preprocessDepth(const std::vector<float> & depth_m, int width, int height,
                       std::vector<float> & tensor_out);

  // Run ONNX inference and return softmax probabilities
  bool runInference(const std::vector<float> & input_tensor,
                    float & p_open_out);

  // Initialize ONNX Runtime session (once)
  bool initOnnxSession();

  // ROS node handle
  rclcpp::Node::SharedPtr node_;

  // Depth subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  // Latest depth frame (thread-safe)
  std::mutex depth_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  rclcpp::Time latest_depth_time_;

  // ONNX Runtime objects
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::unique_ptr<Ort::MemoryInfo> ort_memory_info_;
  bool onnx_initialized_;

  // Parameters (loaded from ports)
  std::string depth_topic_;
  std::string model_path_;
  double timeout_sec_;
  double poll_rate_hz_;
  double max_depth_stale_sec_;
  double stable_time_sec_;
  double clip_min_m_;
  double clip_max_m_;
  int open_index_;
  double threshold_;
  bool debug_log_;

  // Runtime state
  rclcpp::Time start_time_;
  rclcpp::Time last_poll_time_;
  rclcpp::Time stable_start_time_;
  bool currently_open_;
  bool started_;

  // Model constants
  static constexpr int MODEL_INPUT_SIZE = 96;
  static constexpr int MODEL_NUM_CLASSES = 2;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_MODEL_ACTION_HPP_
