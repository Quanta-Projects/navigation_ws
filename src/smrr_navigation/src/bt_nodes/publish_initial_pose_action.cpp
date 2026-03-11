#include "smrr_navigation/bt_nodes/publish_initial_pose_action.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace smrr_navigation
{

// ─── Helper: geometry_msgs::msg::Pose → tf2::Transform ────────────────────────
static tf2::Transform poseToTf(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf;
  tf2::fromMsg(pose, tf);
  return tf;
}

// ─── Helper: tf2::Transform → geometry_msgs::msg::Pose ────────────────────────
static geometry_msgs::msg::Pose tfToPose(const tf2::Transform & tf)
{
  geometry_msgs::msg::Pose pose;
  tf2::toMsg(tf, pose);
  return pose;
}

// ─── Constructor ──────────────────────────────────────────────────────────────
PublishInitialPoseAction::PublishInitialPoseAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Retrieve shared ROS node from blackboard (set by the BT mission executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishInitialPoseAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  // Initialise TF2 buffer + listener (used for both parking-error and AprilTag modes)
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// ─── tick() ───────────────────────────────────────────────────────────────────
BT::NodeStatus PublishInitialPoseAction::tick()
{
  // ── Mandatory input ────────────────────────────────────────────────────────
  auto initial_pose_input = getInput<geometry_msgs::msg::PoseStamped>("initial_pose");
  if (!initial_pose_input) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PublishInitialPoseAction"),
      "Missing required input: initial_pose");
    return BT::NodeStatus::FAILURE;
  }
  const auto & target_pose_stamped = initial_pose_input.value();

  // ── Optional inputs ────────────────────────────────────────────────────────
  std::string topic_name = "/initialpose";
  auto topic_input = getInput<std::string>("topic_name");
  if (topic_input.has_value() && !topic_input.value().empty()) {
    topic_name = topic_input.value();
  }

  std::string frame_id_override;
  auto frame_input = getInput<std::string>("frame_id");
  if (frame_input.has_value()) {
    frame_id_override = frame_input.value();
  }

  // ── Create publisher on first tick ────────────────────────────────────────
  if (!initial_pose_pub_) {
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic_name, 10);
  }

  // ── Build message skeleton (frame, stamp, covariance) ─────────────────────
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = node_->get_clock()->now();
  pose_msg.header.frame_id = (!frame_id_override.empty())
    ? frame_id_override
    : (target_pose_stamped.header.frame_id.empty()
        ? "map" : target_pose_stamped.header.frame_id);

  // Standard 2-D covariance: 0.5 m / ~15 deg std dev
  pose_msg.pose.covariance[0]  = 0.25;    // x
  pose_msg.pose.covariance[7]  = 0.25;    // y
  pose_msg.pose.covariance[14] = 0.0;     // z  (unused in 2D)
  pose_msg.pose.covariance[21] = 0.0;     // roll  (unused in 2D)
  pose_msg.pose.covariance[28] = 0.0;     // pitch (unused in 2D)
  pose_msg.pose.covariance[35] = 0.0685;  // yaw

  // ══════════════════════════════════════════════════════════════════════════
  // MODE 1 — AprilTag relocalization
  // ══════════════════════════════════════════════════════════════════════════
  bool use_apriltag = false;
  auto apriltag_input = getInput<bool>("use_apriltag");
  if (apriltag_input.has_value()) {
    use_apriltag = apriltag_input.value();
  }

  if (use_apriltag) {
    std::string tag_frame  = getInput<std::string>("tag_frame").value_or("tag36h11:0");
    std::string base_frame = getInput<std::string>("base_frame").value_or("base_link");

    double exp_x     = getInput<double>("expected_tag_x").value_or(0.0);
    double exp_y     = getInput<double>("expected_tag_y").value_or(0.0);
    double exp_z     = getInput<double>("expected_tag_z").value_or(0.0);
    double exp_roll  = getInput<double>("expected_tag_roll").value_or(0.0);
    double exp_pitch = getInput<double>("expected_tag_pitch").value_or(0.0);
    double exp_yaw   = getInput<double>("expected_tag_yaw").value_or(0.0);

    RCLCPP_INFO(
      rclcpp::get_logger("PublishInitialPoseAction"),
      "[PublishInitialPose] AprilTag mode — looking up TF: %s → %s",
      base_frame.c_str(), tag_frame.c_str());

    // ── Live TF lookup: base_frame → tag_frame ─────────────────────────────
    geometry_msgs::msg::TransformStamped tag_tf_stamped;
    bool tag_lookup_ok = true;
    try {
      tag_tf_stamped = tf_buffer_->lookupTransform(
        base_frame,           // target frame (robot base)
        tag_frame,            // source frame (tag)
        tf2::TimePointZero);  // latest available
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        rclcpp::get_logger("PublishInitialPoseAction"),
        "[PublishInitialPose] AprilTag TF lookup failed: %s — falling back to direct publish",
        ex.what());
      tag_lookup_ok = false;
    }

    if (tag_lookup_ok) {
      // ── a. T_map_to_ideal_base ───────────────────────────────────────────
      tf2::Transform map_to_ideal_base = poseToTf(target_pose_stamped.pose);

      // ── b. T_ideal_base_to_tag (all 6-DOF to handle optical frame) ───────
      tf2::Quaternion ideal_tag_quat;
      ideal_tag_quat.setRPY(exp_roll, exp_pitch, exp_yaw);
      tf2::Transform ideal_base_to_tag(
        ideal_tag_quat,
        tf2::Vector3(exp_x, exp_y, exp_z));

      // ── c. T_actual_base_to_tag (from live TF lookup) ────────────────────
      tf2::Transform actual_base_to_tag;
      tf2::fromMsg(tag_tf_stamped.transform, actual_base_to_tag);

      // ── d. Compute corrected map pose ─────────────────────────────────────
      //   T_map_to_actual_base = T_map_to_ideal_base
      //                        * T_ideal_base_to_tag
      //                        * T_actual_base_to_tag.inverse()
      tf2::Transform map_to_actual_base =
        map_to_ideal_base * ideal_base_to_tag * actual_base_to_tag.inverse();

      // ── e. Extract yaw from 3D result ─────────────────────────────────────
      double corrected_yaw = tf2::getYaw(map_to_actual_base.getRotation());

      // ── f. Flatten to strict 2D (z=0, roll=0, pitch=0) ───────────────────
      double corrected_x = map_to_actual_base.getOrigin().x();
      double corrected_y = map_to_actual_base.getOrigin().y();
      tf2::Quaternion flat_quat;
      flat_quat.setRPY(0.0, 0.0, corrected_yaw);
      flat_quat.normalize();

      pose_msg.pose.pose.position.x  = corrected_x;
      pose_msg.pose.pose.position.y  = corrected_y;
      pose_msg.pose.pose.position.z  = 0.0;
      pose_msg.pose.pose.orientation = tf2::toMsg(flat_quat);

      const double ideal_x   = target_pose_stamped.pose.position.x;
      const double ideal_y   = target_pose_stamped.pose.position.y;
      const double ideal_yaw = tf2::getYaw(target_pose_stamped.pose.orientation);

      RCLCPP_INFO(
        rclcpp::get_logger("PublishInitialPoseAction"),
        "[PublishInitialPose] AprilTag correction applied — "
        "ideal: (%.3f, %.3f, %.4f rad) → corrected: (%.3f, %.3f, %.4f rad) | "
        "delta: (dx=%.3f m, dy=%.3f m, dyaw=%.4f rad)",
        ideal_x, ideal_y, ideal_yaw,
        corrected_x, corrected_y, corrected_yaw,
        corrected_x - ideal_x, corrected_y - ideal_y, corrected_yaw - ideal_yaw);

      initial_pose_pub_->publish(pose_msg);
      RCLCPP_INFO(
        rclcpp::get_logger("PublishInitialPoseAction"),
        "[PublishInitialPose] Published successfully (AprilTag mode) on topic: %s",
        topic_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    // tag_lookup_ok == false: fall through to direct publish
  }

  // ══════════════════════════════════════════════════════════════════════════
  // MODE 2 — Relative parking-error correction (current_expected_pose port)
  // ══════════════════════════════════════════════════════════════════════════
  auto current_expected_input =
    getInput<geometry_msgs::msg::PoseStamped>("current_expected_pose");

  if (!current_expected_input) {
    // ══════════════════════════════════════════════════════════════════════
    // MODE 3 — Direct publish (no correction)
    // ══════════════════════════════════════════════════════════════════════
    pose_msg.pose.pose = target_pose_stamped.pose;

    RCLCPP_INFO(
      rclcpp::get_logger("PublishInitialPoseAction"),
      "[PublishInitialPose] Direct publish — frame='%s', x=%.3f, y=%.3f, yaw=%.3f rad",
      pose_msg.header.frame_id.c_str(),
      pose_msg.pose.pose.position.x,
      pose_msg.pose.pose.position.y,
      tf2::getYaw(pose_msg.pose.pose.orientation));

  } else {
    // ── Relative-error-adjusted publish ──────────────────────────────────────
    //
    //   error_tf  = expected_tf.inverse() * actual_tf
    //   adjusted  = target_tf * error_tf
    //
    // Look up actual robot pose in the departure-floor map frame via TF.
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        "map", "base_link",
        tf2::TimePointZero);          // latest available transform
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PublishInitialPoseAction"),
        "[PublishInitialPose] TF lookup (map→base_link) failed: %s — returning FAILURE",
        ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // Convert the three poses to tf2::Transform
    const auto & expected_pose = current_expected_input.value().pose;
    tf2::Transform expected_tf = poseToTf(expected_pose);

    geometry_msgs::msg::Pose actual_pose;
    actual_pose.position.x    = tf_stamped.transform.translation.x;
    actual_pose.position.y    = tf_stamped.transform.translation.y;
    actual_pose.position.z    = tf_stamped.transform.translation.z;
    actual_pose.orientation   = tf_stamped.transform.rotation;
    tf2::Transform actual_tf  = poseToTf(actual_pose);

    tf2::Transform target_tf  = poseToTf(target_pose_stamped.pose);

    // Parking error on departure floor
    tf2::Transform error_tf   = expected_tf.inverse() * actual_tf;

    // Apply same error to target floor nominal pose
    tf2::Transform adjusted_tf = target_tf * error_tf;

    pose_msg.pose.pose = tfToPose(adjusted_tf);

    // Compute error components for logging
    const double err_x   = error_tf.getOrigin().x();
    const double err_y   = error_tf.getOrigin().y();
    const double err_yaw = tf2::getYaw(error_tf.getRotation());

    RCLCPP_INFO(
      rclcpp::get_logger("PublishInitialPoseAction"),
      "[PublishInitialPose] Relative error-adjusted publish — "
      "parking error: dx=%.3f m, dy=%.3f m, dyaw=%.4f rad | "
      "adjusted pose: frame='%s', x=%.3f, y=%.3f, yaw=%.3f rad",
      err_x, err_y, err_yaw,
      pose_msg.header.frame_id.c_str(),
      pose_msg.pose.pose.position.x,
      pose_msg.pose.pose.position.y,
      tf2::getYaw(pose_msg.pose.pose.orientation));
  }

  // ── Publish ───────────────────────────────────────────────────────────────
  initial_pose_pub_->publish(pose_msg);

  RCLCPP_INFO(
    rclcpp::get_logger("PublishInitialPoseAction"),
    "[PublishInitialPose] Published successfully on topic: %s", topic_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
