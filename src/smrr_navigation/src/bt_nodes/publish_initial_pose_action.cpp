#include "smrr_navigation/bt_nodes/publish_initial_pose_action.hpp"

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

  // Initialise TF2 buffer + listener for parking-error correction
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
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

  // ── Decide: relative-error path vs. direct publish ────────────────────────
  auto current_expected_input =
    getInput<geometry_msgs::msg::PoseStamped>("current_expected_pose");

  if (!current_expected_input) {
    // ── Direct publish (no parking-error correction) ─────────────────────────
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
