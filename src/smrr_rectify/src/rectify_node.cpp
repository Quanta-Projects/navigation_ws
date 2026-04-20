#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc.hpp>

class RectifyNode : public rclcpp::Node
{
public:
  RectifyNode()
  : Node("rectify_node")
  {
    // Declare parameters
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<int>("interpolation", cv::INTER_LINEAR);
    int queue_size = this->get_parameter("queue_size").as_int();
    interpolation_ = this->get_parameter("interpolation").as_int();

    // Use SensorDataQoS so we match any publisher (RELIABLE or BEST_EFFORT)
    auto qos = rclcpp::SensorDataQoS();

    // Set up message_filters subscribers with explicit QoS
    image_sub_.subscribe(this, "image_raw", qos.get_rmw_qos_profile());
    info_sub_.subscribe(this, "camera_info", qos.get_rmw_qos_profile());

    // ApproximateTime synchronizer
    sync_ = std::make_shared<Sync>(SyncPolicy(queue_size), image_sub_, info_sub_);
    sync_->registerCallback(
      std::bind(&RectifyNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher for rectified image — use SensorDataQoS to match RealSense QoS
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_rect", qos);

    RCLCPP_INFO(this->get_logger(), "Rectify node started (ApproximateTime sync, queue=%d)", queue_size);
  }

private:
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // Verify camera is calibrated
    if (info_msg->k[0] == 0.0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Camera publishing on '%s' is uncalibrated",
        info_sub_.getTopic().c_str());
      return;
    }

    // Update camera model
    model_.fromCameraInfo(info_msg);

    // Check for zero distortion — just pass through
    bool zero_distortion = true;
    for (size_t i = 0; i < info_msg->d.size(); ++i) {
      if (info_msg->d[i] != 0.0) {
        zero_distortion = false;
        break;
      }
    }

    if (zero_distortion || info_msg->d.empty()) {
      pub_->publish(*image_msg);
      return;
    }

    // Convert to OpenCV
    cv_bridge::CvImageConstPtr cv_image;
    try {
      cv_image = cv_bridge::toCvShare(image_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Rectify
    cv::Mat rectified;
    model_.rectifyImage(cv_image->image, rectified, interpolation_);

    // Publish with original header
    auto rect_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, rectified).toImageMsg();
    pub_->publish(*rect_msg);
  }

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  image_geometry::PinholeCameraModel model_;
  int interpolation_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectifyNode>());
  rclcpp::shutdown();
  return 0;
}
