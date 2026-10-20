// Keep image transport small on the competition USB OS without extra packages.
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

namespace gazebo {
class CompetitionCameraStream : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
    if (!sensor_) { gzerr << "CompetitionCameraStream requires a camera sensor\n"; return; }
    node_ = gazebo_ros::Node::Get(sdf);
    frame_ = sdf->Get<std::string>("frame_name", "camera_link").first;
    const auto topic = sdf->Get<std::string>("topic_name", "/camera/image_raw/compressed").first;
    publisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    connection_ = sensor_->Camera()->ConnectNewImageFrame(std::bind(
        &CompetitionCameraStream::Frame, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5));
    sensor_->SetActive(true);
    RCLCPP_INFO(node_->get_logger(), "JPEG camera stream ready: %s", topic.c_str());
  }
  ~CompetitionCameraStream() override { connection_.reset(); }

 private:
  void Frame(const unsigned char *data, unsigned width, unsigned height,
             unsigned depth, const std::string &format) {
    if (!publisher_ || publisher_->get_subscription_count() == 0) return;
    if (depth != 3) return;
    try {
      cv::Mat input(height, width, CV_8UC3, const_cast<unsigned char *>(data)), bgr;
      if (format == "B8G8R8") bgr = input;
      else cv::cvtColor(input, bgr, cv::COLOR_RGB2BGR);
      sensor_msgs::msg::CompressedImage message;
      const auto stamp = sensor_->LastMeasurementTime();
      message.header.stamp.sec = stamp.sec;
      message.header.stamp.nanosec = stamp.nsec;
      message.header.frame_id = frame_;
      message.format = "bgr8; jpeg compressed bgr8";
      if (!cv::imencode(".jpg", bgr, message.data, {cv::IMWRITE_JPEG_QUALITY, 95})) return;
      publisher_->publish(message);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Camera JPEG encoding failed: %s", e.what());
    }
  }
  sensors::CameraSensorPtr sensor_;
  gazebo::event::ConnectionPtr connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  std::string frame_;
};
GZ_REGISTER_SENSOR_PLUGIN(CompetitionCameraStream)
}  // namespace gazebo
