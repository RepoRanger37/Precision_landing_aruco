#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <GeographicLib/LocalCartesian.hpp>

class GPSSmootherNode : public rclcpp::Node
{
public:
  GPSSmootherNode()
  : Node("gps_smoother_node"),
    alpha_(declare_parameter("alpha", 0.9)),
    origin_set_(false),
    initialized_(false)
  {
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/raw", 10,
      std::bind(&GPSSmootherNode::gpsCallback, this, std::placeholders::_1)
    );

    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10
    );

    RCLCPP_INFO(this->get_logger(), "✅ GPS Smoother Node Started");
  }

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "⚠️ No GPS fix.");
      return;
    }

    if (!origin_set_) {
      origin_ = GeographicLib::LocalCartesian(msg->latitude, msg->longitude, msg->altitude);
      origin_set_ = true;
      RCLCPP_INFO(this->get_logger(), "🧭 Origin set to lat=%.8f lon=%.8f alt=%.2f",
                  msg->latitude, msg->longitude, msg->altitude);
    }

    double x, y, z;
    origin_.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    if (!initialized_) {
      smoothed_x_ = x;
      smoothed_y_ = y;
      smoothed_z_ = z;
      initialized_ = true;
    } else {
      smoothed_x_ = alpha_ * smoothed_x_ + (1.0 - alpha_) * x;
      smoothed_y_ = alpha_ * smoothed_y_ + (1.0 - alpha_) * y;
      smoothed_z_ = z;  // no smoothing for Z
    }

    double lat, lon, alt;
    origin_.Reverse(smoothed_x_, smoothed_y_, smoothed_z_, lat, lon, alt);

    auto filtered_msg = *msg;
    filtered_msg.latitude = lat;
    filtered_msg.longitude = lon;
    filtered_msg.altitude = alt;

    // Optional: Adjust covariance to reflect smoother confidence
    filtered_msg.position_covariance[0] = 2.0;   // x
    filtered_msg.position_covariance[4] = 2.0;   // y
    filtered_msg.position_covariance[8] = 0.5;   // z
    filtered_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_pub_->publish(filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

  double alpha_;
  bool origin_set_;
  bool initialized_;
  double smoothed_x_, smoothed_y_, smoothed_z_;

  GeographicLib::LocalCartesian origin_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSSmootherNode>());
  rclcpp::shutdown();
  return 0;
}
