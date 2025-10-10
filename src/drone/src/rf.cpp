#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class RFToOdomNode : public rclcpp::Node
{
public:
  RFToOdomNode();

private:
  void rf_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rf_distance_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string base_frame_id_;
  std::string child_frame_id_;
  double z_covariance_;
};

// Constructor
RFToOdomNode::RFToOdomNode()
: Node("rf_to_odom_node")
{
  // Declare and get parameters
  this->declare_parameter<std::string>("base_frame_id", "odom");
  this->declare_parameter<std::string>("child_frame_id", "gps");
  this->declare_parameter<double>("z_covariance", 0.1);

  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  child_frame_id_ = this->get_parameter("child_frame_id").as_string();
  z_covariance_ = this->get_parameter("z_covariance").as_double();

  // Subscriber to RF distance
  rf_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/rf_distance", 10,
    std::bind(&RFToOdomNode::rf_distance_callback, this, std::placeholders::_1));

  // Publisher for odometry
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/rf_distance_odom", 10);

  RCLCPP_INFO(this->get_logger(), "RF to Odometry node started");
  RCLCPP_INFO(this->get_logger(), "Subscribing to: /rf_distance");
  RCLCPP_INFO(this->get_logger(), "Publishing to: /rf_distance_odom");
}

// Callback
void RFToOdomNode::rf_distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  auto odom_msg = nav_msgs::msg::Odometry();

  // Header
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = base_frame_id_;
  odom_msg.child_frame_id = child_frame_id_;

  // Set Z position
  odom_msg.pose.pose.position.z = msg->data;

  // Covariance (Z axis position is index 14 in the 6x6 matrix)
  odom_msg.pose.covariance[14] = z_covariance_;

  // Optionally, you can zero out the rest of the pose/velocity if not needed

  odom_pub_->publish(odom_msg);
}

// Main
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RFToOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
