#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TrajPub : public rclcpp::Node
{
public:
  TrajPub()
  : Node("Trajectory_Logger"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    reference_frame_ = "map";
    robot_frame_ = "base_link";

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/traj_pub", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&TrajPub::traj_publisher, this));

    path_.header.frame_id = reference_frame_;
  }

private:
  void traj_publisher()
  {
    try {
      // Get current time
      rclcpp::Time now = this->get_clock()->now();

      // Lookup transform from map -> base_link at latest time (0)
      geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
        reference_frame_, robot_frame_, tf2::TimePointZero);

      // Fill PoseStamped with transform info
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = reference_frame_;
      pose.header.stamp = now;

      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      // Append pose to path and update header timestamp
      path_.poses.push_back(pose);
      path_.header.stamp = now;

      path_pub_->publish(path_);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  robot_frame_.c_str(), reference_frame_.c_str(), ex.what());
    }
  }

  std::string reference_frame_;
  std::string robot_frame_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
