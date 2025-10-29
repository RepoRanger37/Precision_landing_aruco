#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MarkerVisualizer : public rclcpp::Node
{
public:
  MarkerVisualizer() : Node("marker_visualizer")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/aruco_marker_visualization", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&MarkerVisualizer::poseCallback, this, std::placeholders::_1));

    target_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/target_visible", 10);

    // Timer checks visibility every 200 ms
    check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MarkerVisualizer::checkVisibility, this));

    last_seen_time_ = this->now();
    target_visible_ = false;

    timeout_sec_ = this->declare_parameter("timeout_sec", 1.0);

    RCLCPP_INFO(get_logger(), "Marker Visualizer Node started!");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_seen_time_ = this->now();
    target_visible_ = true;

    double cz = msg->pose.position.z + 0.01;

    // --- Outer black square ---
    visualization_msgs::msg::Marker box;
    box.header = msg->header;
    box.ns = "landing_pad_box";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.scale.x = 0.05;
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 0.0;
    box.color.a = 1.0;
    box.lifetime = rclcpp::Duration::from_seconds(0.5);

    double box_size = 0.25;
    geometry_msgs::msg::Point p;
    p.z = cz;

    p.x = msg->pose.position.x - box_size; p.y = msg->pose.position.y - box_size; box.points.push_back(p);
    p.x = msg->pose.position.x + box_size; p.y = msg->pose.position.y - box_size; box.points.push_back(p);
    p.x = msg->pose.position.x + box_size; p.y = msg->pose.position.y + box_size; box.points.push_back(p);
    p.x = msg->pose.position.x - box_size; p.y = msg->pose.position.y + box_size; box.points.push_back(p);
    p.x = msg->pose.position.x - box_size; p.y = msg->pose.position.y - box_size; box.points.push_back(p);

    marker_pub_->publish(box);

    // --- Red H ---
    visualization_msgs::msg::Marker h_marker;
    h_marker.header = msg->header;
    h_marker.ns = "landing_pad_H";
    h_marker.id = 1;
    h_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    h_marker.action = visualization_msgs::msg::Marker::ADD;
    h_marker.scale.x = 0.02;
    h_marker.color.r = 1.0;
    h_marker.color.a = 1.0;
    h_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    double half_w = 0.15;
    double half_h = 0.15;
    double cross_w = 0.05;

    // Left vertical
    p.x = msg->pose.position.x - half_w; p.y = msg->pose.position.y - half_h; p.z = cz; h_marker.points.push_back(p);
    p.x = msg->pose.position.x - half_w; p.y = msg->pose.position.y + half_h; p.z = cz; h_marker.points.push_back(p);

    // Right vertical
    p.x = msg->pose.position.x + half_w; p.y = msg->pose.position.y - half_h; p.z = cz; h_marker.points.push_back(p);
    p.x = msg->pose.position.x + half_w; p.y = msg->pose.position.y + half_h; p.z = cz; h_marker.points.push_back(p);

    // Horizontal
    p.x = msg->pose.position.x - cross_w; p.y = msg->pose.position.y; p.z = cz; h_marker.points.push_back(p);
    p.x = msg->pose.position.x + cross_w; p.y = msg->pose.position.y; p.z = cz; h_marker.points.push_back(p);

    marker_pub_->publish(h_marker);
  }

  void checkVisibility()
  {
    auto now = this->now();
    double elapsed = (now - last_seen_time_).seconds();

    bool currently_visible = elapsed < timeout_sec_;
    if (currently_visible != target_visible_) {
      target_visible_ = currently_visible;
      // RCLCPP_INFO(get_logger(), "Target visibility changed: %s",
      //             target_visible_ ? "TRUE" : "FALSE");
    }

    // Publish current state *every timer tick*
    std_msgs::msg::Bool status_msg;
    status_msg.data = currently_visible;
    target_status_pub_->publish(status_msg);
  }

  // Publishers/Subscribers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  // Tracking variables
  rclcpp::Time last_seen_time_;
  bool target_visible_;
  double timeout_sec_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerVisualizer>());
  rclcpp::shutdown();
  return 0;
}
