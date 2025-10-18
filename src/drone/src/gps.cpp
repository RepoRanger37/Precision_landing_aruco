#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class GpsPublisherNode : public rclcpp::Node {
public:
    GpsPublisherNode()
    : Node("gps_publisher_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GpsPublisherNode::publish_gps_data, this));
    }

private:
    void publish_gps_data()
    {
        auto msg = sensor_msgs::msg::NavSatFix();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "gps";

        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        msg.latitude = 12.8481131;
        msg.longitude = 77.6458091;
        msg.altitude = 900.0;  // Dummy altitude

        // Covariance can be left as unknown if just for testing
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        RCLCPP_INFO(this->get_logger(), "Publishing GPS fix: [%.7f, %.7f]", msg.latitude, msg.longitude);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
