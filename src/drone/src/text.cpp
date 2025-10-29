#include <rclcpp/rclcpp.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

class FlightOverlayNode : public rclcpp::Node {
public:
    FlightOverlayNode() : Node("flight_overlay_node") {
        // --- Overlay publishers ---
        pub_gps_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/gps_text", 1);
        pub_alt_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/alt_text", 1);
        pub_batt_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/batt_text", 1);
        pub_heading_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/heading_text", 1);
        pub_mode_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/mode_text", 1);
        pub_target_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/target_text", 1);
        pub_armed_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/armed_text", 1);
        pub_fence_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("/overlay/fence_text", 1);

        // --- Subscriptions ---
        alt_sub_ = this->create_subscription<std_msgs::msg::Float32>("/altitude", 10,
            std::bind(&FlightOverlayNode::alt_callback, this, std::placeholders::_1));
        gps_sat_sub_ = this->create_subscription<std_msgs::msg::Float32>("/gps_satellites", 10,
            std::bind(&FlightOverlayNode::gps_sat_callback, this, std::placeholders::_1));
        gps_fix_sub_ = this->create_subscription<std_msgs::msg::String>("/gps_fix_type", 10,
            std::bind(&FlightOverlayNode::gps_fix_callback, this, std::placeholders::_1));
        batt_v_sub_ = this->create_subscription<std_msgs::msg::Float32>("/battery_voltage", 10,
            std::bind(&FlightOverlayNode::batt_v_callback, this, std::placeholders::_1));
        batt_i_sub_ = this->create_subscription<std_msgs::msg::Float32>("/battery_current", 10,
            std::bind(&FlightOverlayNode::batt_i_callback, this, std::placeholders::_1));
        heading_sub_ = this->create_subscription<std_msgs::msg::Float32>("/drone/heading", 10,
            std::bind(&FlightOverlayNode::heading_callback, this, std::placeholders::_1));
        mode_sub_ = this->create_subscription<std_msgs::msg::String>("/flight_mode", 10,
            std::bind(&FlightOverlayNode::mode_callback, this, std::placeholders::_1));
        armed_sub_ = this->create_subscription<std_msgs::msg::Bool>("/armed_status", 10,
            std::bind(&FlightOverlayNode::armed_callback, this, std::placeholders::_1));
        fence_sub_ = this->create_subscription<std_msgs::msg::Bool>("/fence_enabled", 10,
            std::bind(&FlightOverlayNode::fence_callback, this, std::placeholders::_1));
        target_status_sub_ = this->create_subscription<std_msgs::msg::Bool>("/target_visible", 10,
            std::bind(&FlightOverlayNode::target_status_callback, this, std::placeholders::_1));

        // --- Timer to periodically update overlays ---
        timer_ = this->create_wall_timer(200ms, std::bind(&FlightOverlayNode::publish_overlay, this));

        RCLCPP_INFO(this->get_logger(), "FlightOverlayNode started (overlay only).");
    }

private:
    // --- State variables ---
    bool armed_ = false;
    bool fence_enabled_ = false;
    bool target_found_ = false;

    float altitude_ = 0.0;
    float gps_sats_ = 0.0;
    float batt_v_ = 0.0;
    float batt_i_ = 0.0;
    float heading_ = 0.0;
    std::string gps_fix_ = "No Fix";
    std::string flight_mode_ = "STABILIZE";

    // --- Overlay Publishers ---
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr
        pub_gps_, pub_alt_, pub_batt_, pub_heading_, pub_mode_, pub_target_,
        pub_armed_, pub_fence_;

    // --- Subscribers ---
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        alt_sub_, gps_sat_sub_, batt_v_sub_, batt_i_sub_, heading_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        gps_fix_sub_, mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        armed_sub_, fence_sub_, target_status_sub_;

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callbacks ---
    void alt_callback(const std_msgs::msg::Float32::SharedPtr msg) { altitude_ = msg->data; }
    void gps_sat_callback(const std_msgs::msg::Float32::SharedPtr msg) { gps_sats_ = msg->data; }
    void gps_fix_callback(const std_msgs::msg::String::SharedPtr msg) { gps_fix_ = msg->data; }
    void batt_v_callback(const std_msgs::msg::Float32::SharedPtr msg) { batt_v_ = msg->data; }
    void batt_i_callback(const std_msgs::msg::Float32::SharedPtr msg) { batt_i_ = msg->data; }
    void heading_callback(const std_msgs::msg::Float32::SharedPtr msg) { heading_ = msg->data; }
    void mode_callback(const std_msgs::msg::String::SharedPtr msg) { flight_mode_ = msg->data; }
    void armed_callback(const std_msgs::msg::Bool::SharedPtr msg) { armed_ = msg->data; }
    void fence_callback(const std_msgs::msg::Bool::SharedPtr msg) { fence_enabled_ = msg->data; }
    void target_status_callback(const std_msgs::msg::Bool::SharedPtr msg) { target_found_ = msg->data; }

    // --- Overlay Helper ---
    rviz_2d_overlay_msgs::msg::OverlayText make_text(
        const std::string &text, int x, int y,
        int align_h, int align_v,
        float size = 14.0, int width = 250, int height = 35)
    {
        rviz_2d_overlay_msgs::msg::OverlayText msg;
        msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
        msg.text = text;
        msg.width = width;
        msg.height = height;
        msg.text_size = size;
        msg.font = "DejaVu Sans";
        msg.line_width = 1;
        msg.horizontal_alignment = align_h;
        msg.vertical_alignment = align_v;
        msg.horizontal_distance = x;
        msg.vertical_distance = y;
        msg.fg_color.r = 1.0;
        msg.fg_color.g = 1.0;
        msg.fg_color.b = 1.0;
        msg.fg_color.a = 1.0;
        msg.bg_color.r = 0.0;
        msg.bg_color.g = 0.0;
        msg.bg_color.b = 0.0;
        msg.bg_color.a = 0.4;
        return msg;
    }

    // --- Convert heading to 8-direction quadrant ---
    std::string heading_to_quadrant(float deg) {
        deg = fmod(deg, 360.0f);
        if (deg < 0) deg += 360.0f;

        std::string direction;
        if (deg >= 337.5 || deg < 22.5) direction = "N";
        else if (deg >= 22.5 && deg < 67.5) direction = "NE";
        else if (deg >= 67.5 && deg < 112.5) direction = "E";
        else if (deg >= 112.5 && deg < 157.5) direction = "SE";
        else if (deg >= 157.5 && deg < 202.5) direction = "S";
        else if (deg >= 202.5 && deg < 247.5) direction = "SW";
        else if (deg >= 247.5 && deg < 292.5) direction = "W";
        else if (deg >= 292.5 && deg < 337.5) direction = "NW";

        return std::to_string((int)deg) + "° " + direction;
    }

    // --- Periodic Overlay Updates ---
    void publish_overlay() {
        std::ostringstream ss;

        // GPS
        ss << "GPS: " << gps_fix_ << "  Sats: " << (int)gps_sats_;
        pub_gps_->publish(make_text(ss.str(), 10, 10,
            rviz_2d_overlay_msgs::msg::OverlayText::LEFT,
            rviz_2d_overlay_msgs::msg::OverlayText::TOP));

        // Altitude
        ss.str(""); ss << "Alt: " << std::fixed << std::setprecision(2) << altitude_ << " m";
        pub_alt_->publish(make_text(ss.str(), 10, 250,
            rviz_2d_overlay_msgs::msg::OverlayText::LEFT,
            rviz_2d_overlay_msgs::msg::OverlayText::CENTER));

        // Battery
        ss.str(""); ss << "Batt: " << std::fixed << std::setprecision(2) << batt_v_ << "V  " << batt_i_ << "A";
        pub_batt_->publish(make_text(ss.str(), 10, 10,
            rviz_2d_overlay_msgs::msg::OverlayText::RIGHT,
            rviz_2d_overlay_msgs::msg::OverlayText::TOP));

        // Heading
        ss.str(""); ss << "HDG: " << heading_to_quadrant(heading_);
        pub_heading_->publish(make_text(ss.str(), 0, 10,
            rviz_2d_overlay_msgs::msg::OverlayText::CENTER,
            rviz_2d_overlay_msgs::msg::OverlayText::TOP));

        // Armed status overlay
        ss.str(""); ss << (armed_ ? "ARMED" : "DISARMED");
        pub_armed_->publish(make_text(ss.str(), 10, 60,
            rviz_2d_overlay_msgs::msg::OverlayText::LEFT,
            rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM, 14.0, 200, 30));

        // Fence status overlay
        ss.str(""); ss << "Fence: " << (fence_enabled_ ? "Y" : "N");
        pub_fence_->publish(make_text(ss.str(), 10, 100,
            rviz_2d_overlay_msgs::msg::OverlayText::LEFT,
            rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM, 14.0, 200, 30));

        // Flight mode
        ss.str(""); ss << flight_mode_;
        pub_mode_->publish(make_text(ss.str(), 0, 60,
            rviz_2d_overlay_msgs::msg::OverlayText::CENTER,
            rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM,
            16.0, 300, 50));

        // Target found overlay
        if (target_found_) {
            ss.str(""); ss << "FOUND THE BASE";
            pub_target_->publish(make_text(ss.str(), 0, 40,
                rviz_2d_overlay_msgs::msg::OverlayText::CENTER,
                rviz_2d_overlay_msgs::msg::OverlayText::TOP,
                16.0, 300, 50));
        } else {
            // Clear target overlay if not found
            rviz_2d_overlay_msgs::msg::OverlayText clear_msg;
            clear_msg.action = rviz_2d_overlay_msgs::msg::OverlayText::DELETE;
            pub_target_->publish(clear_msg);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
