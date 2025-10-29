#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class ArucoDetectorNode : public rclcpp::Node {
public:
  ArucoDetectorNode() : Node("aruco_detector_node") {
    declare_parameter<std::string>("camera_topic", "/camera");
    declare_parameter<std::string>("camera_info_topic", "/camera_info");
    declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_250);
    declare_parameter<std::string>("target_frame", "base_link");

    std::string camera_topic = get_parameter("camera_topic").as_string();
    std::string camera_info_topic = get_parameter("camera_info_topic").as_string();
    int dictionary_id = get_parameter("dictionary_id").as_int();
    target_frame_ = get_parameter("target_frame").as_string();

    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_sizes_ = {
      {0, 0.25},  // Marker ID 0 → 0.256m
      {1, 0.1}     // Marker ID 1 → 0.1m
    };

    image_sub_ = image_transport::create_subscription(
      this, camera_topic,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
      "raw");

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/landing_status", 10,
      std::bind(&ArucoDetectorNode::statusCallback, this, std::placeholders::_1));

    rf_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/rf_distance", 10,
      std::bind(&ArucoDetectorNode::rfDistanceCallback, this, std::placeholders::_1));

    flight_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/flight_mode", 10,
      std::bind(&ArucoDetectorNode::flightModeCallback, this, std::placeholders::_1));

    armed_status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/armed_status", 10,
      std::bind(&ArucoDetectorNode::armedStatusCallback, this, std::placeholders::_1));

    recording_status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/recording_status", 10,
      std::bind(&ArucoDetectorNode::recordingStatusCallback, this, std::placeholders::_1));

    fence_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
  "/fence_enabled", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
    fence_enabled_ = msg->data;
    });

    gyro_ok_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/gyro_ok", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gyro_ok_ = msg->data;
      });

    magfield_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/magfield", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        magfield_ = msg->data;
      });

    gps_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/gps_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        gps_status_ = msg->data;
      });

    gps_satellites_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/gps_satellites", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        gps_satellites_ = static_cast<int>(msg->data);
      });

    battery_voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_voltage", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        battery_voltage_ = msg->data;
      });

    battery_current_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_current", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        battery_current_ = msg->data;
      });


    image_pub_ = image_transport::create_publisher(this, "/aruco_detection");
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    detector_params_ = cv::aruco::DetectorParameters::create();

    camera_info_received_ = false;
    recording_active_ = false;
    rf_distance_ = 0.0;
    rf_distance_received_ = false;

    armed_ = false;
    flight_mode_ = "UNKNOWN";

    RCLCPP_INFO(get_logger(), "Aruco detector node initialized");

    // --- Add these for full-screen OpenCV window ---
    cv::namedWindow("ARuco View", cv::WINDOW_NORMAL);
  }

private:
  // ROS
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rf_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_status_sub_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Fence and system diagnostics
  bool fence_enabled_ = false;
  bool gyro_ok_ = false;
  float magfield_ = 0.0f;
  std::string gps_status_ = "UNKNOWN";
  int gps_satellites_ = 0;
  float battery_voltage_ = 0.0f;
  float battery_current_ = 0.0f;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fence_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gyro_ok_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr magfield_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gps_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gps_satellites_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_current_sub_;


  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::map<int, double> marker_sizes_;
  cv::Mat camera_matrix_, dist_coeffs_;
  bool camera_info_received_;

  // Status
  bool landing_status_;
  bool recording_active_;
  bool is_recording_ = false;
  rclcpp::Time recording_start_time_;
  rclcpp::Time start_time_;

  float rf_distance_;
  bool rf_distance_received_;

  bool armed_;
  std::string flight_mode_;
  std::string target_frame_;


  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!camera_info_received_) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 9; ++i)
        camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];

      dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F);
      for (size_t i = 0; i < msg->d.size(); ++i)
        dist_coeffs_.at<double>(i) = msg->d[i];

      camera_info_received_ = true;
      RCLCPP_INFO(get_logger(), "Camera calibration parameters received");
    }
  }

  void statusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    landing_status_ = msg->data;
    if (landing_status_ && !recording_active_) {
      recording_active_ = true;
      start_time_ = now();
    } else if (!landing_status_) {
      recording_active_ = false;
    }
  }

  void rfDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    rf_distance_ = msg->data;
    rf_distance_received_ = true;
  }

  void flightModeCallback(const std_msgs::msg::String::SharedPtr msg) {
    flight_mode_ = msg->data;
  }

  void armedStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    armed_ = msg->data;
  }

  void recordingStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !is_recording_) {
      is_recording_ = true;
      recording_start_time_ = now();
    } else if (!msg->data) {
      is_recording_ = false;
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera calibration data...");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    auto &image = cv_ptr->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

    bool marker_detected = false;
    float error_percentage = 0.0f;
    cv::Vec3d current_tvec(0, 0, 0);
    int current_marker_id = -1;

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(image, corners, ids);

      for (size_t i = 0; i < ids.size(); ++i) {
        int marker_id = ids[i];

        if (marker_sizes_.find(marker_id) == marker_sizes_.end())
          continue;

        double marker_length = marker_sizes_[marker_id];
        std::vector<cv::Vec3d> rvecs(1), tvecs(1);
        cv::aruco::estimatePoseSingleMarkers(
          std::vector<std::vector<cv::Point2f>>{corners[i]},
          marker_length, camera_matrix_, dist_coeffs_, rvecs, tvecs
        );

        cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[0], tvecs[0], marker_length * 0.5f);

        bool use_this_marker = false;

        // Corrected logic: ID 0 → distance > 2.0, ID 1 → distance ≤ 2.0
        if (marker_id == 0 && rf_distance_ > 3.0f) {
          use_this_marker = true;
        } else if (marker_id == 1 && rf_distance_ <= 3.0f) {
          use_this_marker = true;
        }

        if (use_this_marker) {
          marker_detected = true;
          current_tvec = tvecs[0];
          current_marker_id = marker_id;

          float z_distance = static_cast<float>(current_tvec[2]);
          if (rf_distance_received_ && rf_distance_ > 0.0f) {
            float error = rf_distance_ - z_distance;
            error_percentage = (error / rf_distance_) * 100.0f;
          }

          cv::Mat rot;
          cv::Rodrigues(rvecs[0], rot);
          tf2::Matrix3x3 m(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                           rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                           rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
          tf2::Quaternion q;
          m.getRotation(q);

          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header.stamp = this->now();
          pose_msg.header.frame_id = "cam_down_link";
          pose_msg.pose.position.x = current_tvec[0];
          pose_msg.pose.position.y = current_tvec[1];
          pose_msg.pose.position.z = current_tvec[2];
          pose_msg.pose.orientation.x = q.x();
          pose_msg.pose.orientation.y = q.y();
          pose_msg.pose.orientation.z = q.z();
          pose_msg.pose.orientation.w = q.w();
          pose_pub_->publish(pose_msg);

          // Log for debug
        //   RCLCPP_INFO(get_logger(), "Published pose of marker ID %d at distance %.2f m", marker_id, rf_distance_);
        //   break;  // Publish only for first suitable marker
        }
      }
    }

    drawCoordinates(image, current_tvec, marker_detected, error_percentage, current_marker_id);
    drawOverlayText(image);
    drawSearchingText(image, marker_detected);
    drawCrosshair(image);

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;
    image_pub_.publish(out_msg.toImageMsg());

    cv::imshow("ARuco View", image);
    cv::resizeWindow("ARuco View", image.cols, image.rows);
    cv::waitKey(1);
  }

  void drawCoordinates(cv::Mat &image, const cv::Vec3d &tvec, bool detected, float error_percent, int marker_id) {
    if (!detected)
      return;

    const double font_scale = 0.7;
    const int thickness = 2;
    const int line_height = 30;
    const cv::Scalar color = cv::Scalar(0, 255, 0); // green for detected

    // Start from bottom-right corner with some margin
    int margin_x = 20;
    int margin_y = 20;
    int x_pos = image.cols - margin_x;
    int y_pos = image.rows - margin_y;

    std::ostringstream oss;

    // Show marker ID on top
    oss << "ID: " << marker_id;
    std::string id_text = oss.str();
    oss.str("");

    // Show X, Y, Z below ID
    oss << std::fixed << std::setprecision(3);
    oss << "X: " << tvec[0] << " m";
    std::string x_text = oss.str();
    oss.str("");
    oss << "Y: " << tvec[1] << " m";
    std::string y_text = oss.str();
    oss.str("");
    oss << "Z: " << tvec[2] << " m";
    std::string z_text = oss.str();
    oss.str("");

    // Show error % below coords if rf_distance received
    std::string error_text;
    if (rf_distance_received_) {
      std::ostringstream error_ss;
      error_ss << std::fixed << std::setprecision(2);
      error_ss << "Error: " << error_percent << " %";
      error_text = error_ss.str();
    }

    // Compute text sizes to right-align
    int baseline = 0;
    int id_width = cv::getTextSize(id_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
    int x_width = cv::getTextSize(x_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
    int y_width = cv::getTextSize(y_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
    int z_width = cv::getTextSize(z_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
    int error_width = cv::getTextSize(error_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;

    int max_width = std::max({id_width, x_width, y_width, z_width, error_width});

    // Vertical positions (bottom-up)
    int y_line0 = y_pos - 4 * line_height; // ID top line
    int y_line1 = y_pos - 3 * line_height;
    int y_line2 = y_pos - 2 * line_height;
    int y_line3 = y_pos - line_height;
    int y_line4 = y_pos;

    // Draw texts right-aligned
    cv::putText(image, id_text, cv::Point(x_pos - max_width, y_line0), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    cv::putText(image, x_text, cv::Point(x_pos - max_width, y_line1), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    cv::putText(image, y_text, cv::Point(x_pos - max_width, y_line2), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    cv::putText(image, z_text, cv::Point(x_pos - max_width, y_line3), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);

    if (!error_text.empty()) {
      cv::putText(image, error_text, cv::Point(x_pos - max_width, y_line4), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    }
  }

  void drawOverlayText(cv::Mat &image) {
  const double font_scale = 0.5;
  const int thickness = 2;

  // Colors
  const cv::Scalar blue(255, 0, 0);
  const cv::Scalar green(0, 255, 0);
  const cv::Scalar red(0, 0, 255);
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar yellow(0, 255, 255);
  const cv::Scalar orange(0, 165, 255);
  const cv::Scalar black(0, 0, 0);

  int y = 30;

  // Top-left: Recording status
  std::string rec_text;
  cv::Scalar rec_color;
  if (is_recording_) {
    auto elapsed = now() - recording_start_time_;
    int total_seconds = elapsed.seconds();
    int minutes = total_seconds / 60;
    int seconds = total_seconds % 60;
    char buf[20];
    snprintf(buf, sizeof(buf), "REC %02d:%02d", minutes, seconds);
    rec_text = buf;
    rec_color = red;
  } else {
    rec_text = "NOT REC";
    rec_color = white;
  }

  cv::putText(image, rec_text, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, rec_color, thickness);
  y += 30;

  // Keep Alt
  if (rf_distance_received_) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Keep Alt: " << rf_distance_ << " m";
    cv::putText(image, oss.str(), cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, yellow, thickness);
    y += 30;
  }

  // Fence
  cv::putText(image, "FENCE", cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, fence_enabled_ ? green : red, thickness);
  y += 30;

  // Gyro
  cv::putText(image, std::string("GYRO ") + (gyro_ok_ ? "YES" : "NO"),
              cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, gyro_ok_ ? green : red, thickness);
  y += 30;

  // Magfield
  cv::Scalar mag_color = (magfield_ > 480.0f) ? red : white;
  std::ostringstream mag_ss;
  mag_ss << "MAGFIELD: " << static_cast<int>(magfield_);
  cv::putText(image, mag_ss.str(), cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, mag_color, thickness);
  y += 30;

  // GPS
  cv::Scalar gps_color = gps_satellites_ < 5 ? red : (gps_satellites_ < 10 ? orange : green);
  std::ostringstream gps_ss;
  gps_ss << "GPS: " << gps_status_ << " Sat: " << gps_satellites_;
  cv::putText(image, gps_ss.str(), cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, gps_color, thickness);
  y += 30;

  // Battery
  std::ostringstream batt_ss;
  batt_ss << std::fixed << std::setprecision(2);
  batt_ss << "BATT V: " << battery_voltage_ << "V  I: " << battery_current_ << "A";
  cv::putText(image, batt_ss.str(), cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, font_scale, white, thickness);
  y += 30;

  // Top center: Flight Mode & Armed
  std::string armed_str = armed_ ? "ARMED" : "DISARMED";
  int baseline = 0;
  int mode_width = cv::getTextSize(flight_mode_, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
  int armed_width = cv::getTextSize(armed_str, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline).width;
  int total_width = mode_width + armed_width + 10;
  int x_center = image.cols / 2;

  cv::putText(image, flight_mode_, cv::Point(x_center - total_width/2, 30), cv::FONT_HERSHEY_SIMPLEX, font_scale, blue, thickness);
  cv::putText(image, " - ", cv::Point(x_center - total_width/2 + mode_width, 30), cv::FONT_HERSHEY_SIMPLEX, font_scale, white, thickness);
  cv::putText(image, armed_str, cv::Point(x_center - total_width/2 + mode_width + 30, 30), cv::FONT_HERSHEY_SIMPLEX, font_scale,
              armed_ ? green : red, thickness);
}


  void drawSearchingText(cv::Mat &image, bool detected) {
    if (detected)
      return;

    std::string text = "Searching";
    const double font_scale = 0.5;
    const int thickness = 2.0;
    cv::Scalar white(255, 255, 255);

    int baseline = 0;
    int text_width = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, font_scale, thickness, &baseline).width;
    int x_pos = (image.cols - text_width) / 2;
    int y_pos = image.rows - 30; // Bottom center, slight margin

    cv::putText(image, text, cv::Point(x_pos, y_pos), cv::FONT_HERSHEY_DUPLEX, font_scale, white, thickness);
  }

  void drawCrosshair(cv::Mat &image) {
    int center_x = image.cols / 2;
    int center_y = image.rows / 2;
    int size = 15;
    cv::Scalar white(255, 255, 255);
    int thickness = 2;

    cv::line(image, cv::Point(center_x - size, center_y), cv::Point(center_x + size, center_y), white, thickness);
    cv::line(image, cv::Point(center_x, center_y - size), cv::Point(center_x, center_y + size), white, thickness);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
