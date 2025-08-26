#include <memory>
#include <string>
#include <vector>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ArucoDetectorNode : public rclcpp::Node {
public:
  ArucoDetectorNode() : Node("aruco_detector_node") {
    declare_parameter<double>("marker_length", 0.256);
    declare_parameter<std::string>("camera_topic", "/camera");
    declare_parameter<std::string>("camera_info_topic", "/camera_info");
    declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_250);

    marker_length_ = get_parameter("marker_length").as_double();
    std::string camera_topic = get_parameter("camera_topic").as_string();
    std::string camera_info_topic = get_parameter("camera_info_topic").as_string();
    int dictionary_id = get_parameter("dictionary_id").as_int();

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

    armed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/land_armed_status", 10, std::bind(&ArucoDetectorNode::armedCallback, this, std::placeholders::_1));
    disarmed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/land_disarmed_status", 10, std::bind(&ArucoDetectorNode::disarmedCallback, this, std::placeholders::_1));

    image_pub_ = image_transport::create_publisher(this, "/aruco_detection");
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    detector_params_ = cv::aruco::DetectorParameters::create();

    camera_info_received_ = false;
    recording_active_ = false;
    rf_distance_ = 0.0;
    rf_distance_received_ = false;

    RCLCPP_INFO(get_logger(), "Aruco detector node initialized");
  }

private:
  // ROS
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rf_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarmed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  double marker_length_;
  cv::Mat camera_matrix_, dist_coeffs_;
  bool camera_info_received_;

  // Landing status
  bool landing_status_;
  bool recording_active_;
  rclcpp::Time start_time_;

  // RF distance
  float rf_distance_;
  bool rf_distance_received_;

  // Armed/disarmed state
  bool armed_{false};
  bool disarmed_{false};

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

  void armedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    armed_ = msg->data;
  }

  void disarmedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    disarmed_ = msg->data;
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

    // Detect markers
    cv::Vec3d current_tvec(0, 0, 0);
    bool marker_detected = false;
    float error_percentage = 0.0f;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(image, corners, ids);

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_length_ * 0.5f);

        if (ids[i] == 0) {
          marker_detected = true;
          current_tvec = tvecs[i];

          // Compute error
          float z_distance = static_cast<float>(current_tvec[2]);  // meters
          if (rf_distance_received_ && rf_distance_ > 0.0f) {
            float error = rf_distance_ - z_distance;
            error_percentage = (error / rf_distance_) * 100.0f;
          }

          cv::Mat rot;
          cv::Rodrigues(rvecs[i], rot);
          tf2::Matrix3x3 m(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                           rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                           rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
          tf2::Quaternion q;
          m.getRotation(q);

          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header = msg->header;
          pose_msg.pose.position.x = current_tvec[0];
          pose_msg.pose.position.y = current_tvec[1];
          pose_msg.pose.position.z = current_tvec[2];
          pose_msg.pose.orientation.x = q.x();
          pose_msg.pose.orientation.y = q.y();
          pose_msg.pose.orientation.z = q.z();
          pose_msg.pose.orientation.w = q.w();
          pose_pub_->publish(pose_msg);
        }
      }
    }

    drawCoordinates(image, current_tvec, marker_detected, error_percentage);
    drawOverlayText(image);
    drawCrosshair(image);

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;
    image_pub_.publish(out_msg.toImageMsg());
  }

  void drawCoordinates(cv::Mat &image, const cv::Vec3d &tvec, bool detected, float error_percent) {
    const double font_scale = 0.7;
    const int thickness = 2;
    const int line_height = 30;
    const cv::Scalar color = detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "X: " << tvec[0] << " m";
    std::string x = oss.str();
    oss.str(""); oss << "Y: " << tvec[1] << " m";
    std::string y = oss.str();
    oss.str(""); oss << "Z: " << tvec[2] << " m";
    std::string z = oss.str();

    int x_pos = image.cols - 220;
    int y_pos = image.rows - 3 * line_height;

    cv::putText(image, x, cv::Point(x_pos, y_pos), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    cv::putText(image, y, cv::Point(x_pos, y_pos + line_height), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
    cv::putText(image, z, cv::Point(x_pos, y_pos + 2 * line_height), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);

    // Draw error at bottom-left
    std::ostringstream error_ss;
    error_ss << std::fixed << std::setprecision(2);
    error_ss << "Error: " << (detected && rf_distance_received_ ? error_percent : 0.0f) << " %";
    std::string error_text = error_ss.str();
    cv::putText(image, error_text, cv::Point(20, image.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
  }

  void drawOverlayText(cv::Mat &image) {
    const double font_scale = 0.7;
    const int thickness = 2;
    const cv::Scalar color(255, 255, 255);

    // Top-center status
    std::string status_text = landing_status_ ? "Landing" : "Searching";
    int status_width = cv::getTextSize(status_text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, 0).width;
    cv::putText(image, status_text, cv::Point((image.cols - status_width) / 2, 30), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);

    // Top-left rec
    std::string rec_text;
    if (recording_active_) {
      int elapsed = (now() - start_time_).seconds();
      int mm = elapsed / 60;
      int ss = elapsed % 60;
      std::ostringstream oss;
      oss << "Rec [" << std::setw(2) << std::setfill('0') << mm << ":" << std::setw(2) << ss << "]";
      rec_text = oss.str();
    } else {
      rec_text = "Not Rec";
    }
    cv::putText(image, rec_text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);

    // --- Draw ARMED, DISARMED, LAND vertically at bottom-left above ERROR ---
    int base_x = 20;
    int error_y = image.rows - 10; // ERROR text y
    int line_height = 38; // vertical spacing
    int dot_radius = 12;
    int font_thickness = 2;
    double font_scale_overlay = 0.7;

    // ARMED
    cv::Scalar armed_dot = armed_ ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
    std::string armed_text = "ARMED";
    int armed_y = error_y - line_height * 1;
    cv::putText(image, armed_text, cv::Point(base_x + 30, armed_y), cv::FONT_HERSHEY_SIMPLEX, font_scale_overlay, cv::Scalar(255,255,255), font_thickness);
    cv::circle(image, cv::Point(base_x + 10, armed_y - 10), dot_radius, armed_dot, -1);

    // DISARMED
    cv::Scalar disarmed_dot = disarmed_ ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
    std::string disarmed_text = "DISARMED";
    int disarmed_y = error_y - line_height * 2;
    cv::putText(image, disarmed_text, cv::Point(base_x + 30, disarmed_y), cv::FONT_HERSHEY_SIMPLEX, font_scale_overlay, cv::Scalar(255,255,255), font_thickness);
    cv::circle(image, cv::Point(base_x + 10, disarmed_y - 10), dot_radius, disarmed_dot, -1);

    // LAND
    bool any_true = armed_ || disarmed_;
    cv::Scalar land_color = any_true ? cv::Scalar(255,0,0) : cv::Scalar(255,255,255); // Blue if any true, else white
    std::string land_text = "LAND";
    int land_y = error_y - line_height * 3;
    cv::putText(image, land_text, cv::Point(base_x + 30, land_y), cv::FONT_HERSHEY_SIMPLEX, font_scale_overlay, land_color, font_thickness);
    cv::circle(image, cv::Point(base_x + 10, land_y - 10), dot_radius, land_color, -1);
  }

  void drawCrosshair(cv::Mat &image) {
    cv::Point center(image.cols / 2, image.rows / 2);
    int square_size = 80;
    int corner_len = 15;
    int thickness = 2;
    cv::Scalar color(0, 0, 255);

    int left = center.x - square_size / 2;
    int right = center.x + square_size / 2;
    int top = center.y - square_size / 2;
    int bottom = center.y + square_size / 2;

    // Corners
    cv::line(image, {left, top}, {left + corner_len, top}, color, thickness);
    cv::line(image, {left, top}, {left, top + corner_len}, color, thickness);
    cv::line(image, {right, top}, {right - corner_len, top}, color, thickness);
    cv::line(image, {right, top}, {right, top + corner_len}, color, thickness);
    cv::line(image, {left, bottom}, {left + corner_len, bottom}, color, thickness);
    cv::line(image, {left, bottom}, {left, bottom - corner_len}, color, thickness);
    cv::line(image, {right, bottom}, {right - corner_len, bottom}, color, thickness);
    cv::line(image, {right, bottom}, {right, bottom - corner_len}, color, thickness);

    // Central plus
    int plus_len = 10;
    cv::line(image, {center.x - plus_len, center.y}, {center.x + plus_len, center.y}, color, thickness);
    cv::line(image, {center.x, center.y - plus_len}, {center.x, center.y + plus_len}, color, thickness);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

