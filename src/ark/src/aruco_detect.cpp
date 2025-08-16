#include <memory>
#include <string>
#include <vector>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector_node")
  {
    // Initialize parameters
    this->declare_parameter<double>("marker_length", 0.26);
    this->declare_parameter<std::string>("camera_topic", "/camera");
    this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
    this->declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_250);

    marker_length_ = this->get_parameter("marker_length").as_double();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    int dictionary_id = this->get_parameter("dictionary_id").as_int();

    // Create subscribers and publishers
    image_sub_ = image_transport::create_subscription(
      this, camera_topic,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
      "raw");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    image_pub_ = image_transport::create_publisher(this, "/aruco_detection");
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

    // Initialize ArUco dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    detector_params_ = cv::aruco::DetectorParameters::create();
    
    camera_info_received_ = false;
    RCLCPP_INFO(this->get_logger(), "Aruco detector node initialized");
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_info_received_) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
      }
      
      dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F);
      for (size_t i = 0; i < msg->d.size(); ++i) {
        dist_coeffs_.at<double>(i) = msg->d[i];
      }
      
      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera calibration parameters received");
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                          "Waiting for camera calibration data...");
      return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Vec3d current_tvec(0, 0, 0);
    bool marker_detected = false;

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

    if (!ids.empty()) {
      // Draw detected markers
      cv::aruco::drawDetectedMarkers(image, corners, ids);

      // Estimate pose for each marker
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, 
                                          camera_matrix_, dist_coeffs_, 
                                          rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        // Draw axis for each marker
        cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, 
                           rvecs[i], tvecs[i], marker_length_ * 0.5f);

        if (ids[i] == 0) {  // Process marker with ID 0
          marker_detected = true;
          current_tvec = tvecs[i];
          
          // Convert rotation vector to quaternion
          cv::Mat rotation_matrix;
          cv::Rodrigues(rvecs[i], rotation_matrix);
          
          tf2::Matrix3x3 tf_rot(
            rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
            rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
            rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
          
          tf2::Quaternion tf_quat;
          tf_rot.getRotation(tf_quat);
          
          // Publish pose
          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header = msg->header;
          pose_msg.pose.position.x = tvecs[i][0];
          pose_msg.pose.position.y = tvecs[i][1];
          pose_msg.pose.position.z = tvecs[i][2];
          pose_msg.pose.orientation.x = tf_quat.x();
          pose_msg.pose.orientation.y = tf_quat.y();
          pose_msg.pose.orientation.z = tf_quat.z();
          pose_msg.pose.orientation.w = tf_quat.w();
          
          pose_pub_->publish(pose_msg);
        }
      }
    }

    // Display coordinates on image
    drawCoordinates(image, current_tvec, marker_detected);

    // Publish processed image
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;
    image_pub_.publish(out_msg.toImageMsg());
  }

  void drawCoordinates(cv::Mat &image, const cv::Vec3d &tvec, bool detected)
  {
    const double font_scale = 1.0;
    const int thickness = 2;
    const int margin = 20;
    const int line_height = 30;
    const cv::Scalar text_color = detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "X: " << tvec[0] << " m";
    std::string x_text = oss.str();
    oss.str("");
    oss << "Y: " << tvec[1] << " m";
    std::string y_text = oss.str();
    oss.str("");
    oss << "Z: " << tvec[2] << " m";
    std::string z_text = oss.str();

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(x_text, cv::FONT_HERSHEY_SIMPLEX, 
                                        font_scale, thickness, &baseline);

    cv::Point text_org(image.cols - text_size.width - margin, 
                       image.rows - margin - 2 * line_height);

    cv::putText(image, x_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 
                font_scale, text_color, thickness);
    text_org.y += line_height;
    cv::putText(image, y_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 
                font_scale, text_color, thickness);
    text_org.y += line_height;
    cv::putText(image, z_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 
                font_scale, text_color, thickness);
  }

  // ROS communication
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ArUco detection
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  double marker_length_;
  
  // Camera parameters
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
