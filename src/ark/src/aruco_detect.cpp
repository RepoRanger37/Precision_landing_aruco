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
    image_sub_ = image_transport::create_subscription(
      this, "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image",
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
      "raw");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info", 10,
      std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    image_pub_ = image_transport::create_publisher(this, "/image_proc");
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250));
    marker_length_ = 0.5;
    camera_info_received_ = false;
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_info_received_) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
      dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
      for (size_t i = 0; i < msg->d.size(); i++) {
        dist_coeffs_.at<double>(i) = msg->d[i];
      }
      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera info received");
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for camera info...");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    cv::Vec3d current_tvec(0, 0, 0); // To store the position of marker ID 0

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, params, rejected);

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(image, corners, ids);

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        // Draw axis for each detected marker
        cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_length_ * 0.5f);

        if (ids[i] == 0) {
          current_tvec = tvecs[i];
          
          // Convert rotation vector to quaternion
          cv::Mat R;
          cv::Rodrigues(rvecs[i], R);
          tf2::Matrix3x3 tf2_r(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
          tf2::Quaternion q;
          tf2_r.getRotation(q);

          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header = msg->header;
          pose_msg.pose.position.x = tvecs[i][0];
          pose_msg.pose.position.y = tvecs[i][1];
          pose_msg.pose.position.z = tvecs[i][2];
          pose_msg.pose.orientation.x = q.x();
          pose_msg.pose.orientation.y = q.y();
          pose_msg.pose.orientation.z = q.z();
          pose_msg.pose.orientation.w = q.w();
          pose_pub_->publish(pose_msg);
        }
      }
    }

    // Display XYZ coordinates in bottom-right corner with colored text
    int baseLine = 0;
    double fontScale = 1.2;
    int thickness = 3;
    int margin = 20;
    int lineHeight = 40;

    // Get the image dimensions
    int imageHeight = image.rows;
    int imageWidth = image.cols;

    // Create strings for each coordinate
    std::string xText = "X: " + std::to_string(current_tvec[0]).substr(0, 5);
    std::string yText = "Y: " + std::to_string(current_tvec[1]).substr(0, 5);
    std::string zText = "Z: " + std::to_string(current_tvec[2]).substr(0, 5);

    // Calculate text positions (bottom-right corner)
    cv::Point xPos(imageWidth - margin - 150, imageHeight - margin - lineHeight * 2);
    cv::Point yPos(imageWidth - margin - 150, imageHeight - margin - lineHeight);
    cv::Point zPos(imageWidth - margin - 150, imageHeight - margin);

    // Draw the text with appropriate colors
    cv::putText(image, xText, xPos, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 255), thickness); // Red
    cv::putText(image, yText, yPos, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 255, 0), thickness); // Green
    cv::putText(image, zText, zPos, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 0, 0), thickness); // Blue

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = "bgr8";
    out_msg.image = image;
    image_pub_.publish(out_msg.toImageMsg());
  }

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_length_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
