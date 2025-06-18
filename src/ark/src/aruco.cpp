#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class WebcamPublisher : public rclcpp::Node
{
public:
    WebcamPublisher() : Node("webcam_publisher")
    {
        // Initialize camera matrix with your values
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            956.03474061, 0.0, 303.35197101,
            0.0, 954.97037164, 242.94739056,
            0.0, 0.0, 1.0);
        
        // Assume zero distortion for simplicity
        dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);

        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

        // Initialize camera
        cap_.open(0); // Open default camera
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open webcam!");
            rclcpp::shutdown();
            return;
        }

        // Set camera resolution (adjust if needed)
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // Create timer for publishing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 fps
            std::bind(&WebcamPublisher::publish_frame, this));

        RCLCPP_INFO(this->get_logger(), "Webcam publisher started with horizontal flip");
    }

private:
    void publish_frame()
    {
        cv::Mat flipped_frame;
        cap_ >> flipped_frame; // Capture frame from webcam
        if (flipped_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }
        
        // Flip the image horizontally (mirror effect)
        //cv::Mat flipped_frame;
        //cv::flip(frame, flipped_frame, 1); // 1 means horizontal flip
        
        // Convert OpenCV image to ROS message
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), 
            "bgr8", 
            flipped_frame
        ).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";

        // Publish flipped image
        image_pub_->publish(*msg);

        // Create and publish camera info
        auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
        camera_info->header = msg->header;
        camera_info->height = flipped_frame.rows;
        camera_info->width = flipped_frame.cols;
        
        // Set camera matrix
        camera_info->k = {
            camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
            camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
            camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2)
        };

        // Set distortion coefficients
        camera_info->d = {
            dist_coeffs_.at<double>(0),
            dist_coeffs_.at<double>(1),
            dist_coeffs_.at<double>(2),
            dist_coeffs_.at<double>(3)
        };

        // Set rectification and projection matrices (identity for now)
        camera_info->r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        camera_info->p = {
            camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2), 0,
            camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2), 0,
            camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2), 0
        };

        camera_info_pub_->publish(std::move(camera_info));

        // Optional: Show the flipped image in a window for debugging
        //cv::imshow("Flipped Webcam Output", flipped_frame);
        //cv::waitKey(1);
    }

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Webcam capture
    cv::VideoCapture cap_;

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamPublisher>();
    rclcpp::spin(node);
    
    // Cleanup
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
