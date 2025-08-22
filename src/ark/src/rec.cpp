#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <regex>

class VideoRecorder : public rclcpp::Node
{
public:
    VideoRecorder()
    : Node("video_recorder_node")
    {
        using std::placeholders::_1;

        // Create folder if it doesn't exist
        std::filesystem::create_directories("/home/pi/Video");

        // Generate unique filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << "/home/pi/Video/video_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".mp4";
        output_filename_ = filename.str();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10, std::bind(&VideoRecorder::image_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Video recording started: %s", output_filename_.c_str());
    }

    ~VideoRecorder()
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Video recording stopped and saved.");
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (!video_writer_.isOpened()) {
            // Initialize video writer once image size is known
             int width = cv_ptr->image.cols;
             int height = cv_ptr->image.rows;
            int fps = 10;  // You can set this based on your camera/topic frequency
            video_writer_.open(output_filename_, cv::VideoWriter::fourcc('m','p','4','v'), fps, cv::Size(width, height));
            if (!video_writer_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open the output video file for write.");
                return;
            }
        }

        video_writer_.write(cv_ptr->image);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    std::string output_filename_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRecorder>());
    rclcpp::shutdown();
    return 0;
}
