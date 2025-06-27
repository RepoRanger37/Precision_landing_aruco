#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <fcntl.h>
#include <unistd.h>

class Buffer : public rclcpp::Node {
public:
    Buffer() : Node("buffer"), count(0) {
        // Get framebuffer resolution from system
        width_ = 704;   // Match fbset output
        height_ = 432;  // Match fbset output
        
        buffer_sub = create_subscription<sensor_msgs::msg::Image>(
            "/aruco_detection", 10, std::bind(&Buffer::image_Callback, this, std::placeholders::_1)
        );
    }

private:
    void image_Callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        } 
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat image = cv_ptr->image;
        
        // Resize to framebuffer dimensions
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(width_, height_));
        
       // Add alpha channel (required by framebuffer)
        cv::Mat frame;
        cv::cvtColor(resized, frame, cv::COLOR_BGR2BGRA);

        int fb = open("/dev/fb0", O_RDWR);
        if (fb < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open framebuffer device");
            return;
        }
        
        // Calculate exact bytes to write
        size_t bytes_to_write = width_ * height_ * 4;
        ssize_t written = write(fb, frame.data, bytes_to_write);
        
        if (written != static_cast<ssize_t>(bytes_to_write)) {
            RCLCPP_ERROR(this->get_logger(), "Partial write: %zd/%zu bytes", written, bytes_to_write);
        }
        
        close(fb);
        
        count = (count + 1) % 10;
       // RCLCPP_INFO(this->get_logger(), "Wrote %dx%d image %d", width_, height_, count);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr buffer_sub;
    int width_;
    int height_;
    int count;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Buffer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

