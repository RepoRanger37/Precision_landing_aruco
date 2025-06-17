#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <fcntl.h>
#include <unistd.h>


class Buffer : public rclcpp::Node {
public:
    Buffer()
        : Node("buffer"), count(0) {
        buffer_sub = create_subscription<sensor_msgs::msg::Image>(
            "/image_proc", 10, std::bind(&Buffer::image_Callback, this, std::placeholders::_1)
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
            return ;
        }
        cv::Mat image = cv_ptr->image;
        cv::Mat frame = cv::Mat(900, 1600, CV_8UC4);
        cv::cvtColor(image, frame, cv::COLOR_BGR2BGRA);

        int fb = open("/dev/fb0", O_RDWR);
        if (fb < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open framebuffer device");
            return;
        }else {
        write(fb, frame.data, 900*1600*4);
        close(fb);
        
        count = (count + 1)%10;
        RCLCPP_INFO(this->get_logger(), "Wrote Image data %d", count);
         return; 
         }
         }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr buffer_sub;
        int count;
    
  };
    int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Buffer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}