#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <regex>
#include <chrono>
#include <iomanip>

class CameraRecorder : public rclcpp::Node
{
public:
    CameraRecorder() : Node("camera_recorder")
    {
        std::string output_folder = "/home/pi/Video";
        std::filesystem::create_directories(output_folder);

        video_output_dir_ = output_folder;
        video_index_ = get_next_video_index(video_output_dir_);
        temp_path_ = video_output_dir_ + "/temp_video.mp4";

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aruco_detection", 10,
            std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/landing_status", 10,
            std::bind(&CameraRecorder::status_callback, this, std::placeholders::_1));

        recording_active_ = false;
        writer_initialized_ = false;
        timing_started_ = false;
        frame_count_ = 0;
        landing_status_ = false;

        RCLCPP_INFO(this->get_logger(), "CameraRecorder node initialized.");
    }

private:
    // ---- Subscriptions ----
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;

    // ---- OpenCV Video ----
    cv::VideoWriter video_writer_;

    // ---- File Management ----
    std::string temp_path_;
    std::string video_output_dir_;
    int video_index_;

    // ---- State Tracking ----
    bool recording_active_;
    bool writer_initialized_;
    bool timing_started_;
    int frame_count_;
    rclcpp::Time start_time_;
    bool landing_status_;  // last received landing status

    // ---- File Naming ----
    int get_next_video_index(const std::string &folder)
    {
        namespace fs = std::filesystem;
        int max_index = 0;
        std::regex pattern(R"(\((\d+)\)_\d+_min_\d+_sec\.mp4)");

        for (const auto &p : fs::directory_iterator(folder)) {
            if (!p.is_regular_file()) continue;

            std::string filename = p.path().filename().string();
            std::smatch match;
            if (std::regex_match(filename, match, pattern)) {
                if (match.size() == 2) {
                    int num = std::stoi(match[1].str());
                    if (num > max_index) max_index = num;
                }
            }
        }

        return max_index + 1;
    }

    // ---- Landing Status Callback ----
    void status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        landing_status_ = msg->data;

        if (msg->data && !recording_active_) {
            RCLCPP_INFO(this->get_logger(), "Landing status: TRUE. Starting recording.");
            recording_active_ = true;
            writer_initialized_ = false;
            frame_count_ = 0;
            timing_started_ = false;
            temp_path_ = video_output_dir_ + "/temp_video.mp4";
            start_time_ = this->now();
        }
        else if (!msg->data && recording_active_) {
            RCLCPP_INFO(this->get_logger(), "Landing status: FALSE. Stopping and saving recording.");
            finalize_video(this->now());
            recording_active_ = false;
            writer_initialized_ = false;
            timing_started_ = false;
        }
    }

    // ---- Image Callback ----
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!recording_active_) return;

        auto now = this->now();

        if (!timing_started_) {
            start_time_ = now;
            timing_started_ = true;
        }

        frame_count_++;

        auto elapsed = (now - start_time_).seconds();

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat &frame = cv_ptr->image;

        // Draw overlays (Rec + Landing/Searching)
       // draw_overlays(frame);

        if (!writer_initialized_ && elapsed >= 2.0) {
            double measured_fps = static_cast<double>(frame_count_) / elapsed;
            RCLCPP_INFO(this->get_logger(), "Measured FPS: %.2f", measured_fps);

            int width = frame.cols;
            int height = frame.rows;

            video_writer_.open(
                temp_path_,
                cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                measured_fps,
                cv::Size(width, height)
            );

            if (!video_writer_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Could not open video file for writing.");
                rclcpp::shutdown();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "VideoWriter initialized. Recording started.");
            writer_initialized_ = true;
        }

        if (writer_initialized_) {
            video_writer_.write(frame);
        }
    }

    // // ---- Draw Overlay ----
    // void draw_overlays(cv::Mat &frame)
    // {
    
    //     // Font settings
    // int font_face = cv::FONT_HERSHEY_SIMPLEX;
    // double font_scale = 0.7;
    // int thickness = 2;
    // cv::Scalar text_color(0, 255, 0); // white

    // // ---- Top-left: "Rec" with timer and red dot ----
    // auto now = this->now();
    // int elapsed_sec = static_cast<int>((now - start_time_).seconds());
    // int minutes = elapsed_sec / 60;
    // int seconds = elapsed_sec % 60;

    // std::ostringstream oss;
    // oss << "Rec [" << std::setfill('0') << std::setw(2) << minutes
    //     << ":" << std::setw(2) << seconds << "]";

    // std::string rec_text = oss.str();
    // cv::Point rec_pos(20, 30);
    // cv::putText(frame, rec_text, rec_pos, font_face, font_scale, text_color, thickness);

    // // Red dot next to "Rec"
    // int dot_radius = 6;
    // int dot_x = rec_pos.x + 50 + 80; // adjust based on text width
    // int dot_y = rec_pos.y - 10;
    // cv::circle(frame, cv::Point(dot_x, dot_y), dot_radius, cv::Scalar(0, 0, 255), -1);

    //     // -- Top-center: "Landing" or "Searching"
    //     std::string status_text = landing_status_ ? "Landing" : "Searching";
    //     int text_width = cv::getTextSize(status_text, font_face, font_scale, thickness, 0).width;
    //     cv::Point center_pos((frame.cols - text_width) / 2, 30);
    //     cv::putText(frame, status_text, center_pos, font_face, font_scale, text_color, thickness);
    // }

    // ---- Finalize and Save ----
    void finalize_video(const rclcpp::Time &now)
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }

        int total_seconds = static_cast<int>((now - start_time_).seconds());
        int minutes = total_seconds / 60;
        int seconds = total_seconds % 60;

        std::ostringstream final_name;
        final_name << video_output_dir_ << "/(" << video_index_ << ")_"
                   << minutes << "_min_" << seconds << "_sec.mp4";

        try {
            std::filesystem::rename(temp_path_, final_name.str());
            RCLCPP_INFO(this->get_logger(), "Video saved as: %s", final_name.str().c_str());
        } catch (const std::filesystem::filesystem_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to rename video file: %s", e.what());
        }

        video_index_++;
    }
};

// ---- Main Function ----
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRecorder>());
    rclcpp::shutdown();
    return 0;
}
