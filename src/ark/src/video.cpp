#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <regex>
#include <chrono>
#include <iomanip>

class CameraRecorder : public rclcpp::Node
{
public:
    CameraRecorder()
        : Node("camera_recorder"),
          recording_active_(false),
          writer_initialized_(false),
          timing_started_(false),
          frame_count_(0),
          prev_flight_mode_(""),
          prev_armed_status_(false)
    {
        std::string output_folder = "/home/pi/Video";
        std::filesystem::create_directories(output_folder);

        video_output_dir_ = output_folder;
        video_index_ = get_next_video_index(video_output_dir_);
        temp_path_ = video_output_dir_ + "/temp_video.mp4";

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aruco_detection", 10,
            std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1));

        flight_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/flight_mode", 10,
            std::bind(&CameraRecorder::flight_mode_callback, this, std::placeholders::_1));

        armed_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/armed_status", 10,
            std::bind(&CameraRecorder::armed_status_callback, this, std::placeholders::_1));

        rec_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/recording_status", 10);

        RCLCPP_INFO(this->get_logger(), "CameraRecorder node initialized.");
    }

private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_status_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rec_status_pub_;

    // Video recording
    cv::VideoWriter video_writer_;
    std::string temp_path_;
    std::string video_output_dir_;
    int video_index_;

    // State tracking
    bool recording_active_;
    bool writer_initialized_;
    bool timing_started_;
    int frame_count_;
    std::string prev_flight_mode_;
    bool prev_armed_status_;
    rclcpp::Time start_time_;

    // --- Get next index for video file ---
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

    // --- Flight mode callback ---
    void flight_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string current_mode = msg->data;

        if (prev_flight_mode_ != "LAND" && current_mode == "LAND" && prev_armed_status_ && !recording_active_) {
            RCLCPP_INFO(this->get_logger(), "Flight mode changed to LAND. Starting recording.");
            start_recording();
        }
        else if (prev_flight_mode_ == "LAND" &&
                 (current_mode == "LOITER" || current_mode == "ALTHOLD")) {
            if (recording_active_) {
                RCLCPP_WARN(this->get_logger(), "Mode changed from LAND to %s. Discarding video.", current_mode.c_str());
                discard_recording();
            }
        }

        prev_flight_mode_ = current_mode;
    }

    void publish_recording_status(bool status)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = status;
        rec_status_pub_->publish(msg);
    }

    // --- Armed status callback ---
    void armed_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool current_armed = msg->data;

        if (prev_flight_mode_ == "LAND" && prev_armed_status_ && !current_armed && recording_active_) {
            RCLCPP_INFO(this->get_logger(), "Drone disarmed in LAND mode. Saving video.");
            stop_and_save_recording(this->now());
        }

        prev_armed_status_ = current_armed;
    }

    // --- Start recording ---
    void start_recording()
    {
        recording_active_ = true;
        writer_initialized_ = false;
        frame_count_ = 0;
        timing_started_ = false;
        temp_path_ = video_output_dir_ + "/temp_video.mp4";
        start_time_ = this->now();

        publish_recording_status(true);
    }

    // --- Stop and save video ---
    void stop_and_save_recording(const rclcpp::Time &now)
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

        publish_recording_status(false);
        video_index_++;
        recording_active_ = false;
        writer_initialized_ = false;
        timing_started_ = false;
    }

    // --- Discard temp video ---
    void discard_recording()
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }

        try {
            if (std::filesystem::exists(temp_path_)) {
                std::filesystem::remove(temp_path_);
                RCLCPP_INFO(this->get_logger(), "Temporary video discarded.");
            }
        } catch (const std::filesystem::filesystem_error &e) {
            RCLCPP_ERROR(this->get_logger(), "Error deleting temp video: %s", e.what());
        }

        publish_recording_status(false);
        recording_active_ = false;
        writer_initialized_ = false;
        timing_started_ = false;
    }

    // --- Image stream callback ---
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
};

// --- Main ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRecorder>());
    rclcpp::shutdown();
    return 0;
}

