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
    CameraRecorder()
        : Node("camera_recorder"),
          debounce_duration_(rclcpp::Duration::from_seconds(0.5)),
          suppression_duration_(rclcpp::Duration::from_seconds(10.0))
    {
        // Declare parameters and override durations if needed
        this->declare_parameter<double>("debounce_duration", 0.5);
        this->declare_parameter<double>("suppression_duration", 10.0);

        double debounce_sec = this->get_parameter("debounce_duration").as_double();
        double suppression_sec = this->get_parameter("suppression_duration").as_double();

        debounce_duration_ = rclcpp::Duration::from_seconds(debounce_sec);
        suppression_duration_ = rclcpp::Duration::from_seconds(suppression_sec);

        RCLCPP_INFO(this->get_logger(), "Debounce duration: %.2f seconds", debounce_sec);
        RCLCPP_INFO(this->get_logger(), "Suppression duration: %.2f seconds", suppression_sec);

        std::string output_folder = "/home/pi/Video";
        std::filesystem::create_directories(output_folder);

        video_output_dir_ = output_folder;
        video_index_ = get_next_video_index(video_output_dir_);
        temp_path_ = video_output_dir_ + "/temp_video.mp4";

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/aruco_detection", 10,
            std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1));

        armed_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/land_armed_status", 10,
            std::bind(&CameraRecorder::armed_status_callback, this, std::placeholders::_1));

        disarmed_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/land_disarmed_status", 10,
            std::bind(&CameraRecorder::disarmed_status_callback, this, std::placeholders::_1));

        ch8_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/rcin_channel_8", 10,
            std::bind(&CameraRecorder::ch8_status_callback, this, std::placeholders::_1));

        recording_active_ = false;
        writer_initialized_ = false;
        timing_started_ = false;
        frame_count_ = 0;

        last_armed_trigger_time_ = this->now();
        last_disarmed_trigger_time_ = this->now();
        last_ch8_trigger_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "CameraRecorder node initialized.");
    }

private:
    // ---- Subscriptions ----
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarmed_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ch8_status_sub_;

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

    // ---- Trigger Debounce ----
    bool prev_armed_status_ = false;
    bool prev_disarmed_status_ = false;
    bool prev_ch8_status_ = false;

    rclcpp::Time last_armed_trigger_time_;
    rclcpp::Time last_disarmed_trigger_time_;
    rclcpp::Time last_ch8_trigger_time_;

    rclcpp::Duration debounce_duration_;
    rclcpp::Duration suppression_duration_;

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

    void armed_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if (!prev_armed_status_ && msg->data) {
            if ((now - last_armed_trigger_time_) > debounce_duration_ &&
                (now - last_armed_trigger_time_) > suppression_duration_) {

                RCLCPP_INFO(this->get_logger(), "Land armed status: TRUE. Starting recording.");
                start_recording();
                last_armed_trigger_time_ = now;

            } else {
                RCLCPP_WARN(this->get_logger(), "Armed trigger suppressed.");
            }
        }
        prev_armed_status_ = msg->data;
    }

    void disarmed_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if (!prev_disarmed_status_ && msg->data && recording_active_) {
            if ((now - last_disarmed_trigger_time_) > debounce_duration_ &&
                (now - last_disarmed_trigger_time_) > suppression_duration_) {

                RCLCPP_INFO(this->get_logger(), "Land disarmed status: TRUE. Stopping and saving recording.");
                stop_and_save_recording(now);
                last_disarmed_trigger_time_ = now;

            } else {
                RCLCPP_WARN(this->get_logger(), "Disarmed trigger suppressed.");
            }
        }
        prev_disarmed_status_ = msg->data;
    }

    void ch8_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if (!prev_ch8_status_ && msg->data && recording_active_) {
            if ((now - last_ch8_trigger_time_) > debounce_duration_ &&
                (now - last_ch8_trigger_time_) > suppression_duration_) {

                RCLCPP_INFO(this->get_logger(), "RC channel 8 triggered. Stopping and saving recording.");
                stop_and_save_recording(now);
                last_ch8_trigger_time_ = now;

            } else {
                RCLCPP_WARN(this->get_logger(), "CH8 trigger suppressed.");
            }
        }
        prev_ch8_status_ = msg->data;
    }

    void start_recording()
    {
        recording_active_ = true;
        writer_initialized_ = false;
        frame_count_ = 0;
        timing_started_ = false;
        temp_path_ = video_output_dir_ + "/temp_video.mp4";
        start_time_ = this->now();
    }

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

        video_index_++;
        recording_active_ = false;
        writer_initialized_ = false;
        timing_started_ = false;
    }

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

// ---- Main Function ----
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRecorder>());
    rclcpp::shutdown();
    return 0;
}

