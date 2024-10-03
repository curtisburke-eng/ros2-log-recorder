#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/TFMessage.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <filesystem>

class LogManager : public rclcpp::Node {
public:
    LogManager()
        : Node("log_manager"), recording_(false) {
        
        // Service for starting recording
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_log_recording",
            std::bind(&LogManager::startRecording, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Service for stopping recording
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_log_recording",
            std::bind(&LogManager::stopRecording, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscribe to multiple topics
        auto tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                if (recording_) {
                    recorder_.write(*msg);
                    RCLCPP_INFO(this->get_logger(), "Recorded tf message");
                }
            });

        auto tf_static_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf_static", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                if (recording_) {
                    recorder_.write(*msg);
                    RCLCPP_INFO(this->get_logger(), "Recorded tf_static message");
                }
            });

        RCLCPP_INFO(this->get_logger(), "Log Manager Node Initialized");
    }

private:
    void startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (recording_) {
            response->success = false;
            response->message = "Recording is already in progress.";
            return;
        }

        // Generate unique filename
        std::string filename = generateUniqueFilename();
        recorder_ = std::make_shared<rosbag2_cpp::LogManager>(filename);
        recorder_->start();

        recording_ = true;
        response->success = true;
        response->message = "Started recording to " + filename;
        RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename.c_str());
    }

    void stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!recording_) {
            response->success = false;
            response->message = "No recording is in progress.";
            return;
        }

        recorder_->stop();
        recording_ = false;

        response->success = true;
        response->message = "Stopped recording.";
        RCLCPP_INFO(this->get_logger(), "Stopped recording.");
    }

    std::string generateUniqueFilename() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "log_recording_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".db3";
        return ss.str();
    }

    bool recording_;
    std::shared_ptr<rosbag2_cpp::LogManager> recorder_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogManager>());
    rclcpp::shutdown();
    return 0;
}
